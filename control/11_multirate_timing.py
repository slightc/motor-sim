import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))
# -*- coding: utf-8 -*-
"""多速率时序与计算延迟演示（Simulator 新增 f_ctrl / control_delay）。

两件事：
  ① 计算延迟（control_delay）：复现真实数字 FOC 采样→计算→下周期更新的 z^-N 延迟。
     在高带宽电流内环上扫描延迟，可见超调随延迟单调上升、稳定裕度下降——这是单速率
     理想仿真（控制=物理步、瞬时施加）无法暴露的效应。
  ② 多速率编排（f_ctrl + 物理细 dt + 速度环 decimation）：物理用细 dt 积分并过采样
     PWM 载波，控制器按 f_ctrl 执行、指令零阶保持(ZOH)，速度外环按分频慢速运行。

旧行为不变：f_ctrl=None、control_delay=0 即单速率、零延迟，与改动前逐位一致。
"""
from motorsim_core import (
    MotorConfig, ElectricalParams, MechanicalParams, ThermalParams,
    MotorPlant, MotorState, InverterLimits, Simulator, Recorder,
    VoltageCommand, _dq_to_abc,
)
from motorsim_inverter import SVPWMInverter
from motorsim_sensors import SensorSuite, IdealCurrentSensor, Encoder, IdealEncoder
import math

cfg=MotorConfig(electrical=ElectricalParams(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4),
                thermal=ThermalParams(enabled=False))
lim=InverterLimits(24,2.5)


# ======================================================================
# ① 计算延迟对电流内环的影响
# ======================================================================
class CurrentLoopPI:
    """纯 q 轴电流环（id 参考=0），IMC 整定到高带宽，用于凸显计算延迟。"""
    def __init__(self, cfg, limits, wc):
        e=cfg.electrical
        self.kp=wc*e.Lq; self.ki=wc*e.R0          # IMC: kp=ωc·L, ki=ωc·R
        self.lim=limits; self.iid=self.iiq=0.0
    def compute(self, meas, setpoint, dt):
        Vmax=self.lim.v_max
        e_id=0.0-meas.i_d; e_iq=setpoint-meas.i_q
        v_d=self.kp*e_id+self.ki*self.iid; v_q=self.kp*e_iq+self.ki*self.iiq
        vs=math.hypot(v_d,v_q)
        if vs>Vmax: sc=Vmax/vs; v_d*=sc; v_q*=sc
        else: self.iid+=e_id*dt; self.iiq+=e_iq*dt
        va,vb,vc=_dq_to_abc(v_d,v_q,meas.theta_e)
        return VoltageCommand(va,vb,vc)

def current_step(delay, f_ctrl=10000.0, wc=2*math.pi*1200, iq_ref=1.5, dur=8e-3):
    # 大转动惯量使转子几乎不动 -> 纯 RL 电气环，反电动势可忽略，单看电流环时序
    big_J=MotorConfig(electrical=cfg.electrical,
                      mechanical=MechanicalParams(J=1.0,B=0.0,Tc=0.0),
                      thermal=ThermalParams(enabled=False))
    plant=MotorPlant(big_J, init_state=MotorState(omega_m=0.0,theta_m=0.0))
    ctrl=CurrentLoopPI(big_J, lim, wc)
    inv=SVPWMInverter(v_dc=24.0, f_pwm=f_ctrl)
    sens=SensorSuite(current=IdealCurrentSensor(), position=IdealEncoder())
    rec=Recorder(["t","i_q"]);
    sim=Simulator(plant, ctrl, inverter=inv, sensors=sens, observers=[rec])
    sim.run(dur, dt=2e-6, reference=lambda t: iq_ref, f_ctrl=f_ctrl, control_delay=delay)
    iq=rec.array("i_q")
    overshoot=(iq.max()/iq_ref-1)*100
    return overshoot

print("=== ① 计算延迟对电流内环的影响 (iq 阶跃 0->1.5A, 控制10kHz, 物理2µs) ===")
print("   电流环 IMC 整定 ωc=2π·1200; 延迟以 PWM/控制周期(100µs)为单位\n")
for d in (0, 1, 2, 3):
    os_pct=current_step(d)
    tag="单速率理想(无延迟)" if d==0 else "延迟%d周期(%dµs)"%(d, d*100)
    bar="#"*max(0,int(os_pct))
    print("   control_delay=%d %-18s 超调=%5.1f%% %s" % (d, tag, os_pct, bar))
print("   -> 延迟越大超调越大，逼近临界稳定；理想单速率(d=0)看不到这一退化。\n")


# ======================================================================
# ② 多速率编排：物理细 dt + 控制 f_ctrl + 速度环 decimation
# ======================================================================
class MultiRateFOC:
    """级联有感 FOC。电流环每个控制周期执行；速度外环每 speed_decim 个周期更新一次
    (decimation)，其余周期保持上次 iq 指令——对应真实 MCU 内环快、外环慢。"""
    def __init__(self, cfg, limits, speed_decim=10):
        self.cfg=cfg; self.lim=limits; self.speed_decim=speed_decim
        self.kp_w,self.ki_w=0.6,10.0; self.kp_i,self.ki_i=12.0,3000.0
        self.iw=self.iid=self.iiq=0.0; self.iq_cmd=0.0; self._n=0
    def compute(self, meas, setpoint, dt):
        e=self.cfg.electrical; Vmax,Imax=self.lim.v_max,self.lim.i_max
        if self._n % self.speed_decim == 0:               # 速度外环（分频）
            dts=dt*self.speed_decim                        # 外环真实周期
            e_w=setpoint-meas.omega_m
            iq=self.kp_w*e_w+self.ki_w*self.iw
            if -Imax<iq<Imax: self.iw+=e_w*dts
            self.iq_cmd=max(-Imax,min(Imax,iq))
        self._n+=1
        e_id,e_iq=0.0-meas.i_d, self.iq_cmd-meas.i_q       # 电流内环（每周期）
        v_d=self.kp_i*e_id+self.ki_i*self.iid; v_q=self.kp_i*e_iq+self.ki_i*self.iiq
        vs=math.hypot(v_d,v_q)
        if vs>Vmax: sc=Vmax/vs; v_d*=sc; v_q*=sc
        else: self.iid+=e_id*dt; self.iiq+=e_iq*dt
        va,vb,vc=_dq_to_abc(v_d,v_q,meas.theta_e)
        return VoltageCommand(va,vb,vc)

def speed_run(f_ctrl, dt, speed_decim, delay, dur=0.4):
    plant=MotorPlant(cfg, init_state=MotorState(omega_m=0.0))
    ctrl=MultiRateFOC(cfg, lim, speed_decim=speed_decim)
    inv=SVPWMInverter(v_dc=24.0, f_pwm=20000.0)
    sens=SensorSuite(current=IdealCurrentSensor(), position=Encoder(2500,p=4))
    rec=Recorder(["t","state.omega_m"])
    sim=Simulator(plant, ctrl, inverter=inv, sensors=sens, observers=[rec])
    sim.run(dur, dt=dt, reference=lambda t: 50.0, load=lambda t: 0.10,
            f_ctrl=f_ctrl, control_delay=delay)
    w=rec.array("state.omega_m")
    return w[-200:].mean()

print("=== ② 多速率编排 (目标末速 50 rad/s, 负载 0.10 N·m) ===")
w0=speed_run(f_ctrl=None, dt=20e-6, speed_decim=1, delay=0)
print("   A 单速率   控制=物理步20µs, 速度环=电流环同率 : 末速 %.2f rad/s" % w0)
w1=speed_run(f_ctrl=20000.0, dt=5e-6, speed_decim=10, delay=1)
print("   B 多速率   物理5µs(过采样PWM)/控制20kHz/速度环2kHz/延迟1周期 : 末速 %.2f rad/s" % w1)
print("   -> 两者收敛一致；B 路径下电流环以 PWM 率跑、速度环 1/10 分频、物理过采样载波。")
