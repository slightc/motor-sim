# -*- coding: utf-8 -*-
"""
motorsim.core —— 电机仿真核心（只含电机物理 + 编排）

职责边界（严格）：
  core 只负责电机本体物理（电磁/机械/热）与仿真编排。
  逆变器(inverter)、传感器(sensors) 是独立物理扩展模块，不在 core。
  信号链：Controller -> Inverter -> Motor -> Sensors -> Controller
"""
from dataclasses import dataclass, field, replace
from typing import Protocol, Callable, Optional, List
from collections import deque
import math

_S3 = math.sqrt(3.0)

def clarke(a,b,c): return (2/3)*(a-0.5*b-0.5*c), (1/_S3)*(b-c)
def inv_clarke(a,b): return a, -0.5*a+(_S3/2)*b, -0.5*a-(_S3/2)*b
def park(a,b,th):
    c,s=math.cos(th),math.sin(th); return a*c+b*s, -a*s+b*c
def inv_park(d,q,th):
    c,s=math.cos(th),math.sin(th); return d*c-q*s, d*s+q*c


# ---------------- 电机配置 ----------------
@dataclass
class ElectricalParams:
    R0: float=0.2; Ld: float=4.0e-3; Lq: float=6.0e-3; psi0: float=0.08; p: int=4
    a_cu: float=3.9e-3; a_pm: float=-1.1e-3; T0: float=25.0
    kfe_h: float=5.0; kfe_e: float=0.08; kfe_a: float=0.10
    k_cross: float=0.0    # 交叉饱和系数 L_dq=k_cross*i_q（高负载下凸极轴偏移）
    i_pm_sat: float=0.0   # 等效永磁偏置电流（>0启用d轴极性饱和；磁极辨识依据）
    i_knee_sat: float=8.0 # 饱和拐点电流（越小饱和越强）
@dataclass
class MechanicalParams:
    J: float=6.0e-4; B: float=1.5e-4; Tc: float=0.02
@dataclass
class ThermalParams:
    enabled: bool=True
    C_w: float=12.0; C_m: float=45.0; C_s: float=150.0
    R_wm: float=0.30; R_ms: float=0.40; R_sa0: float=0.8; k_cool: float=0.0; T_amb: float=25.0
@dataclass
class MotorConfig:
    electrical: ElectricalParams=field(default_factory=ElectricalParams)
    mechanical: MechanicalParams=field(default_factory=MechanicalParams)
    thermal: ThermalParams=field(default_factory=ThermalParams)
    name: str="PMSM"


# ---------------- 数据接口 ----------------
@dataclass
class MotorState:
    i_alpha: float=0.0; i_beta: float=0.0; omega_m: float=0.0; theta_m: float=0.0
    T_w: float=25.0; T_m: float=25.0; T_s: float=25.0
    def as_tuple(self):
        return (self.i_alpha,self.i_beta,self.omega_m,self.theta_m,self.T_w,self.T_m,self.T_s)

@dataclass
class MotorInput:
    """施加到电机绕组的三相电压（逆变器输出）。"""
    v_a: float=0.0; v_b: float=0.0; v_c: float=0.0; t_load: float=0.0

@dataclass
class Observation:
    """电机物理真值快照。"""
    t: float; state: MotorState
    i_a: float; i_b: float; i_c: float; i_d: float; i_q: float
    torque: float; omega_e: float; theta_e: float; back_emf: float
    R: float; psi: float; p_copper: float; p_iron: float
    i_mag: float; v_mag: float
    T_winding: float; T_magnet: float; T_housing: float

@dataclass
class VoltageCommand:
    """控制器 -> 逆变器的三相参考电压。"""
    v_a: float=0.0; v_b: float=0.0; v_c: float=0.0

@dataclass
class Measurements:
    """传感器 -> 控制器的测量集合（控制器只能看到这些，非真值）。"""
    t: float
    i_a: float; i_b: float; i_c: float          # 电流传感器
    i_d: float; i_q: float                        # 用测量角度 Park 得到
    theta_e: float; omega_m: float                # 位置/速度传感器（有感）
    theta_e_true: float                           # 真电角(逆变器死区/调试用，非控制可用)


# ---------------- 电机物理 ----------------
class MotorPlant:
    def __init__(self, config=None, init_state=None):
        self.cfg=config or MotorConfig()
        th=self.cfg.thermal
        self.state=init_state or MotorState(T_w=th.T_amb,T_m=th.T_amb,T_s=th.T_amb)
        self.t=0.0; self._last_input=MotorInput()
    def R(self,Tw):  e=self.cfg.electrical; return e.R0*(1+e.a_cu*(Tw-e.T0))
    def psi(self,Tm):e=self.cfg.electrical; return e.psi0*(1+e.a_pm*(Tm-e.T0))
    def R_sa(self,w):th=self.cfg.thermal; return th.R_sa0/(1+th.k_cool*abs(w))
    def iron_loss(self,i_d,i_q,we,T_m):
        e=self.cfg.electrical
        psi_s=math.hypot(e.Ld*i_d+self.psi(T_m), e.Lq*i_q); f=abs(we)/(2*math.pi)
        return e.kfe_h*f*psi_s**2 + e.kfe_e*f*f*psi_s**2 + e.kfe_a*f**1.5*psi_s**1.5
    def _deriv(self,s,inp):
        e,mech,th=self.cfg.electrical,self.cfg.mechanical,self.cfg.thermal
        i_al,i_be,wm,thm,Tw,Tm,Ts=s
        we=e.p*wm; R=self.R(Tw); psi=self.psi(Tm); the=e.p*thm
        c2,s2=math.cos(2*the),math.sin(2*the)
        Sig=(e.Ld+e.Lq)/2; Dlt=(e.Ld-e.Lq)/2
        i_d,i_q=park(i_al,i_be,the)
        # d轴极性饱和：增量Ld随(i_d+i_pm)变化，永磁偏置使±不对称（磁极辨识依据）
        if e.i_pm_sat>0:
            base=1.0/(1.0+(e.i_pm_sat/e.i_knee_sat)**2)
            satf=1.0/(1.0+((i_d+e.i_pm_sat)/e.i_knee_sat)**2)
            Ld_eff=e.Ld*satf/base                      # i_d=0时=Ld；+d饱和降低Ld，-d去饱和升高Ld
        else:
            Ld_eff=e.Ld
        Sig=(Ld_eff+e.Lq)/2; Dlt=(Ld_eff-e.Lq)/2
        Ldq=e.k_cross*i_q                              # 交叉饱和：凸极轴偏移源
        # αβ 电感矩阵含交叉项：L=[[Sig+A, B],[B, Sig-A]], A=Δc2-Ldq*s2, B=Δs2+Ldq*c2
        A=Dlt*c2-Ldq*s2; B=Dlt*s2+Ldq*c2; det=Ld_eff*e.Lq-Ldq*Ldq
        v_al,v_be=clarke(inp.v_a,inp.v_b,inp.v_c)
        emf_al=-we*psi*math.sin(the); emf_be=we*psi*math.cos(the)
        dLi_al=we*(-2*Dlt*s2*i_al+2*Dlt*c2*i_be); dLi_be=we*(2*Dlt*c2*i_al+2*Dlt*s2*i_be)
        r_al=v_al-R*i_al-dLi_al-emf_al; r_be=v_be-R*i_be-dLi_be-emf_be
        di_al=((Sig-A)*r_al+(-B)*r_be)/det
        di_be=((-B)*r_al+(Sig+A)*r_be)/det
        Te=1.5*e.p*(psi*i_q+(e.Ld-e.Lq)*i_d*i_q)
        P_fe=self.iron_loss(i_d,i_q,we,Tm)
        T_iron=P_fe/max(abs(wm),1e-3)*(1.0 if wm>=0 else -1.0)
        T_fric=mech.Tc*math.tanh(wm/1e-3)
        dwm=(Te-inp.t_load-mech.B*wm-T_fric-T_iron)/mech.J
        if th.enabled:
            P_cu=1.5*R*(i_al*i_al+i_be*i_be)
            q_wm=(Tw-Tm)/th.R_wm; q_ms=(Tm-Ts)/th.R_ms; q_sa=(Ts-th.T_amb)/self.R_sa(wm)
            dTw=(P_cu-q_wm)/th.C_w; dTm=(P_fe+q_wm-q_ms)/th.C_m; dTs=(q_ms-q_sa)/th.C_s
        else: dTw=dTm=dTs=0.0
        return (di_al,di_be,dwm,wm,dTw,dTm,dTs)
    def step(self,inp,dt):
        self._last_input=inp; s=self.state.as_tuple(); d=self._deriv
        k1=d(s,inp)
        s2=tuple(s[i]+0.5*dt*k1[i] for i in range(7)); k2=d(s2,inp)
        s3=tuple(s[i]+0.5*dt*k2[i] for i in range(7)); k3=d(s3,inp)
        s4=tuple(s[i]+dt*k3[i] for i in range(7));     k4=d(s4,inp)
        ns=tuple(s[i]+(dt/6)*(k1[i]+2*k2[i]+2*k3[i]+k4[i]) for i in range(7))
        self.state=MotorState(*ns); self.t+=dt
        return self.observe()
    def observe(self):
        e=self.cfg.electrical; s=self.state; we=e.p*s.omega_m; the=e.p*s.theta_m
        R=self.R(s.T_w); psi=self.psi(s.T_m)
        i_a,i_b,i_c=inv_clarke(s.i_alpha,s.i_beta); i_d,i_q=park(s.i_alpha,s.i_beta,the)
        Te=1.5*e.p*(psi*i_q+(e.Ld-e.Lq)*i_d*i_q); inp=self._last_input
        return Observation(t=self.t,state=s,i_a=i_a,i_b=i_b,i_c=i_c,i_d=i_d,i_q=i_q,
            torque=Te,omega_e=we,theta_e=the,back_emf=abs(we)*psi,R=R,psi=psi,
            p_copper=1.5*R*(s.i_alpha**2+s.i_beta**2),p_iron=self.iron_loss(i_d,i_q,we,s.T_m),
            i_mag=math.hypot(s.i_alpha,s.i_beta),v_mag=math.hypot(*clarke(inp.v_a,inp.v_b,inp.v_c)),
            T_winding=s.T_w,T_magnet=s.T_m,T_housing=s.T_s)


# ---------------- 模块协议 ----------------
class Controller(Protocol):
    def compute(self, meas: Measurements, setpoint: float, dt: float) -> VoltageCommand: ...
class Inverter(Protocol):
    def apply(self, cmd: VoltageCommand, true_obs: Observation, dt: float) -> MotorInput: ...
class SensorSuite(Protocol):
    def measure(self, true_obs: Observation, dt: float) -> Measurements: ...

@dataclass
class InverterLimits:
    v_dc: float=48.0; i_max: float=18.0
    @property
    def v_max(self): return self.v_dc/_S3

# core 自带的理想默认（避免 core 依赖扩展模块）
class IdealInverter:
    """理想逆变器：参考电压直接施加，无开关/死区。"""
    def apply(self, cmd, true_obs, dt):
        return MotorInput(v_a=cmd.v_a, v_b=cmd.v_b, v_c=cmd.v_c)

class IdealSensors:
    """理想传感器：测量=真值。"""
    def measure(self, obs, dt):
        return Measurements(t=obs.t,i_a=obs.i_a,i_b=obs.i_b,i_c=obs.i_c,
            i_d=obs.i_d,i_q=obs.i_q,theta_e=obs.theta_e,omega_m=obs.state.omega_m,
            theta_e_true=obs.theta_e)


# ---------------- 控制器实现 ----------------
def _dq_to_abc(v_d,v_q,the): return inv_clarke(*inv_park(v_d,v_q,the))

class FieldWeakeningFOC:
    """弱磁 FOC。消费 Measurements（测量电流+测量位置/速度），输出三相参考电压。"""
    def __init__(self,cfg,limits):
        self.cfg=cfg; self.lim=limits
        self.kp_w,self.ki_w=0.6,10.0; self.kp_i,self.ki_i=12.0,3000.0; self.ki_fw=30.0
        self.iw=self.iid=self.iiq=0.0; self.fw_int=0.0; self.id_ref=self.iq_ref=0.0
    @staticmethod
    def _demag(Tm,Id25=-45.0,k=0.006): return Id25*max(0.1,1-k*(Tm-25.0))
    def compute(self,meas,setpoint,dt):
        e=self.cfg.electrical; Vmax,Imax=self.lim.v_max,self.lim.i_max
        e_w=setpoint-meas.omega_m
        iq_cmd=self.kp_w*e_w+self.ki_w*self.iw
        if -Imax<iq_cmd<Imax: self.iw+=e_w*dt
        iq_cmd=max(-Imax,min(Imax,iq_cmd))
        dL=e.Lq-e.Ld; is_=abs(iq_cmd)
        id_mtpa=(e.psi0-math.sqrt(e.psi0**2+8*dL*dL*is_*is_))/(4*dL)
        we=e.p*meas.omega_m
        vs_est=abs(we)*math.hypot(e.Ld*self.id_ref+e.psi0,e.Lq*self.iq_ref)
        e_v=0.95*Vmax-vs_est; self.fw_int+=self.ki_fw*e_v*dt
        id_min=max(-Imax,self._demag(25.0))   # 简化：用环境温（热在core，控制不直接读磁体温）
        self.fw_int=max(id_min,min(0.0,self.fw_int))
        id_ref=max(min(id_mtpa,self.fw_int),id_min)
        iq_lim=math.sqrt(max(0.0,Imax**2-id_ref**2)); iq_ref=max(-iq_lim,min(iq_lim,iq_cmd))
        self.id_ref,self.iq_ref=id_ref,iq_ref
        e_id,e_iq=id_ref-meas.i_d,iq_ref-meas.i_q
        v_d=self.kp_i*e_id+self.ki_i*self.iid; v_q=self.kp_i*e_iq+self.ki_i*self.iiq
        vs=math.hypot(v_d,v_q)
        if vs>Vmax: sc=Vmax/vs; v_d*=sc; v_q*=sc
        else: self.iid+=e_id*dt; self.iiq+=e_iq*dt
        va,vb,vc=_dq_to_abc(v_d,v_q,meas.theta_e)   # 用测量角度变换
        return VoltageCommand(va,vb,vc)


# ---------------- 观测器 ----------------
class Recorder:
    def __init__(self,fields): self.fields=fields; self.data={f:[] for f in fields}
    def __call__(self,obs):
        for f in self.fields:
            o=obs
            for p in f.split('.'): o=getattr(o,p)
            self.data[f].append(o)
    def array(self,f):
        import numpy as np; return np.array(self.data[f])


# ---------------- 仿真编排 ----------------
class Simulator:
    """闭环仿真编排，支持多速率时序（物理细分积分 + 控制按 f_ctrl 执行 + 计算延迟）。

    单速率（默认）：f_ctrl=None、control_delay=0 时，控制每个物理步执行一次、指令
    立即施加，与旧行为完全一致。

    多速率：设定 f_ctrl 后，控制周期 Tc=1/f_ctrl（对齐到 dt 整数倍），物理在两次控制
    更新之间以 dt 细分积分，控制指令零阶保持(ZOH)施加；逆变器/死区仍按物理 dt 解析，
    因此可用细 dt 过采样 PWM 载波。control_delay 以"控制周期"为单位施加 z^-N 延迟，
    复现真实数字 FOC 的采样→计算→下周期更新延迟（典型 1~2 个周期）。
    """
    def __init__(self, plant, controller, inverter=None, sensors=None, observers=None):
        self.plant=plant; self.controller=controller
        self.inverter=inverter or IdealInverter()
        self.sensors=sensors or IdealSensors()
        self.observers=observers or []
    def run(self, duration, dt, reference=lambda t:0.0, load=lambda t:0.0,
            f_ctrl=None, control_delay=0):
        """
        dt            物理积分步长（细）。
        f_ctrl        控制器执行频率(Hz)。None=每个物理步都执行控制（单速率，旧行为）。
        control_delay 计算/更新延迟，单位=控制周期（整数，默认0）。第 k 个控制周期采样
                      算得的指令延迟 control_delay 个周期才施加（z^-N）。
        """
        n_steps=int(round(duration/dt))
        if f_ctrl is None:
            n_sub=1; Tc=dt
        else:
            n_sub=max(1, int(round((1.0/f_ctrl)/dt)))
            Tc=n_sub*dt                                       # 对齐到物理步长整数倍
        delay=int(control_delay)
        # 延迟队列：长度 delay 的零指令预填，append 后 popleft 取出 delay 周期前的指令
        queue=deque((VoltageCommand() for _ in range(delay)), maxlen=delay+1)
        k=0
        while k<n_steps:
            t=self.plant.t
            true_obs=self.plant.observe()
            meas=self.sensors.measure(true_obs, Tc)          # 传感器物理（按控制率采样）
            cmd=self.controller.compute(meas, reference(t), Tc)  # 控制决策（用控制周期 Tc）
            queue.append(cmd); held=queue.popleft()          # 计算/更新延迟 z^-N
            for _ in range(n_sub):                            # 控制周期内细分积分，ZOH 施加
                if k>=n_steps: break
                obs=self.plant.observe()
                v_abc=self.inverter.apply(held, obs, dt)     # 逆变器物理（死区用真值，细 dt）
                v_abc=replace(v_abc, t_load=load(self.plant.t))
                self.plant.step(v_abc, dt)                   # 电机物理
                for ob in self.observers: ob(self.plant.observe())
                k+=1
        return self.plant.observe()


# ---------------- 无感控制器（位置解算在 controller 内部）----------------
class BackEMFObserver:
    """反电动势观测器 + PLL。controller 内部组件，仅用电流测量 i_αβ 与施加电压
    v_αβ 解算转子位置：e_hat = v - R i - L di/dt，再 PLL 锁相。中高速有效。"""
    def __init__(self, R, L, p, kp_pll=300.0, ki_pll=20000.0, f_lp=400.0):
        self.R=R; self.L=L; self.p=p
        self.kp_pll=kp_pll; self.ki_pll=ki_pll
        self.i_prev=None; self.theta=0.0; self.omega_e=0.0; self.pll_i=0.0
        self.e_lp=[0.0,0.0]; self.f_lp=f_lp
    def update(self, i_al, i_be, v_al, v_be, dt):
        if self.i_prev is None: self.i_prev=(i_al,i_be)
        di_al=(i_al-self.i_prev[0])/dt; di_be=(i_be-self.i_prev[1])/dt
        self.i_prev=(i_al,i_be)
        # 反电动势 = 端电压 - 电阻压降 - 电感压降
        e_al=v_al-self.R*i_al-self.L*di_al
        e_be=v_be-self.R*i_be-self.L*di_be
        a=dt/(1/(2*math.pi*self.f_lp)+dt)             # 低通抑制 di/dt 噪声
        self.e_lp[0]+=a*(e_al-self.e_lp[0]); self.e_lp[1]+=a*(e_be-self.e_lp[1])
        ea,eb=self.e_lp
        emag=math.hypot(ea,eb)+1e-6
        # PLL：e=|e|[-sinθ,cosθ]，误差 ∝ sin(θ-θ_est)
        eps=-(ea*math.cos(self.theta)+eb*math.sin(self.theta))/emag
        self.pll_i+=self.ki_pll*eps*dt
        self.omega_e=self.kp_pll*eps+self.pll_i
        self.theta=(self.theta+self.omega_e*dt)%(2*math.pi)
        return self.theta, self.omega_e

class SensorlessFOC:
    """无感 FOC：不依赖位置传感器，仅用电流测量，内部用 BackEMFObserver 解算位置。
    适用中高速（反电动势足够强）。低速需改用高频注入解算（同属 controller 内部）。"""
    def __init__(self, cfg, limits):
        self.cfg=cfg; self.lim=limits
        e=cfg.electrical
        self.obs=BackEMFObserver(R=e.R0, L=e.Lq, p=e.p, f_lp=2000.0)
        self.kp_w,self.ki_w=0.5,8.0; self.kp_i,self.ki_i=12.0,3000.0
        self.iw=self.iid=self.iiq=0.0
        self.v_prev=(0.0,0.0)   # 上次施加电压 αβ（供观测器）
    def compute(self, meas, setpoint, dt):
        e=self.cfg.electrical; Vmax,Imax=self.lim.v_max,self.lim.i_max
        i_al,i_be=clarke(meas.i_a,meas.i_b,meas.i_c)
        # 位置解算（controller 内部，仅用电流+电压指令）
        theta_est,omega_e_est=self.obs.update(i_al,i_be,self.v_prev[0],self.v_prev[1],dt)
        omega_m_est=omega_e_est/e.p
        i_d,i_q=park(i_al,i_be,theta_est)
        # 速度环
        e_w=setpoint-omega_m_est
        iq_cmd=self.kp_w*e_w+self.ki_w*self.iw
        if -Imax<iq_cmd<Imax: self.iw+=e_w*dt
        iq_cmd=max(-Imax,min(Imax,iq_cmd))
        # 电流环
        e_id,e_iq=0.0-i_d,iq_cmd-i_q
        v_d=self.kp_i*e_id+self.ki_i*self.iid; v_q=self.kp_i*e_iq+self.ki_i*self.iiq
        vs=math.hypot(v_d,v_q)
        if vs>Vmax: sc=Vmax/vs; v_d*=sc; v_q*=sc
        else: self.iid+=e_id*dt; self.iiq+=e_iq*dt
        v_al,v_be=inv_park(v_d,v_q,theta_est)
        self.v_prev=(v_al,v_be)
        va,vb,vc=inv_clarke(v_al,v_be)
        return VoltageCommand(va,vb,vc)
