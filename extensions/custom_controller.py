import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))
# -*- coding: utf-8 -*-
"""扩展示例：自定义控制器。实现 Controller Protocol 即可。
   这里做一个最简的"有感比例位置保持"控制器作为模板。"""
from motorsim_core import *
from motorsim_sensors import *
import math
from dataclasses import replace

class MyHoldController:
    """模板：消费 Measurements，输出 VoltageCommand。把你的算法写在 compute 里。"""
    def __init__(self, cfg, lim, target_e):
        self.cfg=cfg; self.lim=lim; self.target_e=target_e
        self.iid=self.iiq=0.0; self.kp_i,self.ki_i=10.0,2500.0; self.kp_pos=8.0
    def compute(self, meas, setpoint, dt):
        th=meas.theta_e
        i_d,i_q=meas.i_d,meas.i_q
        e=(self.target_e-th+math.pi)%(2*math.pi)-math.pi   # 位置误差(电)
        iq_ref=max(-self.lim.i_max,min(self.lim.i_max,self.kp_pos*e))
        vd=self.kp_i*(0-i_d)+self.ki_i*self.iid
        vq=self.kp_i*(iq_ref-i_q)+self.ki_i*self.iiq
        vs=math.hypot(vd,vq)
        if vs>self.lim.v_max: sc=self.lim.v_max/vs; vd*=sc; vq*=sc
        else: self.iid+=(0-i_d)*dt; self.iiq+=(iq_ref-i_q)*dt
        return VoltageCommand(*inv_clarke(*inv_park(vd,vq,th)))

if __name__=="__main__":
    cfg=MotorConfig(electrical=ElectricalParams(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4),
                    thermal=ThermalParams(enabled=False))
    lim=InverterLimits(24,2.5); DT=20e-6
    plant=MotorPlant(cfg, init_state=MotorState(omega_m=0.0,theta_m=0.2))
    ctl=MyHoldController(cfg,lim,target_e=0.0)
    sens=SensorSuite(IdealCurrentSensor(),IdealEncoder())
    for k in range(int(0.3/DT)):
        to=plant.observe(); meas=sens.measure(to,DT)
        plant.step(replace(IdealInverter().apply(ctl.compute(meas,0,DT),to,DT),t_load=0.0),DT)
    print(f"自定义控制器: 转子从0.2rad保持到 θ_e={plant.observe().theta_e:.3f} (目标0)")
