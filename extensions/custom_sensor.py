import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))
# -*- coding: utf-8 -*-
"""扩展示例：自定义传感器。实现 SensorSuite Protocol 即可。
   这里在官方电流传感器上叠加一个随温度漂移的增益误差作为模板。"""
from motorsim_core import *
from motorsim_sensors import *
import math

class DriftingCurrentSuite:
    """模板：measure(true_obs,dt) -> Measurements。把你的检测物理写在这里。"""
    def __init__(self, position, gain_drift=0.05):
        self.cur=CurrentSensor(f_sample=100000,adc_bits=12,i_range=3.27,noise_std=0.003)
        self.position=position; self.gain_drift=gain_drift
    def measure(self, true_obs, dt):
        ia,ib,ic=self.cur.read(true_obs,dt)
        g=1.0+self.gain_drift            # 模拟温漂增益误差
        ia,ib,ic=ia*g,ib*g,ic*g
        i_al,i_be=clarke(ia,ib,ic)
        th_e,om=self.position.measure_angle(true_obs,dt) if hasattr(self.position,'measure_angle') else (true_obs.theta_e,true_obs.state.omega_m)
        i_d,i_q=park(i_al,i_be,th_e)
        return Measurements(t=true_obs.t,i_a=ia,i_b=ib,i_c=ic,i_d=i_d,i_q=i_q,
                            theta_e=th_e,omega_m=om,theta_e_true=true_obs.theta_e)

if __name__=="__main__":
    cfg=MotorConfig(electrical=ElectricalParams(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4),
                    thermal=ThermalParams(enabled=False))
    plant=MotorPlant(cfg, init_state=MotorState(omega_m=30.0))
    suite=DriftingCurrentSuite(IdealEncoder(), gain_drift=0.05)
    m=suite.measure(plant.observe(),20e-6)
    print(f"自定义传感器(带5%增益漂移): 测得 i_q={m.i_q:.3f} A")
    print("说明: 实现 measure() 即可插入仿真，core/控制器无需改动")
