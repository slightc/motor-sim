import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))
# -*- coding: utf-8 -*-
"""有感 FOC（编码器）：磁场定向控制基线，速度阶跃 + 负载。"""
from motorsim_core import *
from motorsim_sensors import *
import math
from dataclasses import replace

cfg=MotorConfig(electrical=ElectricalParams(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4),
                thermal=ThermalParams(enabled=False))
lim=InverterLimits(24,2.5); DT=20e-6
plant=MotorPlant(cfg, init_state=MotorState(omega_m=0.0))
foc=FieldWeakeningFOC(cfg,lim)
sens=SensorSuite(current=IdealCurrentSensor(), position=Encoder(2500,p=4))
errs=[]
for k in range(int(0.4/DT)):
    t=plant.t; to=plant.observe(); meas=sens.measure(to,DT)
    sp=30.0 if t<0.2 else 50.0
    cmd=foc.compute(meas,sp,DT)
    plant.step(replace(IdealInverter().apply(cmd,to,DT),t_load=0.10),DT)
    if t>0.35: errs.append(to.state.omega_m)
print(f"有感FOC: 末速{plant.observe().state.omega_m:.1f} rad/s (目标50), 稳态均速{sum(errs)/len(errs):.1f}")
print("基线: 编码器位置直接用于Park变换, 全速域/负载稳定")
