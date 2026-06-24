import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))
# -*- coding: utf-8 -*-
"""反电动势无感 FOC：PLL 跟踪反电动势相位（中高速有效，低速失效）。"""
from motorsim_core import *
from motorsim_sensors import *
import math
from dataclasses import replace

cfg=MotorConfig(electrical=ElectricalParams(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4),
                thermal=ThermalParams(enabled=False))
lim=InverterLimits(24,2.5); DT=20e-6; SPD=60.0
plant=MotorPlant(cfg, init_state=MotorState(omega_m=SPD))
sfoc=SensorlessFOC(cfg,lim)
sfoc.obs.theta=0.0; sfoc.obs.omega_e=cfg.electrical.p*SPD; sfoc.obs.pll_i=cfg.electrical.p*SPD
sens=SensorSuite(current=IdealCurrentSensor(), position=IdealEncoder())
errs=[]
for k in range(int(0.4/DT)):
    t=plant.t; to=plant.observe(); meas=sens.measure(to,DT)
    cmd=sfoc.compute(meas,SPD,DT)
    plant.step(replace(IdealInverter().apply(cmd,to,DT),t_load=0.10),DT)
    if t>0.3:
        e=(sfoc.obs.theta-to.theta_e+math.pi)%(2*math.pi)-math.pi
        errs.append(abs(math.degrees(e)))
print(f"反电动势无感@{SPD}rad/s: 位置误差RMS={ (sum(e*e for e in errs)/len(errs))**0.5:.2f}°(电)")
print("注意: 低速(反电动势弱)与参数失配下会失锁——见 05_ekf / 07_lowspeed")
