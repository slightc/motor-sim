import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))

# -*- coding: utf-8 -*-
"""低速高负载无感：交叉饱和导致 HFI 位置误差 ∝ iq，及其补偿"""
from motorsim_core import *
from motorsim_sensors import *
import numpy as np, math
from dataclasses import replace

K_CROSS=-8e-5
cfg=MotorConfig(electrical=ElectricalParams(Ld=4e-3,Lq=6e-3,psi0=0.08,k_cross=K_CROSS),
                thermal=ThermalParams(enabled=False))
lim=InverterLimits(48,18); DT=10e-6
F_H=1000.0; W_H=2*math.pi*F_H; V_H=12.0
Ld,Lq=cfg.electrical.Ld,cfg.electrical.Lq

def phi_sat(iq):
    return 0.5*math.atan(2*K_CROSS*iq/(Ld-Lq))

def run_at_load(Tload):
    plant=MotorPlant(cfg, init_state=MotorState(omega_m=2.0))
    foc=FieldWeakeningFOC(cfg,lim)
    sens=SensorSuite(current=IdealCurrentSensor(),position=IdealEncoder())
    carrier=0.0
    dq_s=dd_s=0.0; iq_acc=0.0; n=0      # 相干累积（整周期，DC×sin 自动抵消）
    t_start=0.30; t_end=0.30+0.200       # 200个载波周期
    for k in range(int(0.55/DT)):
        t=plant.t; true_obs=plant.observe(); meas=sens.measure(true_obs,DT)
        cmd=foc.compute(meas,2.0,DT)
        carrier+=W_H*DT
        vinj_al,vinj_be=inv_park(V_H*math.cos(carrier),0.0,true_obs.theta_e)
        cal,cbe=clarke(cmd.v_a,cmd.v_b,cmd.v_c)
        va,vb,vc=inv_clarke(cal+vinj_al,cbe+vinj_be)
        plant.step(replace(MotorInput(va,vb,vc),t_load=Tload),DT)
        no=plant.observe()
        if t_start<=t<t_end:
            s=math.sin(carrier)
            dq_s+=no.i_q*s; dd_s+=no.i_d*s; iq_acc+=no.i_q; n+=1
    iq=iq_acc/n; ratio=dq_s/dd_s if abs(dd_s)>1e-9 else 0.0
    Ldq_meas=-ratio*Lq
    return iq, math.degrees(0.5*math.atan(2*Ldq_meas/(Ld-Lq)))

print("=== 低速(2rad/s) HFI 位置误差随负载(iq) ===")
print(f"{'负载Nm':>7}{'iq(A)':>8}{'HFI误差':>10}{'补偿后':>9}{'理论φ_sat':>11}")
res=[]
for T in [0.0,1.0,2.0,3.0,4.0]:
    iq,phi_nc=run_at_load(T); phi_th=math.degrees(phi_sat(iq))
    res.append((iq,phi_nc,phi_nc-phi_th,phi_th))
    print(f"{T:>7.1f}{iq:>8.2f}{phi_nc:>8.1f}°{phi_nc-phi_th:>7.1f}°{phi_th:>9.1f}°")
np.savez('/tmp/hfi_cross.npz',res=np.array(res))
print("\n无补偿误差随 iq 增大（交叉饱和）；补偿 φ_sat(iq) 后≈0")
