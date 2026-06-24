import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))

# -*- coding: utf-8 -*-
"""I/f 开环 + 主动阻尼：完全不估计位置的可靠兜底（永不发散）"""
from motorsim_core import *
from motorsim_sensors import *
import numpy as np, math
from dataclasses import replace

cfg=MotorConfig(electrical=ElectricalParams(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4,
                k_cross=-3.2e-4,i_pm_sat=4.0,i_knee_sat=6.0),thermal=ThermalParams(enabled=False))
lim=InverterLimits(24,2.5); DT=10e-6; p=cfg.electrical.p; I_CMD=2.2

def run_if(Kd):
    plant=MotorPlant(cfg, init_state=MotorState(omega_m=0.0))
    th=0.; iid=iiq=0.; vqlp=0.
    log={'t':[],'spd':[],'delta':[]}
    for k in range(int(1.0/DT)):
        t=plant.t; to=plant.observe(); we=p*min(3.,3.*t/0.3)
        ial,ibe=clarke(to.i_a,to.i_b,to.i_c); i_d,i_q=park(ial,ibe,th)
        eid,eiq=I_CMD-i_d,-i_q; vd=8*eid+2000*iid; vq=8*eiq+2000*iiq
        vs=math.hypot(vd,vq)
        if vs>lim.v_max: sc=lim.v_max/vs; vd*=sc; vq*=sc
        else: iid+=eid*DT; iiq+=eiq*DT
        vqlp+=30*DT*(vq-vqlp); vqhp=vq-vqlp
        th=(th+(we-Kd*vqhp)*DT)%(2*math.pi)
        Tl=0.05+(0.12 if t>0.5 else 0.)
        plant.step(replace(MotorInput(*inv_clarke(*inv_park(vd,vq,th))),t_load=Tl),DT)
        if k%100==0:
            to2=plant.observe()
            log['t'].append(t); log['spd'].append(to2.state.omega_m)
            log['delta'].append(math.degrees((th-to2.theta_e+math.pi)%(2*math.pi)-math.pi))
    return {k:np.array(v) for k,v in log.items()}

und=run_if(0.0); dmp=run_if(20.0)
np.savez('/tmp/iff.npz',t=und['t'],spd_u=und['spd'],dl_u=und['delta'],spd_d=dmp['spd'],dl_d=dmp['delta'])
print(f"=== I/f 控制(强加{I_CMD}A, 失步转矩{1.5*p*0.03*I_CMD:.2f}Nm) 0.5s负载阶跃 ===")
m=und['t']>0.55
print(f"无阻尼: 阶跃后载角std={und['delta'][m].std():.1f}° 峰{und['delta'][m].max():.0f}° (始终<90°=未失步)")
print(f"主动阻尼: 阶跃后载角std={dmp['delta'][m].std():.1f}° 峰{dmp['delta'][m].max():.0f}°")
print(f"\n全程无位置估计；只要负载<失步转矩，转子靠同步原理跟随，无观测器可发散")
