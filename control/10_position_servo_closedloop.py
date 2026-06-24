import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))

# -*- coding: utf-8 -*-
"""闭环无感位置伺服: 方波HFI(符号对齐+基波抵消解调)+交叉饱和补偿+级联位置环"""
from motorsim_core import *
from motorsim_sensors import *
import numpy as np, math
from dataclasses import replace

cfg=MotorConfig(electrical=ElectricalParams(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4,
                k_cross=-3.2e-4,i_pm_sat=4.0,i_knee_sat=6.0),thermal=ThermalParams(enabled=False))
lim=InverterLimits(24,2.5); DT=10e-6; p=cfg.electrical.p
Ld,Lq,kc=cfg.electrical.Ld,cfg.electrical.Lq,cfg.electrical.k_cross
F_PWM=10000.0; T_PWM=int((1/F_PWM)/DT); V_H=6.0; STEP_T=0.35

def run(targets_deg, TL=0.08):
    plant=MotorPlant(cfg, init_state=MotorState(omega_m=0.,theta_m=0.))
    cs=CurrentSensor(f_sample=100000,adc_bits=12,i_range=3.27,noise_std=0.003,seed=5)
    th=0.;thc=0.;w=0.;pll=0.;sign=1;iqp=None;dacc=0.;dcnt=0
    Kp_pll,Ki_pll=2500.,8e4
    iid=iiq=0.; spd_i=0.
    kpp=30.; kpw=0.6; kiw=10.; kp_i,ki_i=10.,2500.
    log={'t':[],'ref':[],'true':[],'iq':[],'ehat':[]}
    n=int(len(targets_deg)*STEP_T/DT)
    for k in range(n):
        t=plant.t; to=plant.observe()
        ia_m,ib_m,ic_m=cs.read(to,DT); ial,ibe=clarke(ia_m,ib_m,ic_m)
        if k%T_PWM==0: sign=-sign                      # 周期初翻转
        vinj_al,vinj_be=inv_park(V_H*sign,0.,th)       # 注入(同一符号)
        th=(th+w*DT)%(2*math.pi); thc+=w*DT
        if k%T_PWM==T_PWM-1:                            # 周期末解调(同一符号)
            _,iq_s=park(ial,ibe,th)
            if iqp is not None:
                d=(iq_s-iqp)*sign; dacc+=d; dcnt+=1
                if dcnt>=2: err=dacc/2; pll+=Ki_pll*err*(2*T_PWM*DT); w=Kp_pll*err+pll; dacc=0;dcnt=0
            iqp=iq_s
        # 反馈:交叉饱和补偿后的连续电角
        _,iqc=park(ial,ibe,th); phi=0.5*math.atan(2*(kc*iqc)/(Ld-Lq))
        th_e_fb=thc-phi; th_m_fb=th_e_fb/p; w_m_fb=w/p
        ref=math.radians(targets_deg[min(int(t/STEP_T),len(targets_deg)-1)])
        # 级联 位置->速度->电流
        w_ref=max(-20,min(20,kpp*(ref-th_m_fb)))
        ew=w_ref-w_m_fb; iq_ref=kpw*ew+kiw*spd_i
        if abs(iq_ref)<lim.i_max: spd_i+=ew*DT
        iq_ref=max(-lim.i_max,min(lim.i_max,iq_ref))
        th_ctrl=th_e_fb%(2*math.pi)
        i_d,i_q=park(ial,ibe,th_ctrl); eid,eiq=0-i_d,iq_ref-i_q
        vd=kp_i*eid+ki_i*iid; vq=kp_i*eiq+ki_i*iiq
        vs=math.hypot(vd,vq)
        if vs>lim.v_max: sc=lim.v_max/vs;vd*=sc;vq*=sc
        else: iid+=eid*DT;iiq+=eiq*DT
        val,vbe=inv_park(vd,vq,th_ctrl)
        plant.step(replace(MotorInput(*inv_clarke(val+vinj_al,vbe+vinj_be)),t_load=TL),DT)
        if k%100==0:
            to2=plant.observe(); tm=math.degrees(to2.state.theta_m)
            log['t'].append(t);log['ref'].append(math.degrees(ref));log['true'].append(tm)
            log['iq'].append(i_q); log['ehat'].append(math.degrees((th_e_fb-to2.theta_e+math.pi)%(2*math.pi)-math.pi))
    return {k:np.array(v) for k,v in log.items()}

targets=[0,3,8,5,6]
d=run(targets); np.savez('/tmp/pos2.npz',**d)
print("=== 闭环无感位置伺服(符号对齐+基波抵消) ===")
for i,tgt in enumerate(targets):
    lo=i*STEP_T+0.28; hi=i*STEP_T+0.35; m=(d['t']>=lo)&(d['t']<hi)
    if m.sum()>0:
        err=d['true'][m]-tgt
        print(f"  目标{tgt}°机械: 误差={err.mean():+.2f}°(±{err.std():.2f}) | HFI估计误差(电)={d['ehat'][m].mean():+.1f}° 保持iq={d['iq'][m].mean():.2f}A")
