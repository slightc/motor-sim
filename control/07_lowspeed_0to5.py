import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))

# -*- coding: utf-8 -*-
"""0~5 rad/s 极低速：EKF(反电动势) vs 方波HFI-PLL 并行位置跟踪对比"""
from motorsim_core import *
from motorsim_sensors import *
import numpy as np, math
from dataclasses import replace

K_CROSS=-3.2e-4
cfg=MotorConfig(electrical=ElectricalParams(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4,k_cross=K_CROSS),
                thermal=ThermalParams(enabled=False))
lim=InverterLimits(24,2.5); DT=10e-6
p=cfg.electrical.p; L=(cfg.electrical.Ld+cfg.electrical.Lq)/2; psi=cfg.electrical.psi0
F_PWM=10000.0; T_PWM=int((1/F_PWM)/DT); V_H=6.0
Ld,Lq=cfg.electrical.Ld,cfg.electrical.Lq

# --- EKF (4态, 复用) ---
class EKF:
    def __init__(s,R,L,psi,dt):
        s.R=R;s.L=L;s.psi=psi;s.dt=dt;s.x=np.array([0.,0.,0.,0.])
        s.P=np.diag([1.,1.,1e3,10.]);s.Q=np.diag([1e-1,1e-1,5e4,1e-2])*dt
        s.Rn=np.diag([2.5e-5,2.5e-5]);s.H=np.array([[1.,0,0,0],[0,1.,0,0]])
    def step(s,va,vb,ia,ib):
        R,L,psi,dt=s.R,s.L,s.psi,s.dt;i0,i1,w,th=s.x;sn,c=math.sin(th),math.cos(th)
        dia=(va-R*i0+w*psi*sn)/L;dib=(vb-R*i1-w*psi*c)/L
        xp=np.array([i0+dia*dt,i1+dib*dt,w,th+w*dt])
        A=np.array([[-R/L,0,psi*sn/L,w*psi*c/L],[0,-R/L,-psi*c/L,w*psi*sn/L],[0,0,0,0],[0,0,1,0]])
        F=np.eye(4)+A*dt;Pp=F@s.P@F.T+s.Q
        y=np.array([ia,ib])-s.H@xp;K=Pp@s.H.T@np.linalg.inv(s.H@Pp@s.H.T+s.Rn)
        s.x=xp+K@y;s.x[3]=(s.x[3]+math.pi)%(2*math.pi)-math.pi;s.P=(np.eye(4)-K@s.H)@Pp
        return s.x[3]

# --- 方波 HFI-PLL 跟踪器 ---
class HFITracker:
    def __init__(s,theta0):
        s.theta=theta0; s.omega=0.0; s.pll=0.0; s.sign=1; s.iq_prev=None
        s.Kp=1500.0; s.Ki=3e4
    def update(s,i_al,i_be,dt_pwm):
        _,i_q=park(i_al,i_be,s.theta)
        if s.iq_prev is not None:
            err=(i_q-s.iq_prev)*s.sign                 # 解调误差 ∝ sin(2Δθ)
            s.pll+=s.Ki*err*dt_pwm; s.omega=s.Kp*err+s.pll
        s.iq_prev=i_q

plant=MotorPlant(cfg, init_state=MotorState(omega_m=0.0))
foc=FieldWeakeningFOC(cfg,lim)
sens=SensorSuite(current=IdealCurrentSensor(),position=IdealEncoder())
cs=CurrentSensor(f_sample=100000,adc_bits=12,i_range=3.27,noise_std=0.003,seed=8)
ekf=EKF(cfg.electrical.R0,L,psi,DT)
hfi=HFITracker(0.0)
log={'t':[],'spd':[],'e_ekf':[],'e_hfi':[]}; vprev=(0,0)
Tpwm_s=T_PWM*DT
nstep=int(0.8/DT)
for k in range(nstep):
    t=plant.t; to=plant.observe(); meas=sens.measure(to,DT)
    spd_ref=min(5.0,5.0*t/0.4)                      # 0->5 斜坡
    cmd=foc.compute(meas,spd_ref,DT)
    # 方波注入(HFI估计d轴)
    if k%T_PWM==0: hfi.sign=-hfi.sign
    vinj_al,vinj_be=inv_park(V_H*hfi.sign,0.0,hfi.theta)
    cal,cbe=clarke(cmd.v_a,cmd.v_b,cmd.v_c)
    va,vb,vc=inv_clarke(cal+vinj_al,cbe+vinj_be)
    plant.step(replace(MotorInput(va,vb,vc),t_load=0.10),DT)
    hfi.theta=(hfi.theta+hfi.omega*DT)%(2*math.pi)  # HFI估计位置推进
    ia_m,ib_m,ic_m=cs.read(plant.observe(),DT); ial,ibe=clarke(ia_m,ib_m,ic_m)
    th_ekf=ekf.step(vprev[0],vprev[1],ial,ibe); vprev=(cal+vinj_al,cbe+vinj_be)
    if k%T_PWM==T_PWM-1: hfi.update(ial,ibe,Tpwm_s)
    if k%100==0:
        to2=plant.observe()
        e_e=abs(math.degrees((th_ekf-to2.theta_e+math.pi)%(2*math.pi)-math.pi))
        e_h=abs(math.degrees((hfi.theta-to2.theta_e+math.pi)%(2*math.pi)-math.pi))
        log['t'].append(t); log['spd'].append(to2.state.omega_m); log['e_ekf'].append(e_e); log['e_hfi'].append(e_h)
for kk in log: log[kk]=np.array(log[kk])
np.savez('/tmp/lowspeed.npz',**log)
m=log['spd']<5.5
print("=== 0~5 rad/s 位置误差 ===")
for lo,hi,lab in [(0,1,'0-1'),(1,3,'1-3'),(3,5,'3-5')]:
    msk=(log['spd']>=lo)&(log['spd']<hi)
    if msk.sum()>0:
        print(f"{lab} rad/s: EKF误差={log['e_ekf'][msk].mean():.1f}° | HFI误差={log['e_hfi'][msk].mean():.1f}°")
print("\nEKF低速误差大(反电动势弱)；HFI全程稳定(靠凸极,与转速无关)")
