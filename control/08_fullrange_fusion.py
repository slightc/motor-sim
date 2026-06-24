import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))

# -*- coding: utf-8 -*-
"""全速域融合观测器(改进版): 方波HFI+交叉饱和补偿(低速) ⊕ EKF(高速)
   + 降阶负载转矩观测器。苛刻剖面: 低速高负载→高速→过渡区负载阶跃，板级噪声"""
from motorsim_core import *
from motorsim_sensors import *
import numpy as np, math
from dataclasses import replace

cfg=MotorConfig(electrical=ElectricalParams(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4,
                k_cross=-3.2e-4,i_pm_sat=4.0,i_knee_sat=6.0),thermal=ThermalParams(enabled=False))
lim=InverterLimits(24,2.5); DT=10e-6
p=cfg.electrical.p; Ld,Lq=cfg.electrical.Ld,cfg.electrical.Lq
L=(Ld+Lq)/2; psi=cfg.electrical.psi0; J=cfg.mechanical.J; B=cfg.mechanical.B; kc=cfg.electrical.k_cross
F_PWM=10000.0; T_PWM=int((1/F_PWM)/DT); V_H=6.0; W_LO,W_HI=8.0,20.0

class EKF:
    def __init__(s,R):
        s.R=R;s.x=np.array([0.,0.,0.,0.]);s.P=np.diag([1.,1.,1e3,10.])
        s.Q=np.diag([1e-1,1e-1,5e4,1e-2])*DT;s.Rn=np.diag([2.5e-5,2.5e-5]);s.H=np.array([[1.,0,0,0],[0,1.,0,0]])
    def step(s,va,vb,ia,ib):
        R=s.R;i0,i1,w,th=s.x;sn,c=math.sin(th),math.cos(th)
        xp=np.array([i0+(va-R*i0+w*psi*sn)/L*DT,i1+(vb-R*i1-w*psi*c)/L*DT,w,th+w*DT])
        A=np.array([[-R/L,0,psi*sn/L,w*psi*c/L],[0,-R/L,-psi*c/L,w*psi*sn/L],[0,0,0,0],[0,0,1,0]])
        F=np.eye(4)+A*DT;Pp=F@s.P@F.T+s.Q
        K=Pp@s.H.T@np.linalg.inv(s.H@Pp@s.H.T+s.Rn);s.x=xp+K@(np.array([ia,ib])-s.H@xp)
        s.x[3]=(s.x[3]+math.pi)%(2*math.pi)-math.pi;s.P=(np.eye(4)-K@s.H)@Pp
        return s.x[3],s.x[2]

class HFI:
    def __init__(s,th0): s.th=th0;s.w=0.;s.pll=0.;s.sign=1;s.iqp=None;s.Kp=2000.;s.Ki=6e4
    def demod(s,ial,ibe,dtp):
        i_d,i_q=park(ial,ibe,s.th)
        if s.iqp is not None:
            err=(i_q-s.iqp)*s.sign; s.pll+=s.Ki*err*dtp; s.w=s.Kp*err+s.pll
        s.iqp=i_q
    def corrected(s,ial,ibe):
        _,i_q=park(ial,ibe,s.th); phi=0.5*math.atan(2*(kc*i_q)/(Ld-Lq))   # 交叉饱和补偿
        return (s.th-phi)%(2*math.pi)

class LoadDOB:   # 降阶扰动观测器(两低通)
    def __init__(s,bw): s.p=bw;s.a=0.;s.b=0.
    def step(s,Te,w):
        s.a+=s.p*((Te-B*w)-s.a)*DT; s.b+=s.p*((s.p*J*w)-s.b)*DT
        return s.a+s.b-s.p*J*w

def weight_emf(w_elec, i_mag):
    # 负载自适应：反电动势可观测性 = ωe·ψ /(R·|i|)。比值高→信EKF；低→信HFI
    ratio=abs(w_elec)*psi/(cfg.electrical.R0*i_mag+0.05)
    w_emf=float(np.clip((ratio-0.8)/(2.0-0.8),0,1))
    return 1.0-w_emf    # 返回HFI权重
def cblend(a,b,wa): return math.atan2(wa*math.sin(a)+(1-wa)*math.sin(b),wa*math.cos(a)+(1-wa)*math.cos(b))

plant=MotorPlant(cfg, init_state=MotorState(omega_m=3.0))
foc=FieldWeakeningFOC(cfg,lim)
sens=SensorSuite(current=IdealCurrentSensor(),position=IdealEncoder())
cs=CurrentSensor(f_sample=100000,adc_bits=12,i_range=3.27,noise_std=0.003,seed=11)
ekf=EKF(cfg.electrical.R0); ekf.x[2]=p*3.0
hfi=HFI(0.0); dob=LoadDOB(120.0)
th_f=0.0; w_f=p*3.0; vprev=(0,0); Tpwm_s=T_PWM*DT; iqf=0.0; idf=0.0
log={k:[] for k in ['t','spd','sp','w','e_ekf','e_hfi','e_f','TL','TLt']}

def profile(t):
    if t<0.3: return 3.0
    if t<0.55: return 3.0+(45-3)*(t-0.3)/0.25
    if t<0.85: return 45.0
    if t<1.1: return 45.0+(3-45)*(t-0.85)/0.25
    return 3.0
def load(t):
    b=0.18 if (t<0.3 or t>=1.1) else 0.04   # 低速段高负载
    if 0.6<=t<0.85: b+=0.10                  # 高速负载阶跃
    if 0.93<=t<1.05: b+=0.10                 # 过渡区负载阶跃
    return b

for k in range(int(1.4/DT)):
    t=plant.t; to=plant.observe(); meas=sens.measure(to,DT)
    sp=profile(t); TLt=load(t); cmd=foc.compute(meas,sp,DT)
    aw=abs(w_f)/p; inj=aw<25.0
    if k%T_PWM==0: hfi.sign=-hfi.sign
    vinj_al,vinj_be=(inv_park(V_H*hfi.sign,0.0,hfi.th) if inj else (0.,0.))
    cal,cbe=clarke(cmd.v_a,cmd.v_b,cmd.v_c)
    plant.step(replace(MotorInput(*inv_clarke(cal+vinj_al,cbe+vinj_be)),t_load=TLt),DT)
    ia_m,ib_m,ic_m=cs.read(plant.observe(),DT); ial,ibe=clarke(ia_m,ib_m,ic_m)
    th_ekf,w_ekf=ekf.step(vprev[0],vprev[1],ial,ibe); vprev=(cal+vinj_al,cbe+vinj_be)
    if inj:
        hfi.th=(hfi.th+hfi.w*DT)%(2*math.pi)
        if k%T_PWM==T_PWM-1: hfi.demod(ial,ibe,Tpwm_s)
        th_hfi_c=hfi.corrected(ial,ibe)
    else:
        hfi.th=th_f; hfi.w=w_f; th_hfi_c=th_f
    i_mag=math.hypot(ial,ibe)
    ww=weight_emf(w_f,i_mag); th_f=cblend(th_hfi_c,th_ekf,ww); w_f=ww*hfi.w+(1-ww)*w_ekf
    i_d_m,i_q_m=park(ial,ibe,th_f); idf+=200*DT*(i_d_m-idf); iqf+=200*DT*(i_q_m-iqf)  # 低通取基波
    Te=1.5*p*(psi*iqf+(Ld-Lq)*idf*iqf); TLh=dob.step(Te,w_f/p)
    if k%100==0:
        to2=plant.observe()
        def er(th): return (th-to2.theta_e+math.pi)%(2*math.pi)-math.pi
        log['t'].append(t);log['spd'].append(to2.state.omega_m);log['sp'].append(sp);log['w'].append(ww)
        log['e_ekf'].append(math.degrees(er(th_ekf)));log['e_hfi'].append(math.degrees(er(th_hfi_c)))
        log['e_f'].append(math.degrees(er(th_f)));log['TL'].append(TLh);log['TLt'].append(TLt)
for kk in log: log[kk]=np.array(log[kk])
np.savez('/tmp/fullrange.npz',**log)
msk=log['t']>0.05   # 排除启动暂态
print("=== 全速域融合观测器(板级3mA噪声, 低速高负载) ===")
for lo,hi,lab in [(0,8,'低速<8(高载)'),(8,20,'过渡8-20'),(20,99,'高速>20')]:
    m=(log['spd']>=lo)&(log['spd']<hi)&msk
    if m.sum()>0:
        print(f"{lab:>14}: 融合|err|={np.abs(log['e_f'][m]).mean():4.1f}° | EKF={np.abs(log['e_ekf'][m]).mean():5.1f}° HFI={np.abs(log['e_hfi'][m]).mean():4.1f}°")
print(f"\n融合全程(除启动)最大误差={np.abs(log['e_f'][msk]).max():.1f}°")
print(f"负载观测末值 真实/估计={log['TLt'][-1]:.2f}/{log['TL'][-1]:.2f}Nm")
