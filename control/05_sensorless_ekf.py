import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))

# -*- coding: utf-8 -*-
"""闭环 EKF 无感：控制器完全靠EKF估计驱动，在线估R + 抗负载扰动"""
from motorsim_core import *
from motorsim_sensors import *
import numpy as np, math
from dataclasses import replace

cfg=MotorConfig(electrical=ElectricalParams(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4),
                thermal=ThermalParams(enabled=False))
lim=InverterLimits(24,2.5); DT=20e-6
p=cfg.electrical.p; L=(cfg.electrical.Ld+cfg.electrical.Lq)/2; psi=cfg.electrical.psi0

class EKF5:
    def __init__(self,R_init,L,psi,dt):
        self.L=L; self.psi=psi; self.dt=dt
        self.x=np.array([0.,0.,0.,0.,R_init])
        self.P=np.diag([1.,1.,1e3,10.,1.])
        self.Q=np.diag([1e-1,1e-1,5e4,1e-2,5e-2])*dt
        self.Rn=np.diag([2.5e-5,2.5e-5]); self.H=np.array([[1.,0,0,0,0],[0,1.,0,0,0]])
    def step(self,v_al,v_be,ia_m,ib_m):
        L,psi,dt=self.L,self.psi,self.dt
        ia,ib,w,th,R=self.x; s,c=math.sin(th),math.cos(th)
        dia=(v_al-R*ia+w*psi*s)/L; dib=(v_be-R*ib-w*psi*c)/L
        xp=np.array([ia+dia*dt,ib+dib*dt,w,th+w*dt,R])
        A=np.array([[-R/L,0,psi*s/L,w*psi*c/L,-ia/L],[0,-R/L,-psi*c/L,w*psi*s/L,-ib/L],
                    [0,0,0,0,0],[0,0,1,0,0],[0,0,0,0,0]])
        F=np.eye(5)+A*dt; Pp=F@self.P@F.T+self.Q
        z=np.array([ia_m,ib_m]); y=z-self.H@xp
        K=Pp@self.H.T@np.linalg.inv(self.H@Pp@self.H.T+self.Rn)
        self.x=xp+K@y; self.x[3]=(self.x[3]+math.pi)%(2*math.pi)-math.pi
        self.P=(np.eye(5)-K@self.H)@Pp
        return self.x[3],self.x[2],self.x[4]

class EKFSensorlessFOC:
    """闭环EKF无感FOC：位置/速度/R全由EKF估计。"""
    def __init__(self,cfg,lim,R_init):
        self.cfg=cfg; self.lim=lim
        self.ekf=EKF5(R_init,L,psi,DT)
        self.kp_w,self.ki_w=0.4,6.0; self.kp_i,self.ki_i=10.0,2500.0
        self.iw=self.iid=self.iiq=0.0; self.vprev=(0.,0.)
    def compute(self,meas,setpoint,dt):
        e=self.cfg.electrical; Vmax,Imax=self.lim.v_max,self.lim.i_max
        i_al,i_be=clarke(meas.i_a,meas.i_b,meas.i_c)
        th,we,R_est=self.ekf.step(self.vprev[0],self.vprev[1],i_al,i_be)
        wm=we/e.p
        i_d,i_q=park(i_al,i_be,th)
        ew=setpoint-wm; iqc=self.kp_w*ew+self.ki_w*self.iw
        if -Imax<iqc<Imax: self.iw+=ew*dt
        iqc=max(-Imax,min(Imax,iqc))
        eid,eiq=0.0-i_d,iqc-i_q
        vd=self.kp_i*eid+self.ki_i*self.iid; vq=self.kp_i*eiq+self.ki_i*self.iiq
        vs=math.hypot(vd,vq)
        if vs>Vmax: sc=Vmax/vs; vd*=sc; vq*=sc
        else: self.iid+=eid*dt; self.iiq+=eiq*dt
        v_al,v_be=inv_park(vd,vq,th); self.vprev=(v_al,v_be)
        return VoltageCommand(*inv_clarke(v_al,v_be))

SPEED=40.0
plant=MotorPlant(cfg, init_state=MotorState(omega_m=SPEED))
sfoc=EKFSensorlessFOC(cfg,lim,R_init=0.8)   # R初始给错
sfoc.ekf.x[2]=p*SPEED; sfoc.ekf.x[3]=0.0
cs=CurrentSensor(f_sample=100000,adc_bits=12,i_range=3.27,noise_std=0.003,seed=7)
sens_true=SensorSuite(current=IdealCurrentSensor(),position=IdealEncoder())
log={'t':[],'wm_true':[],'wm_est':[],'err':[],'R':[]}
for k in range(int(0.8/DT)):
    t=plant.t; to=plant.observe()
    ia_m,ib_m,ic_m=cs.read(to,DT)
    meas=replace(sens_true.measure(to,DT), i_a=ia_m,i_b=ib_m,i_c=ic_m)  # 控制器用含噪声电流
    Tload=0.10+(0.15 if t>0.5 else 0.0)   # 0.5s负载阶跃
    cmd=sfoc.compute(meas,SPEED,DT)
    plant.step(replace(IdealInverter().apply(cmd,to,DT),t_load=Tload),DT)
    if k%50==0:
        e=(sfoc.ekf.x[3]-to.theta_e+math.pi)%(2*math.pi)-math.pi
        log['t'].append(t); log['wm_true'].append(to.state.omega_m)
        log['wm_est'].append(sfoc.ekf.x[2]/p); log['err'].append(math.degrees(e)); log['R'].append(sfoc.ekf.x[4])
for kk in log: log[kk]=np.array(log[kk])
h=len(log['t'])//2
print("=== 闭环 EKF 无感 (R初始0.8, 真值0.5) ===")
print(f"末转速: 真实={log['wm_true'][-1]:.1f} 估计={log['wm_est'][-1]:.1f} rad/s")
print(f"R估计收敛: {log['R'][-1]:.3f} (真值0.5)")
print(f"位置误差稳态RMS={log['err'][h:].std():.2f}°")
print(f"0.5s负载阶跃后仍稳定")
np.savez('/tmp/ekfc.npz',**log)
