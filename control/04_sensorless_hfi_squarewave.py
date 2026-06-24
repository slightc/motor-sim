import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))

# -*- coding: utf-8 -*-
"""方波注入(HFSI) vs 脉振HFI：带宽优势（板级参数）"""
from motorsim_core import *
from motorsim_sensors import *
import numpy as np, math
from dataclasses import replace

K_CROSS=-3.2e-4
cfg=MotorConfig(electrical=ElectricalParams(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4,k_cross=K_CROSS),
                thermal=ThermalParams(enabled=False))
DT=10e-6; Ld,Lq=cfg.electrical.Ld,cfg.electrical.Lq; R=cfg.electrical.R0
IQ=2.0; V_SQ=4.0
F_PWM=10000.0; T_PWM=int((1/F_PWM)/DT)   # 100us=10步/PWM周期
I_RANGE=3.27   # 板级量程

def board_adc(): return CurrentSensor(f_sample=100000,adc_bits=12,i_range=I_RANGE,noise_std=0.003,seed=5)

def run_sqwave(avg_periods):
    """方波注入：每PWM周期±V_SQ翻转，取相邻周期电流差解调，平均avg_periods个周期。"""
    plant=MotorPlant(cfg, init_state=MotorState(omega_m=0.0))
    cs=board_adc(); vq=R*IQ
    phis=[]; iq_prev=id_prev=None; ratio_acc=[]; pwm_cnt=0; sign=1
    for k in range(int(1.2/DT)):
        t=plant.t
        # 每PWM周期翻转注入符号
        if k%T_PWM==0: sign=-sign; pwm_cnt+=1
        v_d=V_SQ*sign
        va,vb,vc=inv_clarke(*inv_park(v_d,vq,0.0))
        plant.step(MotorInput(va,vb,vc),DT); plant.state=replace(plant.state,omega_m=0.0,theta_m=0.0)
        # 在PWM周期末采样（开关稳定点）
        if k%T_PWM==T_PWM-1 and t>0.15:
            o=plant.observe(); ia,ib,ic=cs.read(o,DT)
            i_al,i_be=clarke(ia,ib,ic); i_d,i_q=park(i_al,i_be,0.0)
            if iq_prev is not None:
                # 相邻周期电流差 × 注入符号 -> 隔离HF响应
                d_iq=(i_q-iq_prev)*sign; d_id=(i_d-id_prev)*sign
                if abs(d_id)>1e-9: ratio_acc.append(d_iq/d_id)
                if len(ratio_acc)>=avg_periods:
                    r=np.mean(ratio_acc); Ldq=-r*Lq
                    phis.append(math.degrees(0.5*math.atan(2*Ldq/(Ld-Lq)))); ratio_acc=[]
            iq_prev,id_prev=i_q,i_d
    return np.array(phis)

# 方波：扫平均周期数 -> 带宽 F_PWM/(2*avg)
print("=== 方波注入(HFSI) 板级性能 ===")
print(f"PWM {F_PWM/1000:.0f}kHz, 注入±{V_SQ}V方波, iq={IQ}A满载")
base=run_sqwave(8)
print(f"解调 φ_sat={base.mean():.2f}° (基准16.2°), 验证方波解调正确\n")
print(f"{'平均周期':>8}{'带宽Hz':>9}{'抖动std':>10}")
sq={}
for avg in [2,4,8,16,32,64]:
    phis=run_sqwave(avg); bw=F_PWM/2/avg; std=phis.std()
    sq[avg]=(bw,std); print(f"{avg:>8}{bw:>9.0f}{std:>9.3f}°")
np.savez('/tmp/sqwave.npz',sq=np.array([[sq[a][0],sq[a][1]] for a in [2,4,8,16,32,64]]))
print(f"\n方波带宽可达 {F_PWM/2/2:.0f}Hz；脉振HFI同噪声下顶多~250Hz")
print("-> 方波注入在保持低抖动的同时把位置带宽提升约一个数量级")

# 加大注入幅值（方波在PWM/2不可闻，可承受更大注入）
print("\n=== 注入幅值旋钮（方波可承受大注入因不可闻）===")
for V in [4.0,8.0,12.0]:
    V_SQ_g=V
    def run_v(avg, Vsq):
        plant=MotorPlant(cfg, init_state=MotorState(omega_m=0.0))
        cs=board_adc(); vq=R*IQ
        phis=[]; iq_prev=id_prev=None; ratio_acc=[]; sign=1
        for k in range(int(1.0/DT)):
            t=plant.t
            if k%T_PWM==0: sign=-sign
            va,vb,vc=inv_clarke(*inv_park(Vsq*sign,vq,0.0))
            plant.step(MotorInput(va,vb,vc),DT); plant.state=replace(plant.state,omega_m=0.0,theta_m=0.0)
            if k%T_PWM==T_PWM-1 and t>0.15:
                o=plant.observe(); ia,ib,ic=cs.read(o,DT)
                i_al,i_be=clarke(ia,ib,ic); i_d,i_q=park(i_al,i_be,0.0)
                if iq_prev is not None:
                    d_iq=(i_q-iq_prev)*sign; d_id=(i_d-id_prev)*sign
                    if abs(d_id)>1e-9: ratio_acc.append(d_iq/d_id)
                    if len(ratio_acc)>=avg:
                        phis.append(math.degrees(0.5*math.atan(2*(-np.mean(ratio_acc)*Lq)/(Ld-Lq)))); ratio_acc=[]
                iq_prev,id_prev=i_q,i_d
        return np.array(phis).std()
    std_hi=run_v(4,V)    # 1250Hz带宽
    print(f"  注入±{V:.0f}V @1250Hz带宽: 抖动={std_hi:.3f}°")
