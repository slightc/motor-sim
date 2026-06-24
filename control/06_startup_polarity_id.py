import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))

# -*- coding: utf-8 -*-
"""磁极辨识：静止时±d脉冲解180°模糊（依赖core新增的极性饱和模型）"""
from motorsim_core import *
from motorsim_sensors import *
import numpy as np, math
from dataclasses import replace

cfg=MotorConfig(electrical=ElectricalParams(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4,
                i_pm_sat=4.0,i_knee_sat=6.0),thermal=ThermalParams(enabled=False))
DT=5e-6; V_PULSE=8.0; T_PULSE=int(0.8e-3/DT)   # 0.8ms脉冲

def pulse_response(theta_axis, theta_true, sign):
    """沿theta_axis的d方向施加sign*V脉冲，电机锁定在theta_true，返回峰值|i_d|(轴系)。"""
    plant=MotorPlant(cfg, init_state=MotorState(omega_m=0.0,theta_m=theta_true/cfg.electrical.p))
    i_peak=0.0
    for k in range(T_PULSE):
        # 沿估计轴d方向注入
        v_al,v_be=inv_park(sign*V_PULSE,0.0,theta_axis)
        plant.step(MotorInput(*inv_clarke(v_al,v_be)),DT)
        plant.state=replace(plant.state,omega_m=0.0,theta_m=theta_true/cfg.electrical.p)  # 锁定
        o=plant.observe(); i_al,i_be=clarke(o.i_a,o.i_b,o.i_c)
        i_d_axis,_=park(i_al,i_be,theta_axis)         # 在估计轴系看d电流
        i_peak=max(i_peak,abs(i_d_axis))
    return i_peak

def identify_polarity(theta_axis, theta_true):
    """打±脉冲，电流大的方向=低电感=N极。返回校正后的轴角。"""
    I_plus=pulse_response(theta_axis, theta_true, +1)
    I_minus=pulse_response(theta_axis, theta_true, -1)
    # +方向电流更大 -> +d是N -> 轴正确；否则翻180°
    corrected = theta_axis if I_plus>I_minus else (theta_axis+math.pi)%(2*math.pi)
    return corrected, I_plus, I_minus

print("=== 磁极辨识：±0.8ms脉冲解180°模糊 ===")
print(f"{'真实θ':>8}{'IPD轴(模糊)':>12}{'I+':>8}{'I-':>8}{'判定':>8}{'校正θ':>9}{'结果':>6}")
np.random.seed(1); ok=0; N=8
for _ in range(N):
    th_true=np.random.uniform(0,2*math.pi)
    # IPD只能定到mod 180°：随机给出 th_true 或 th_true+π
    th_ipd=(th_true + (math.pi if np.random.rand()>0.5 else 0))%(2*math.pi)
    corr,Ip,Im=identify_polarity(th_ipd, th_true)
    err=abs((corr-th_true+math.pi)%(2*math.pi)-math.pi)
    good=err<0.3; ok+=good
    judge='N在+' if Ip>Im else 'N在-'
    print(f"{math.degrees(th_true):>7.0f}°{math.degrees(th_ipd):>11.0f}°{Ip:>8.2f}{Im:>8.2f}{judge:>8}{math.degrees(corr):>8.0f}°{'✓' if good else '✗':>6}")
print(f"\n{ok}/{N} 正确辨识N极并解出真实位置")
print("机理：+d(朝N)铁芯饱和→Ld低→电流大；-d(朝S)去饱和→Ld高→电流小")
