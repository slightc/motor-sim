# -*- coding: utf-8 -*-
"""
motorsim.sensors —— 传感器物理模块（扩展）

传感器测量电机真值，输出控制器能用的失真测量。包含：
  CurrentSensor   电流检测（采样保持 + ADC 量化 + 噪声）
  位置传感器（决定有感 FOC 的位置精度）：
    IdealEncoder  理想（真值，对照基准）
    HallSensor    霍尔：每 60° 电角一个扇区，低分辨率（BLDC 常用）
    Encoder       增量式光栅编码器：高分辨率，PPR 线数量化
组合用 SensorSuite：电流传感器 + 位置传感器 -> Measurements。
"""
from motorsim_core import Measurements, clarke, park
import math, random

# ---------- 电流传感器 ----------
class CurrentSensor:
    def __init__(self, f_sample=10000.0, adc_bits=12, i_range=30.0, noise_std=0.0, seed=0):
        self.Ts=1.0/f_sample; self.lsb=(2*i_range)/(2**adc_bits)
        self.noise_std=noise_std; self._rng=random.Random(seed)
        self._held=None; self._next=0.0
    def _q(self,x):
        n=self._rng.gauss(0,self.noise_std) if self.noise_std>0 else 0.0
        return round((x+n)/self.lsb)*self.lsb
    def read(self, obs, dt):
        if self._held is None or obs.t>=self._next-1e-12:
            self._held=(self._q(obs.i_a),self._q(obs.i_b),self._q(obs.i_c)); self._next=obs.t+self.Ts
        return self._held
class IdealCurrentSensor:
    def read(self, obs, dt): return (obs.i_a,obs.i_b,obs.i_c)

# ---------- 位置传感器 ----------
class IdealEncoder:
    """理想位置：真值（对照基准）。"""
    def read(self, obs, dt): return obs.theta_e, obs.state.omega_m

class HallSensor:
    """霍尔传感器：3 元件，每 60° 电角一个扇区。位置只能定位到扇区中点
    (±30°电角误差)，速度由扇区切换时间间隔估计。低速误差大 -> 转矩脉动。"""
    def __init__(self, p=4):
        self.p=p; self._last_sector=None; self._last_t=0.0; self._omega_m=0.0
    def read(self, obs, dt):
        the=obs.theta_e%(2*math.pi)
        sector=int(the//(math.pi/3))                  # 0..5
        theta_meas=sector*(math.pi/3)+math.pi/6       # 扇区中点（低分辨率位置）
        if self._last_sector is not None and sector!=self._last_sector:
            ds=(sector-self._last_sector)%6
            d_elec=(math.pi/3) if ds==1 else (-(math.pi/3) if ds==5 else 0.0)
            dtt=obs.t-self._last_t
            if dtt>1e-6 and d_elec!=0.0:
                self._omega_m=(d_elec/dtt)/self.p     # 电角速度/p -> 机械速度
            self._last_t=obs.t
        if self._last_sector is None: self._last_t=obs.t
        self._last_sector=sector
        return theta_meas, self._omega_m

class Encoder:
    """增量式光栅编码器：PPR 线数 × 4(正交) 量化机械角。高分辨率位置。
    速度用 M 法（固定采样窗口内位置差分），而非每步差分（否则量化产生脉冲噪声）。"""
    def __init__(self, ppr=2500, quad=4, offset=0.0, p=4, f_sample=10000.0):
        self.res=2*math.pi/(ppr*quad)               # 机械角分辨率
        self.offset=offset; self.p=p; self.Ts=1.0/f_sample
        self._last_theta=None; self._last_t=0.0; self._next=0.0; self._omega=0.0
    def read(self, obs, dt):
        thm_q=round(obs.state.theta_m/self.res)*self.res    # 量化机械角（高分辨率位置）
        if self._last_theta is None:
            self._last_theta=thm_q; self._last_t=obs.t; self._next=obs.t+self.Ts
        elif obs.t>=self._next-1e-12:                        # M 法：采样窗口测速
            self._omega=(thm_q-self._last_theta)/(obs.t-self._last_t)
            self._last_theta=thm_q; self._last_t=obs.t; self._next=obs.t+self.Ts
        theta_e_meas=(self.p*thm_q+self.offset)%(2*math.pi)
        return theta_e_meas, self._omega

# ---------- 组合 ----------
class SensorSuite:
    """组合电流传感器 + 位置传感器，产出 Measurements。
    用测量角度把测量 i_abc 变换成 i_dq（与真实数字 FOC 一致）。"""
    def __init__(self, current=None, position=None):
        self.current=current or IdealCurrentSensor()
        self.position=position or IdealEncoder()
    def measure(self, obs, dt):
        ia,ib,ic=self.current.read(obs,dt)
        theta_e,omega_m=self.position.read(obs,dt)
        i_al,i_be=clarke(ia,ib,ic)
        i_d,i_q=park(i_al,i_be,theta_e)             # 用测量角度
        return Measurements(t=obs.t,i_a=ia,i_b=ib,i_c=ic,i_d=i_d,i_q=i_q,
                            theta_e=theta_e,omega_m=omega_m,theta_e_true=obs.theta_e)
