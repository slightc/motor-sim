# -*- coding: utf-8 -*-
"""
motorsim.inverter —— 逆变器物理模块（扩展）

逆变器是硬件物理，不是控制逻辑：它接收控制器的三相参考电压，经 SVPWM 开关
产生施加到电机的真实三相电压。死区效应由真实相电流方向决定（续流二极管），
因此 apply() 接收电机真值 true_obs —— 物理因果正确。
"""
from motorsim_core import MotorInput, VoltageCommand, Observation
import math

class SVPWMInverter:
    """SVPWM 逆变器：零序注入 + 载波比较 + 开关，可选死区(用真实电流方向)。"""
    def __init__(self, v_dc=48.0, f_pwm=10000.0, dead_time=0.0):
        self.v_dc=v_dc; self.f_pwm=f_pwm; self.dead_time=dead_time
        self.probe={}
    def apply(self, cmd: VoltageCommand, true_obs: Observation, dt) -> MotorInput:
        var,vbr,vcr=cmd.v_a,cmd.v_b,cmd.v_c
        voff=(max(var,vbr,vcr)+min(var,vbr,vcr))/2          # SVPWM 零序注入
        var-=voff; vbr-=voff; vcr-=voff
        if self.dead_time>0:                                 # 死区用真实电流方向（物理）
            Vd=self.dead_time*self.f_pwm*self.v_dc
            var-=math.copysign(Vd,true_obs.i_a)
            vbr-=math.copysign(Vd,true_obs.i_b)
            vcr-=math.copysign(Vd,true_obs.i_c)
        phase=(true_obs.t*self.f_pwm)%1.0
        carrier=(4*abs(phase-0.5)-1)*(self.v_dc/2)
        Vh=self.v_dc/2
        la=Vh if var>carrier else -Vh
        lb=Vh if vbr>carrier else -Vh
        lc=Vh if vcr>carrier else -Vh
        vn=(la+lb+lc)/3
        self.probe={'v_mod':(var,vbr,vcr),'carrier':carrier,'leg':(la,lb,lc)}
        return MotorInput(v_a=la-vn,v_b=lb-vn,v_c=lc-vn)
