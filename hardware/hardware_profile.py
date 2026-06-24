# -*- coding: utf-8 -*-
"""
motorsim.hardware —— 硬件抽象层

把"一块真实硬件"抽象成一组**参数配置**，并提供工厂方法把这些参数装配成
core 能用的对象（电机/逆变器/传感器/限幅）。对外只暴露一个 HardwareProfile 类：

    hw = X_NUCLEO_IHM07M1()                 # 取一块硬件
    plant   = hw.build_plant()              # 电机物理
    inv     = hw.build_inverter()           # 逆变器物理（SVPWM/死区）
    sens    = hw.build_sensors()            # 传感器物理（ADC量化/噪声/位置）
    lim     = hw.limits()                   # 控制器用的电压/电流限幅
    sim     = hw.build_simulator(controller)# 一步到位组装 Simulator

设计原则（与项目一致）：物理参数归 core 的 dataclass，硬件层只做**聚合 + 装配**，
不写任何物理/控制逻辑。换硬件 = 换一个 HardwareProfile，core 与 controller 都不动。
真机标定得到的新参数，回填到对应 *Config 即可（见 docs/05_hardware_deployment.md）。
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))

from dataclasses import dataclass, field, replace
from typing import Optional, Callable
import math

from motorsim_core import (
    MotorConfig, ElectricalParams, MechanicalParams, ThermalParams,
    MotorPlant, MotorState, InverterLimits, Simulator,
)
from motorsim_inverter import SVPWMInverter
from motorsim_sensors import (
    CurrentSensor, IdealCurrentSensor,
    IdealEncoder, HallSensor, Encoder, SensorSuite,
)


# ---------------- 子配置：功率级 / 电压 ----------------
@dataclass
class PowerStageConfig:
    """逆变器 + 母线电压 + 电流能力。映射到 SVPWMInverter 与 InverterLimits。"""
    v_dc: float = 24.0          # 母线电压 (V)
    f_pwm: float = 20_000.0     # PWM 载波频率 (Hz)
    dead_time: float = 1.0e-6   # 死区时间 (s)
    i_peak: float = 2.5         # 峰值电流能力 (A)，做控制器电流限幅
    v_dc_range: tuple = (8.0, 48.0)   # 允许母线范围（信息性）


# ---------------- 子配置：电流传感器 ----------------
@dataclass
class CurrentSensorConfig:
    """电流检测链。shunt/gain 为信息性硬件事实，量化由 adc_bits + i_range 决定。"""
    ideal: bool = False         # True=理想电流传感器（对照基准）
    f_sample: float = 20_000.0  # 采样频率 (Hz)
    adc_bits: int = 12
    i_range: float = 3.27       # ADC 满量程对应电流 (±A)
    noise_std: float = 0.003    # 模拟噪声标准差 (A)
    seed: int = 0
    shunt_ohm: float = 0.33     # 分流电阻 (Ω，信息性)
    amp_gain: float = 1.53      # 运放增益 (信息性)

    @property
    def v_per_amp(self) -> float:
        return self.shunt_ohm * self.amp_gain

    @property
    def lsb_amp(self) -> float:
        return (2 * self.i_range) / (2 ** self.adc_bits)


# ---------------- 子配置：位置传感器 ----------------
@dataclass
class PositionSensorConfig:
    """位置检测。kind: ideal / hall / encoder。无感方案选 ideal（controller 自行解算，
    不消费 theta_e）。"""
    kind: str = "encoder"       # "ideal" | "hall" | "encoder"
    ppr: int = 2500             # 编码器线数（kind=encoder 时用）
    quad: int = 4               # 正交倍频
    offset: float = 0.0         # 电角安装偏置 (rad)
    f_sample: float = 20_000.0  # 测速采样频率 (Hz)


# ---------------- 硬件 Profile（对外主类）----------------
@dataclass
class HardwareProfile:
    """一块真实硬件的完整抽象：电机 + 功率级 + 电流/位置传感器。
    通过 build_* 工厂把参数装配成 core 对象。"""
    name: str = "GenericHardware"
    motor: MotorConfig = field(default_factory=MotorConfig)
    power: PowerStageConfig = field(default_factory=PowerStageConfig)
    current_sensor: CurrentSensorConfig = field(default_factory=CurrentSensorConfig)
    position_sensor: PositionSensorConfig = field(default_factory=PositionSensorConfig)
    notes: str = ""

    # --- 装配 ---
    def motor_config(self) -> MotorConfig:
        return self.motor

    def limits(self) -> InverterLimits:
        """控制器用的电压/电流上限。"""
        return InverterLimits(v_dc=self.power.v_dc, i_max=self.power.i_peak)

    def build_plant(self, init_state: Optional[MotorState] = None) -> MotorPlant:
        return MotorPlant(self.motor, init_state=init_state)

    def build_inverter(self) -> SVPWMInverter:
        p = self.power
        return SVPWMInverter(v_dc=p.v_dc, f_pwm=p.f_pwm, dead_time=p.dead_time)

    def _build_current_sensor(self):
        c = self.current_sensor
        if c.ideal:
            return IdealCurrentSensor()
        return CurrentSensor(f_sample=c.f_sample, adc_bits=c.adc_bits,
                             i_range=c.i_range, noise_std=c.noise_std, seed=c.seed)

    def _build_position_sensor(self):
        ps = self.position_sensor
        p = self.motor.electrical.p
        if ps.kind == "ideal":
            return IdealEncoder()
        if ps.kind == "hall":
            return HallSensor(p=p)
        if ps.kind == "encoder":
            return Encoder(ppr=ps.ppr, quad=ps.quad, offset=ps.offset,
                           p=p, f_sample=ps.f_sample)
        raise ValueError(f"未知位置传感器类型: {ps.kind!r}")

    def build_sensors(self) -> SensorSuite:
        return SensorSuite(current=self._build_current_sensor(),
                           position=self._build_position_sensor())

    def build_simulator(self, controller, observers=None,
                        init_state: Optional[MotorState] = None) -> Simulator:
        """一步装配完整仿真：plant + inverter + sensors + controller。"""
        return Simulator(plant=self.build_plant(init_state),
                         controller=controller,
                         inverter=self.build_inverter(),
                         sensors=self.build_sensors(),
                         observers=observers or [])

    # --- 派生工具 ---
    def with_overrides(self, **kw) -> "HardwareProfile":
        """返回覆盖部分子配置后的新 Profile（不可变风格，便于做参数扫描/标定）。"""
        return replace(self, **kw)

    def summary(self) -> str:
        e = self.motor.electrical
        c, ps, p = self.current_sensor, self.position_sensor, self.power
        if c.ideal:
            cur = "ideal"
        else:
            cur = ("%dbit ±%.2fA LSB=%.2fmA noise=%.1fmA %.3fV/A"
                   % (c.adc_bits, c.i_range, c.lsb_amp*1e3,
                      c.noise_std*1e3, c.v_per_amp))
        pos = ps.kind + (" ppr=%d" % ps.ppr if ps.kind == "encoder" else "")
        lines = [
            "硬件: %s" % self.name,
            "  电机: R0=%sΩ Ld=%.1fmH Lq=%.1fmH ψ=%sWb p=%d"
            % (e.R0, e.Ld*1e3, e.Lq*1e3, e.psi0, e.p),
            "  功率级: Vdc=%sV f_pwm=%.0fkHz dead=%.0fns i_peak=%sA"
            % (p.v_dc, p.f_pwm/1e3, p.dead_time*1e9, p.i_peak),
            "  电流传感: %s" % cur,
            "  位置传感: %s" % pos,
        ]
        if self.notes:
            lines.append("  备注: %s" % self.notes)
        return "\n".join(lines)
