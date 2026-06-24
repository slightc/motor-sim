# -*- coding: utf-8 -*-
"""
X-NUCLEO-IHM07M1 硬件 Profile

基于 L6230 的三相 BLDC/PMSM 驱动板（规格见 docs/04_hardware_ihm07m1.md）：
  驱动      L6230, 8–48 VDC
  电流能力  2.8A 峰值 / 1.4 Arms（小电机）
  分流      R43/R44/R45 = 0.33Ω，运放 TSV994 增益 1.53 → 0.505 V/A
  ADC       STM32 内部 12-bit, ±3.27A 量程 → LSB ≈ 1.6mA
  采样      三分流/单分流可选(J5/J6)，低速无感必须三分流
  位置      板载霍尔/编码器接口（BEMF 六步电路仅对六步有效，低速 FOC 无感不用）

电机电磁/机械参数是**占位默认值**，应由真机标定回填（docs/05_hardware_deployment.md 的
参数→实验映射）。改这里的 *Config 即可，core 与 controller 不动。
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))

from motorsim_core import (
    MotorConfig, ElectricalParams, MechanicalParams, ThermalParams,
)
from hardware_profile import (
    HardwareProfile, PowerStageConfig, CurrentSensorConfig, PositionSensorConfig,
)


# 与板子对齐的固定硬件事实（功率级 + 电流链）
_POWER = PowerStageConfig(
    v_dc=24.0, f_pwm=20_000.0, dead_time=1.0e-6,
    i_peak=2.5, v_dc_range=(8.0, 48.0),     # 板子 8–48V；i_peak 受 2.8A 峰值卡限
)
_CURRENT = CurrentSensorConfig(
    f_sample=20_000.0, adc_bits=12, i_range=3.27, noise_std=0.003,
    shunt_ohm=0.33, amp_gain=1.53,          # 0.33×1.53 = 0.505 V/A
)


def _default_motor() -> MotorConfig:
    """占位小型 PMSM（待真机标定）。与 control/ demo 用的电机量级一致。"""
    return MotorConfig(
        name="IHM07M1-PMSM(待标定)",
        electrical=ElectricalParams(R0=0.5, Ld=4.0e-3, Lq=6.0e-3, psi0=0.03, p=4),
        mechanical=MechanicalParams(J=6.0e-4, B=1.5e-4, Tc=0.02),
        thermal=ThermalParams(enabled=True, T_amb=25.0),
    )


def X_NUCLEO_IHM07M1(position: str = "encoder", ppr: int = 2500,
                     ideal_current: bool = False,
                     motor: MotorConfig = None) -> HardwareProfile:
    """构造 IHM07M1 硬件 Profile。

    参数
      position      位置传感器: "encoder"(默认) | "hall" | "ideal"
                    无感方案传 "ideal"（controller 自行解算，不消费 theta_e）
      ppr           编码器线数（position="encoder" 时）
      ideal_current True=理想电流传感器（去掉量化/噪声，做对照基准）
      motor         覆盖默认电机参数（真机标定后传入标定 MotorConfig）
    """
    cur = CurrentSensorConfig(**{**_CURRENT.__dict__, "ideal": ideal_current})
    pos = PositionSensorConfig(kind=position, ppr=ppr, f_sample=_CURRENT.f_sample)
    return HardwareProfile(
        name="X-NUCLEO-IHM07M1",
        motor=motor or _default_motor(),
        power=_POWER,
        current_sensor=cur,
        position_sensor=pos,
        notes="L6230 8-48V, 2.8A峰值; 0.505V/A, 12bit LSB≈1.6mA; 低速无感需三分流(J5/J6).",
    )


if __name__ == "__main__":
    # 冒烟自检：装配硬件 + 跑一段有感 FOC，打印末速与硬件摘要
    from motorsim_core import FieldWeakeningFOC, Recorder

    hw = X_NUCLEO_IHM07M1(position="encoder")
    print(hw.summary())

    foc = FieldWeakeningFOC(hw.motor_config(), hw.limits())
    rec = Recorder(["state.omega_m"])
    sim = hw.build_simulator(foc, observers=[rec])
    final = sim.run(duration=0.4, dt=20e-6,
                    reference=lambda t: 30.0 if t < 0.2 else 50.0,
                    load=lambda t: 0.10)
    print("末速 %.1f rad/s (目标 50), 转矩 %.3f N·m" % (final.state.omega_m, final.torque))
