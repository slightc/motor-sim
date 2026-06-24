---
name: motorsim
description: >
  PMSM (永磁同步电机) 仿真与无感控制研究框架。当用户需要：仿真永磁同步电机/BLDC；
  研究或对比无感控制算法(FOC、反电动势观测、HFI/方波高频注入、EKF、I/f、全速域融合)；
  分析低速高负载、动态负载、传感器噪声/ADC量化、交叉饱和、磁极辨识/IPD、闭环位置伺服；
  或针对 X-NUCLEO-IHM07M1 等硬件评估可行性时，使用本 skill。涉及 PMSM/FOC/无感/sensorless/
  HFI/EKF/电机仿真等关键词即触发。
---

# motorsim — PMSM 仿真与无感控制

## 这是什么

一个模块化电机仿真框架，严格遵循 **物理归 core，算法归 controller**。信号链：
`Controller → Inverter → Motor(物理) → Sensors → Controller`。

## 何时用

- 仿真 PMSM 电磁/机械/热行为、铁损、凸极、交叉饱和、磁极饱和
- 实现/对比无感控制算法，且不想重写电机模型
- 评估真实硬件(分流电阻、运放、ADC 位数/噪声、PWM)对无感的影响
- 研究低速高负载、动态负载、全速域、精密定位

## 目录

```
core/        电机物理 + 官方扩展(逆变器/传感器); import 入口
control/     11 个控制算法可运行 demo (见 docs/03_control_methods.md)
docs/        架构/物理/控制方法/硬件文档
extensions/  自定义控制器/传感器模板
```

## 最快用法

```python
import sys; sys.path.insert(0, "core")
from motorsim_core import *
from motorsim_sensors import *
from dataclasses import replace

cfg = MotorConfig(electrical=ElectricalParams(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4),
                  thermal=ThermalParams(enabled=False))
lim = InverterLimits(24, 2.5)
plant = MotorPlant(cfg, init_state=MotorState(omega_m=0.0))
foc   = FieldWeakeningFOC(cfg, lim)
sens  = SensorSuite(IdealCurrentSensor(), Encoder(2500, p=4))

dt = 20e-6
for _ in range(20000):
    obs  = plant.observe()
    meas = sens.measure(obs, dt)
    cmd  = foc.compute(meas, setpoint=50.0, dt=dt)     # 目标 50 rad/s
    plant.step(replace(IdealInverter().apply(cmd, obs, dt), t_load=0.1), dt)
print(plant.observe().state.omega_m)
```

## 如何扩展（不改 core）

实现三个 Protocol 之一：
- `Controller.compute(meas, setpoint, dt) -> VoltageCommand`  ← 新控制/观测算法
- `SensorSuite.measure(true_obs, dt) -> Measurements`         ← 新传感器/检测物理
- `Inverter.apply(cmd, true_obs, dt) -> MotorInput`           ← 新功率级

模板见 `extensions/custom_controller.py` 和 `extensions/custom_sensor.py`。
仅当引入**新物理现象**(如磁极饱和)时才改 `core/motorsim_core.py` 的 `_deriv`。

## 关键参数对齐真实硬件(X-NUCLEO-IHM07M1)

```python
CurrentSensor(adc_bits=12, i_range=3.27, noise_std=0.003)  # LSB≈1.6mA
InverterLimits(v_dc=24, i_max=2.5)                          # 2.8A峰值
```

## 重要经验(避免踩坑)

- 方波 HFI 解调：注入符号必须**周期初翻转**，注入与解调用**同一符号**；相邻两周期解调取平均以抵消基波斜率。
- 交叉饱和在高负载下偏移凸极轴 φ_sat∝iq，必须补偿(轻中载可用解析式，重载需离线标定查表)。
- 无单一方法覆盖全速域；融合加权按可观测性 ωeψ/(Ri) 而非纯速度。
- matplotlib 无 CJK 字体，图标签用英文，正文用中文。
