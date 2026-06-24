# hardware/ —— 硬件抽象层

把"一块真实硬件"抽象成**参数配置**（电机 / 功率级·电压 / 电流传感器 / 位置传感器），
并提供工厂方法把这些参数一键装配成 core 能用的对象。对外只有一个 `HardwareProfile` 类。

```
hardware_profile.py   HardwareProfile + 四个子配置(PowerStage/CurrentSensor/PositionSensor)
ihm07m1.py            X-NUCLEO-IHM07M1 具体 Profile（+ 冒烟自检 __main__）
__init__.py           包说明
```

## 用法

```python
import sys, os
sys.path.insert(0, "hardware")          # 或把 hardware/ 加入 PYTHONPATH
from ihm07m1 import X_NUCLEO_IHM07M1
from motorsim_core import FieldWeakeningFOC

hw  = X_NUCLEO_IHM07M1(position="encoder")   # 取一块硬件
print(hw.summary())

foc = FieldWeakeningFOC(hw.motor_config(), hw.limits())
sim = hw.build_simulator(foc)                # 自动装好 plant+inverter+sensors
final = sim.run(0.4, dt=20e-6, reference=lambda t: 50.0, load=lambda t: 0.1)
```

直接跑自检：

```bash
cd hardware && python3 ihm07m1.py
```

## 抽象映射

| 硬件层配置 | 装配成的 core 对象 | 工厂方法 |
|------------|--------------------|----------|
| `motor: MotorConfig` | `MotorPlant` | `build_plant()` / `motor_config()` |
| `power: PowerStageConfig`（v_dc/f_pwm/死区/电流能力） | `SVPWMInverter` + `InverterLimits` | `build_inverter()` / `limits()` |
| `current_sensor: CurrentSensorConfig`（ADC/噪声/分流） | `CurrentSensor` 或 `IdealCurrentSensor` | （并入）`build_sensors()` |
| `position_sensor: PositionSensorConfig`（encoder/hall/ideal） | `Encoder`/`HallSensor`/`IdealEncoder` | `build_sensors()` |
| 全部 | `Simulator` | `build_simulator(controller)` |

## 选项

`X_NUCLEO_IHM07M1(position="encoder", ppr=2500, ideal_current=False, motor=None)`

- `position`：`"encoder"` / `"hall"` / `"ideal"`（无感方案选 `ideal`，controller 自行解算位置）。
- `ideal_current`：`True` 去掉量化/噪声做对照基准。
- `motor`：传入真机标定得到的 `MotorConfig` 覆盖占位默认值。
- `hw.with_overrides(...)`：返回覆盖部分子配置的新 Profile，便于参数扫描/标定。

## 新增一块硬件

仿 `ihm07m1.py` 写一个返回 `HardwareProfile` 的工厂函数，填好四个子配置即可——
**core 与 controller 一律不动**。电机电磁/机械参数应由真机标定回填
（流程见 `docs/05_hardware_deployment.md` 的「参数→标定实验映射」）。

> 注：`ihm07m1.py` 里的电机电磁参数是占位默认值，功率级与电流链是与板子对齐的真实硬件事实。
