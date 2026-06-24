# hardware/ —— 硬件抽象层

把"一块真实硬件"抽象成**参数配置**（电机 / 功率级·电压 / 电流传感器 / 位置传感器），
并提供工厂方法把这些参数一键装配成 core 能用的对象。对外只有一个 `HardwareProfile` 类。

```
hardware_profile.py   HardwareProfile + 子配置(PowerStage/CurrentSensor/PositionSensor)
ihm07m1.py            X-NUCLEO-IHM07M1 具体 Profile（+ 冒烟自检 __main__）
motor/                可复用电机配置库（命名 MotorConfig 工厂）
  motor_library.py      small_pmsm / small_pmsm_salient / hfi_high_flux + get_motor/list_motors
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

## 可复用电机库 `motor/`

电机参数从硬件 Profile 里抽出来，集中成命名工厂，硬件与 demo 共用、避免到处硬编码：

```python
import sys; sys.path.insert(0, "hardware/motor")
from motor_library import get_motor, small_pmsm, small_pmsm_salient, list_motors

m = small_pmsm()                         # 基础小型 PMSM（IHM07M1 量级）
m = small_pmsm(thermal=False)            # 关热模型
m = get_motor("small_pmsm_salient")      # 带凸极/饱和（HFI/磁极辨识）
m = get_motor("small_pmsm", R0=0.42, p=4) # 覆盖电气参数（真机标定回填）
list_motors()                            # 查看全部预设
```

| 预设 | 用途 |
|------|------|
| `small_pmsm` | 基础小型 PMSM，凸极/饱和关闭；硬件默认与多数 demo |
| `small_pmsm_salient` | 带交叉饱和 + d 轴极性饱和，HFI 无感 / 磁极辨识 / 闭环无感伺服 |
| `hfi_high_flux` | 高磁链(ψ=0.08)凸极，脉振 HFI 标定场景 |

`X_NUCLEO_IHM07M1(motor=...)` 默认取 `small_pmsm()`，可传入库里任一电机或真机标定的 `MotorConfig`。
每个工厂返回**全新实例**，可安全用于参数扫描/多 plant。新增电机：在 `motor_library.py` 加工厂并登记到 `MOTORS`。

## 新增一块硬件

仿 `ihm07m1.py` 写一个返回 `HardwareProfile` 的工厂函数，填好四个子配置即可——
**core 与 controller 一律不动**。电机电磁/机械参数应由真机标定回填
（流程见 `docs/05_hardware_deployment.md` 的「参数→标定实验映射」）。

> 注：`ihm07m1.py` 里的电机电磁参数是占位默认值，功率级与电流链是与板子对齐的真实硬件事实。
