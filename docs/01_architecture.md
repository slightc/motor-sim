# 架构

## 三层 + 控制器

```
core/motorsim_core.py     电机物理 + 编排(Simulator/Recorder) + 接口(Protocol)
core/motorsim_inverter.py 官方扩展: SVPWM 逆变器(死区/补偿)
core/motorsim_sensors.py  官方扩展: 电流/位置传感器(量化/噪声/采样)
```

## 数据契约（dataclass）

| 类型 | 方向 | 含义 |
|------|------|------|
| `VoltageCommand(v_a,v_b,v_c)` | Controller → Inverter | 三相参考电压 |
| `MotorInput(v_a,v_b,v_c,t_load)` | Inverter → Motor | 三相实加电压 + 负载 |
| `Observation` | Motor → (真值) | 真实状态(αβ电流,θ,ω,温度) |
| `Measurements` | Sensors → Controller | 测量电流/位置/速度(含噪声量化) |

## 接口（Protocol）

```python
class Controller(Protocol):
    def compute(self, meas: Measurements, setpoint: float, dt: float) -> VoltageCommand: ...
class Inverter(Protocol):
    def apply(self, cmd: VoltageCommand, true_obs: Observation, dt: float) -> MotorInput: ...
class SensorSuite(Protocol):
    def measure(self, true_obs: Observation, dt: float) -> Measurements: ...
```

实现这三个 Protocol 之一即可插入仿真，无需改动其它层。

## 电机物理核心（MotorPlant）

- **相域积分**：三相电压为输入，αβ 电流为核心积分状态(RK4)，无零序奇异。
- **凸极电感** L_αβ(θ)，含交叉饱和项 Ldq=k_cross·iq。
- **铁损**(Bertotti)、**d 轴极性饱和**(磁极辨识用，Ld 随 i_d 不对称)。

## 仿真编排（Simulator）多速率时序

`Simulator.run(duration, dt, ..., f_ctrl=None, control_delay=0)` 支持把"控制执行率"与
"物理积分步长"解耦，逼近真实数字 FOC 的离散时序：

- **物理细 dt 积分**：`dt` 是物理步长，可取得足够小以**过采样 PWM 载波**（逆变器/死区按 dt 解析）。
- **控制按 f_ctrl 执行**：控制周期 `Tc=1/f_ctrl`（对齐到 dt 整数倍）；两次控制更新之间，指令
  **零阶保持(ZOH)** 施加。控制器 `compute` 收到的 `dt` 即 Tc。
- **计算/更新延迟 control_delay**：以"控制周期"为单位的整数，复现采样→计算→下周期更新的
  `z^-N` 延迟（真实 MCU 典型 1~2 个周期）；延迟增大→相位裕度下降、超调上升（见 demo 11）。
- **慢环分频(decimation)**：内环(电流)每周期跑、外环(速度/位置)按整数分频跑，属 **controller 内部**
  逻辑（计数器实现，见 demo 11 的 `MultiRateFOC`），框架只负责按 f_ctrl 驱动。

**向后兼容**：`f_ctrl=None` 且 `control_delay=0` 时退化为单速率、零延迟，与改动前逐位一致。
