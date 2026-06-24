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
