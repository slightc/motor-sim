# motorsim 总览

一个模块化的 PMSM（永磁同步电机）仿真与无感控制研究框架，面向 X-NUCLEO-IHM07M1 等真实硬件。

## 核心设计原则

**物理归 core，算法归 controller。**

信号链：
```
Controller(决策) → Inverter(开关物理) → Motor(电磁物理) → Sensors(检测物理) → 回到 Controller
```

- **位置解算**属于算法 → Controller
- **检测**(电流/位置传感)属于硬件 → Sensors
- **电机/铁芯物理**属于 core

任何新的观测器/无感方案 = 新的 Controller，core/inverter/sensors 一律不动。唯一需要动 core 的，是引入**新的物理现象**（如磁极饱和）。

## 目录结构

| 目录 | 内容 |
|------|------|
| `core/` | 仿真核心 + 官方扩展(逆变器/传感器) |
| `control/` | 各控制算法的可运行 demo |
| `docs/` | 文档 |
| `extensions/` | 自定义扩展示例(加控制器/传感器) |
| `skills/motorsim/` | 给外部 agent 使用的 skill |

## 快速开始

```bash
cd control
python3 01_foc_sensored.py             # 有感 FOC 基线
python3 10_position_servo_closedloop.py # 闭环无感位置伺服 <1°
```

## 落到真机

把算法部署到 X-NUCLEO-IHM07M1，并用真机录波反向标定 core 物理参数，以黄金数据集做可回归迭代，
见 `docs/05_hardware_deployment.md`。
