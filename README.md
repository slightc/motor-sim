# motorsim

模块化 PMSM（永磁同步电机）仿真与无感控制研究框架。面向 X-NUCLEO-IHM07M1 等真实硬件。

**核心信条：物理归 core，算法归 controller。**

```
core/        仿真核心 + 官方扩展(逆变器/传感器)
control/     11 个控制算法 demo (FOC / 反电动势 / HFI / 方波 / EKF / 融合 / I/f / 定位)
firmware/    STM32 固件: IHM07M1+F302R8 有感/无感 FOC + 参数自整定 (PlatformIO, 移植自 core)
docs/        文档(总览/架构/物理/控制方法/硬件)
extensions/  自定义控制器与传感器模板
skills/      motorsim / pio skill (供外部 agent 调用)
agent.md     agent 工作指南 (claude.md 为其软链接)
```

## 快速开始

```bash
cd control
python3 01_foc_sensored.py              # 有感 FOC 基线
python3 08_fullrange_fusion.py          # 全速域融合(HFI⊕EKF)
python3 10_position_servo_closedloop.py # 闭环无感位置伺服 <1°
```

落到真机（X-NUCLEO-IHM07M1 + NUCLEO-F302R8）：

```bash
cd firmware
bash test/run_host_test.sh   # 固件 FOC 与仿真逐点回归（无需硬件）
pio run                      # 编译; pio run -t upload 烧录
```

详见 `docs/00_overview.md`、`firmware/README.md`、`skills/pio/SKILL.md`。
