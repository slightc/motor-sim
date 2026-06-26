# firmware — 固件工程集合

本目录存放**多个独立的固件工程**，每个工程对应一块硬件 / 一个 MCU 目标，
各自一个子目录、各自一套 `platformio.ini` 与 `README`。把仿真（`core/` + `control/`）
验证过的控制律按 `docs/05_hardware_deployment.md` 的**算法移植闭环**落到真实硬件。

> 通用 PlatformIO 操作（命令 / `platformio.ini` / 调试 / 测试 / 工程结构约定）见
> `skills/pio/SKILL.md`。各子工程的**板子、引脚、接线、运行与安全**等特定信息，
> 见该子目录自己的 `README`。

## 工程列表

| 子目录 | 目标硬件 | 内容 |
|--------|----------|------|
| [`ihm07m1_foc/`](ihm07m1_foc/README.md) | X-NUCLEO-IHM07M1 + NUCLEO-F302R8 | 有感/无感 FOC + 电机参数自整定 |

## 目录约定

```
firmware/
  README.md            ← 本文件（工程索引）
  <project_a>/         一个固件工程（自带 platformio.ini / README / src / include / test）
    platformio.ini
    README.md          ← 该工程的板子/引脚/运行/安全
    include/  src/  test/
  <project_b>/         另一个固件工程……
```

每个子工程独立构建：`cd firmware/<project>` 后 `pio run` / `pio run -t upload`。

## 新增一个固件工程

1. 在 `firmware/` 下新建子目录，放入 `platformio.ini`（按 `skills/pio/SKILL.md` 的骨架）。
2. 算法尽量写成**硬件无关的纯模块**（只依赖标准库），放 `src/` 并在 `test/` 做 PC 端回归；
   硬件相关（寄存器/HAL/引脚/中断）单独成文件，板级常量集中到一个 BSP 头。
3. 写该子目录的 `README`：目标板、引脚映射、构建环境、运行流程、上电安全。
4. 把工程登记到上面的「工程列表」。
