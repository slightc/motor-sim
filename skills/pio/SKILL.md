---
name: pio
description: >
  用 PlatformIO 把本仓库仿真验证过的电机控制算法落到 STM32 真实硬件
  （X-NUCLEO-IHM07M1 + NUCLEO-F302R8）。当用户需要：编译/烧录/监视 firmware/ 下的
  FOC 固件；新增或修改 STM32 板级支持(BSP)、外设(TIM1 PWM/ADC/编码器)、控制 ISR；
  把 control/ 或 core 的算法移植到 MCU；调试 PlatformIO 工程(platformio.ini、HAL 配置、
  时钟/引脚映射)；或做仿真↔固件的"同输入同输出"回归时，使用本 skill。涉及
  platformio/pio/固件/firmware/烧录/upload/STM32/Nucleo/IHM07M1/HAL/移植 等关键词即触发。
---

# pio — 用 PlatformIO 把仿真算法落到 STM32 硬件

## 这是什么

本仓库是 PMSM 仿真框架（`core/` + `control/`）。**pio skill 负责"算法移植闭环"的硬件一侧**
（见 `docs/05_hardware_deployment.md`）：把 `control/` 里仿真验证过的 `Controller.compute`
逻辑，按位移植成 STM32 固件并用 PlatformIO 构建/烧录。

固件工程在 **`firmware/`**，目标板 **X-NUCLEO-IHM07M1 + NUCLEO-F302R8**，
框架 **STM32Cube HAL**。算法核心 `firmware/src/foc.c` 是 `core/motorsim_core.py` 的
逐行移植，并有 PC 端回归保证"固件 == 仿真"。

## 工程结构

```
firmware/
  platformio.ini              env=nucleo_f302r8, framework=stm32cube
  include/
    foc.h                     FOC 算法接口（纯算法，零硬件依赖）
    bsp_ihm07m1.h             板级常数 + 引脚映射（换板子只改这里）
    stm32f3xx_hal_conf.h      HAL 模块裁剪
  src/
    foc.c                     Clarke/Park/PI 双环/SVPWM（移植自仿真 core）
    bsp_ihm07m1.c             TIM1 PWM / ADC 注入 / 编码器 / 时钟（HAL）
    main.c                    控制编排：开环 I/f + 有感闭环 + ADC 中断电流环
    stm32f3xx_it.c            ADC 注入完成中断 → 跑一拍 FOC
  test/
    foc.c 的 PC 原生回归（gen_golden.py / test_foc_host.c / run_host_test.sh）
```

## 常用命令

```bash
cd firmware
pio run                       # 编译（首次会自动拉平台/工具链/HAL）
pio run -t upload             # 烧录到板子（ST-Link，板载）
pio device monitor            # 串口监视（115200）
pio run -t clean              # 清理构建产物
pio run -v                    # 详细编译（排错时看完整命令）
pio check                     # 静态检查
```

烧录前确认板子已用 USB 接 ST-Link（NUCLEO 板载），`upload_protocol=stlink` 已配好。

## 移植映射（仿真 → 固件，见 docs/05 §2.1）

| 仿真侧 | 固件侧 | 位置 |
|--------|--------|------|
| `Controller.compute(meas,sp,dt)` | 电流环 ISR（PWM 中点触发） | `main.c: foc_control_isr` |
| `VoltageCommand` → SVPWM | 占空比 → TIM1 CCR | `foc.c: foc_svpwm` + `bsp_pwm_set_duty` |
| `SensorSuite.measure` | ADC 注入 + Park | `bsp_read_phase_currents` + `foc_current_step` |
| `InverterLimits(24,2.5)` | `BSP_V_DC/BSP_I_MAX` | `bsp_ihm07m1.h` |
| `Encoder(2500,p=4)` | TIM2 编码器模式 | `bsp_encoder_*` |

PI 增益、限幅、电机参数全部取仿真默认值（`FieldWeakeningFOC`：kp_i=12, ki_i=3000,
kp_w=0.6, ki_w=10）。改算法 = 改 `foc.c`，**改完先跑回归**。

## 仿真↔固件回归（关键：保证移植不走样）

`foc.c` 是纯 C、零硬件依赖，可在 PC 上编译，与仿真 `core` 逐点对齐：

```bash
bash firmware/test/run_host_test.sh
# gen_golden.py 从 core 生成黄金值 → 原生编译 foc.c → 比对 Clarke/Park/SVPWM/电流环
# 期望输出：ALL PASS —— 固件 FOC 与仿真逐点一致。
```

**改 `foc.c` 后必须重跑**，红了说明移植偏离仿真。这是 docs/05「算法移植闭环」的固件侧门槛。

## 引脚映射（NUCLEO-F302R8 ↔ IHM07M1）

| 功能 | 引脚 | 外设 |
|------|------|------|
| PWM U/V/W（L6230 IN1/2/3） | PA8 / PA9 / PA10 | TIM1_CH1/2/3 (AF6) |
| 使能 EN1/2/3 | PC10 / PC11 / PC12 | GPIO 输出 |
| 相电流 ADC | PA0 / PC1 / PB0 | ADC1 注入组 |

⚠ 数字引脚（PWM/EN）已多方确认；**模拟引脚按 ST MCSDK 标准分配填写，
驱动电机前务必对照你板子的 UM1943 原理图 / CubeMX 工程核对**。

## 上电安全顺序（docs/05 §2.3，务必遵守）

1. **脱开负载/桨**，先跑**开环 I/f**（`main.c` 默认 `MODE_OPENLOOP_IF`）确认相序、极对数、转向。
2. 限幅先行：`BSP_I_MAX` 软限流、母线限幅、堵转超时。
3. 确认无误后再切 `MODE_SENSORED`（有感闭环），逐步加 `g_speed_ref`。
4. 每步**录波归档**（含失败样本），用于反推 core 参数（docs/05 §3）。

## 注意

- F302R8：Cortex-M4F（带 FPU），固件用单精度 `float`，与仿真一致；`-ffast-math` 已开。
- 控制律在 **ADC 注入完成中断**（PWM 中点同步采样，电流纹波最小）里跑，主循环只做慢速调度（测速/模式/安全）。
- 电流环 ≫ 速度环（带宽分离）：电流环每 PWM 周期（20kHz），速度环每 1ms。
- 无 HAL 环境也能验算法：`foc.c` 独立编译跑 `firmware/test/`。
