# firmware — IHM07M1 + F302R8 基础 FOC 固件

把本仓库仿真（`core/` + `control/01_foc_sensored.py`）验证过的 FOC 控制律，
按 `docs/05_hardware_deployment.md` 的**算法移植闭环**落到真实硬件：

- **板**：X-NUCLEO-IHM07M1（L6230 三相驱动）+ NUCLEO-F302R8（Cortex-M4F @72MHz）
- **框架**：STM32Cube HAL，构建用 **PlatformIO**（操作见 `skills/pio/SKILL.md`）
- **算法**：`src/foc.c` 是 `core/motorsim_core.py` 的逐行移植，PC 端回归保证"固件==仿真"

> 信条延续：**物理归 core，算法归 controller**。这里把"算法"那一侧搬到 MCU，
> 物理（电机/逆变器/传感器）由真实硬件承担，仿真里的 `Inverter/Sensors` 换成 TIM1/ADC/编码器。

## 快速开始

```bash
cd firmware
pio run                 # 编译（已验证：20KB flash / 524B RAM）
pio run -t upload       # 烧录（板载 ST-Link）
```

改算法前后跑回归（无需硬件）：

```bash
bash test/run_host_test.sh      # 期望：ALL PASS —— 固件 FOC 与仿真逐点一致。
```

## 信号链（对应仿真 `Controller → Inverter → Motor → Sensors`）

```
 ADC 注入(PA0/PC1/PB0) ─→ i_abc ─Clarke→ i_αβ ─Park(θ)→ i_dq
                                                          │
   编码器(TIM2) ─→ θ_e, ω_m              速度PI → iq_ref  ▼
                         │                    └──→ 电流PI(d,q) → v_dq
                         └────────── θ_e ──────────────────────│
                                                  inv_Park → v_αβ
                                                  SVPWM(零序注入) → duty[3]
                                                  → TIM1 CCR1/2/3 (PA8/9/10)
                                                  → L6230 → 电机
```

电流环跑在 **ADC 注入完成中断**（TIM1 TRGO 在 PWM 中点触发采样，电流纹波最小）；
速度环在主循环每 1ms 跑一次（电流环 ≫ 速度环，带宽分离）。

## 两种运行模式（`main.c` 的 `g_mode`）

| 模式 | 说明 | 对应仿真 |
|------|------|----------|
| `MODE_OPENLOOP_IF`（默认） | 开环 I/f：强制旋转角 + 恒定 iq，**安全首转** | `control/09_if_opencontrol.py` 思想 |
| `MODE_SENSORED` | 有感闭环 FOC：编码器角 + 速度环 | `control/01_foc_sensored.py` |

默认开环启动，是 `docs/05 §2.3` 的上电顺序要求：先确认相序/极对/转向，再闭环。

## 引脚映射（NUCLEO-F302R8 ↔ X-NUCLEO-IHM07M1）

| 功能 | MCU 引脚 | 外设 | 来源 |
|------|----------|------|------|
| PWM U/V/W → L6230 IN1/2/3 | PA8 / PA9 / PA10 | TIM1_CH1/2/3 (AF6) | UM1943 + 社区，已确认 |
| 使能 EN1/2/3 | PC10 / PC11 / PC12 | GPIO 推挽输出 | UM1943，已确认 |
| 相电流 op-amp 输出 | PA0 / PC1 / PB0 | ADC1 注入 CH1/7/11 | MCSDK 标准分配，**需核对** |
| 编码器 A/B | TIM2（引脚随接线） | TIM2 编码器模式 | **需按你的接线/UM1943 填** |

> ⚠ **数字引脚（PWM/EN）已多方确认**。**模拟引脚（电流 ADC 通道）按 ST MCSDK 对
> IHM07M1 的标准分配填写**，不同批次/跳线可能不同——**驱动电机前务必对照你板子的
> UM1943 原理图或 CubeMX 工程核对** `include/bsp_ihm07m1.h` 里的通道号。

## 电气常数（对齐仿真 `hardware/ihm07m1.py`）

| 量 | 值 | 仿真对应 |
|----|----|---------|
| 母线电压 v_dc | 24 V | `InverterLimits(24, ..)` |
| 电流上限 i_max | 2.5 A | `InverterLimits(.., 2.5)`（L6230 2.8A 峰值卡限）|
| PWM 频率 | 20 kHz | `f_pwm=20000` |
| 死区 | 1 µs | `dead_time=1e-6` |
| 电流链 | 0.33Ω×1.53=0.505 V/A，12-bit ±3.27A，LSB≈1.6mA | `CurrentSensor(adc_bits=12,i_range=3.27)` |
| 电机（占位，待标定） | R=0.5Ω Ld=4mH Lq=6mH ψ=0.03Wb p=4 | `small_pmsm` |

电机参数是占位默认值，真机标定后回填 `bsp_ihm07m1.h` 的 `BSP_MOTOR_*`（标定实验见 docs/05 §3.2）。

## 上电安全清单（docs/05 §2.3，务必遵守）

1. **脱开负载/桨**做第一次上电。
2. 保持默认 `MODE_OPENLOOP_IF`，观察是否平稳旋转（确认相序、极对数 `p`、转向）。
3. 限幅先行：`BSP_I_MAX` 软限流、母线限幅、堵转/过温超时（按需补全）。
4. 确认无误后改 `g_mode = MODE_SENSORED`，逐步加 `g_speed_ref`。
5. 每步**录波归档**（含失败样本）→ 反推 core 参数，形成 sim↔real 闭环（docs/05 §3-4）。

## 已知边界 / 待办

- 编码器引脚与每转线数需按实际接线填 `bsp_encoder_init` 与 `BSP_ENCODER_PPR`；
  当前默认 PPR=2500（对齐仿真），TIM2 编码器引脚为占位，闭环前务必核对。
- 基础 FOC 固定 `id_ref=0`（无弱磁/MTPA）；如需弱磁，按仿真 `FieldWeakeningFOC` 的
  MTPA/弱磁段扩展 `foc.c`，扩展后同样跑 `test/` 回归。
- 单分流/三分流跳线（J5/J6）：低速无感需三分流；本基础有感 FOC 三相电流直采（注入组三通道）。
- 母线电压/温度 ADC 通道未接入（基础有感 FOC 不必需），需要时按 UM1943 补到 BSP。
