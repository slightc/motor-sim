# firmware — IHM07M1 + F302R8 有感/无感 FOC + 参数自整定 固件

把本仓库仿真（`core/` + `control/`）验证过的 FOC 控制律，
按 `docs/05_hardware_deployment.md` 的**算法移植闭环**落到真实硬件：

> 本目录是 PlatformIO 工程；通用 PlatformIO 操作（命令/`platformio.ini`/调试/测试）见
> `skills/pio/SKILL.md`。**本 README 只讲本工程的板子/引脚/运行/安全等特定信息。**

- **板**：X-NUCLEO-IHM07M1（L6230 三相驱动）+ NUCLEO-F302R8（Cortex-M4F @72MHz）
- **框架**：STM32Cube HAL，构建用 **PlatformIO**（操作见 `skills/pio/SKILL.md`）
- **算法**：`src/foc.c` / `src/foc_sensorless.c` 是 `core/motorsim_core.py` 的逐行移植，
  PC 端回归保证"固件==仿真"
- **能力**：有感 FOC、**无感 FOC（反电动势观测器）**、**电机参数自动测量（Rs/Ld/Lq 自整定）**

源文件：
```
src/foc.c            Clarke/Park/PI 双环/SVPWM（移植自 FieldWeakeningFOC）
src/foc_sensorless.c 反电动势观测器 + PLL + 无感 FOC（移植自 BackEMFObserver/SensorlessFOC）
src/param_id.c       参数自整定状态机（Rs/Ld/Lq，docs/05 §3.2）
src/bsp_ihm07m1.c    TIM1 PWM / ADC 注入 / 编码器 / 时钟（HAL）
src/main.c           模式编排：PARAM_ID→SENSORLESS 默认流程；ADC 中断跑电流环
src/stm32f3xx_it.c   ADC 注入完成中断 → 一拍控制
test/                PC 原生回归（与 core 逐点对齐 + 自整定回收已知参数）
```

> 信条延续：**物理归 core，算法归 controller**。这里把"算法"那一侧搬到 MCU，
> 物理（电机/逆变器/传感器）由真实硬件承担，仿真里的 `Inverter/Sensors` 换成 TIM1/ADC/编码器。

## 快速开始

```bash
cd firmware/ihm07m1_foc
pio run                 # 编译 env=nucleo_f302r8（已验证：22KB flash / 704B RAM）
pio run -t upload       # 烧录（板载 ST-Link，upload_protocol=stlink）
```

构建环境与编译选项见本目录 `platformio.ini`（board=`nucleo_f302r8`，framework=`stm32cube`）。

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

## 运行模式（`main.c` 的 `g_mode`）

| 模式 | 说明 | 对应仿真 |
|------|------|----------|
| `MODE_PARAM_ID`（默认） | 上电**自动测量电机参数** Rs/Ld/Lq（静止） | `docs/05 §3.2` 标定实验 |
| `MODE_OPENLOOP_IF` | 开环 I/f：强制旋转角 + 恒定 iq，安全首转 | `control/09_if_opencontrol.py` |
| `MODE_SENSORED` | 有感闭环 FOC：编码器角 + 速度环 | `control/01_foc_sensored.py` |
| `MODE_SENSORLESS` | **无感闭环 FOC**：I/f 起转 → 反电动势观测器 PLL | `control/02_sensorless_backemf.py` |

**默认流程**（本固件的「无感 FOC + 自动测量参数」主线）：

```
上电 → MODE_PARAM_ID 自测 Rs/Ld/Lq → 回填电机参数
     → MODE_SENSORLESS：I/f 开环起转到 ~80 rad/s(电) → 交接到反电动势观测器闭环
```

全程无需位置传感器，也无需事先知道电机电气参数。

### 无感 FOC：反电动势观测器（`foc_sensorless.c`）

逐行移植自 `core` 的 `BackEMFObserver` + `SensorlessFOC`：

```
e_hat = v_αβ − R·i_αβ − L·di_αβ/dt   → 一阶低通 → PLL 锁相 → 估计 θ_e / ω_e
```

**关键**：PLL 误差按反电动势幅值 |e| 归一化，所以观测器**只需 R 与 L，不需要 ψ**
（ψ 只改 |e| 幅值，被归一化抵消）。这正是「静止自整定测出 Rs/Ld/Lq 就能跑无感」的依据。
中高速有效；**低速反电动势弱会失锁**——低速需高频注入(HFI)，见 `control/03/04/07`（本固件暂未移植）。

### 自动测量电机参数（`param_id.c`，对应 docs/05 §3.2）

电机**静止**即可测出无感所需电气参数，全程 θ_e=0（dq 坍缩成 αβ，数学简洁）：

| 参数 | 方法 | 公式 |
|------|------|------|
| **Rs** | d 轴两级 DC 电流注入 | `Rs = ΔV/ΔI`（差分抵消死区/管压降偏置）|
| **Ld** | d 轴方波高频电压注入，测电流纹波 | `Ld = V_inj·dt/Δi` |
| **Lq** | q 轴方波注入（DC d 偏置锁转子）测纹波 | `Lq = V_inj·dt/Δi` |
| ψ（磁链）| 无感 I/f 旋转段顺带估计、上报 | `ψ = (V_q − Rs·i_q)/ω_e` |

状态机每个电流环周期推进一拍（在 ADC 中断里）：`对齐 → Rs低 → Rs高 → Ld → Lq → DONE`。
主机回归用合成 R-L 电机驱动整个状态机，**回收 Rs=0.500Ω/Ld=3.99mH/Lq=5.99mH**（真值 0.5/4/6），
见 `test/`。

> ⚠ 自整定会让转子先对齐（小幅跳动）。**务必脱开负载**，确认电流限幅，再上电自测。
> 极对数 `p` 无法静止测得，仍由 `BSP_MOTOR_POLEPAIRS` 配置。

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

1. **脱开负载/桨**做第一次上电（自整定会让转子先对齐、起转会旋转）。
2. 限幅先行：`BSP_I_MAX` 软限流、母线限幅、堵转/过温超时（按需补全）。
3. **先单独验自整定**：临时设 `g_mode=MODE_PARAM_ID`，跑完看 `g_pid.Rs/Ld/Lq` 是否合理
   （调试器观察），不合理先查接线/电流采样符号，别急着闭环。
4. 校核 `BSP_MOTOR_POLEPAIRS` = 真实极对数（自整定测不出 `p`，无感锁相依赖它）。
5. 用默认流程跑无感：`MODE_PARAM_ID → MODE_SENSORLESS`，逐步加 `g_speed_ref`
   （无感需中高速，低速会失锁）。也可先用 `MODE_OPENLOOP_IF` 确认相序/转向。
6. 每步**录波归档**（含失败样本）→ 反推 core 参数，形成 sim↔real 闭环（docs/05 §3-4）。

## 已知边界 / 待办

- 编码器引脚与每转线数需按实际接线填 `bsp_encoder_init` 与 `BSP_ENCODER_PPR`；
  当前默认 PPR=2500（对齐仿真），TIM2 编码器引脚为占位，闭环前务必核对。
- 基础 FOC 固定 `id_ref=0`（无弱磁/MTPA）；如需弱磁，按仿真 `FieldWeakeningFOC` 的
  MTPA/弱磁段扩展 `foc.c`，扩展后同样跑 `test/` 回归。
- 单分流/三分流跳线（J5/J6）：低速无感需三分流；本基础有感 FOC 三相电流直采（注入组三通道）。
- 母线电压/温度 ADC 通道未接入（基础有感 FOC 不必需），需要时按 UM1943 补到 BSP。
