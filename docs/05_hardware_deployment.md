# 部署到真实硬件 + 仿真-实测可回归迭代

把 `control/` 里验证过的算法落到 X-NUCLEO-IHM07M1，再用真机数据**反过来修正 core 的物理参数与模型**，
形成 **sim → real → sim 的闭环**，并以**黄金数据集 + 回归脚本**保证每次改 core 都可量化、可回归、不退化。

> 信条延伸：算法在仿真里验证，在真机上证伪；真机暴露的偏差归 core 就修 core（见 `agent.md` 的「core 反哺与演进」）。

---

## 1. 两个闭环

```
        算法移植闭环                         模型对齐闭环
Controller(sim) ──移植──> 固件(MCU)     真机日志 ──标定──> core 参数
      ▲                      │              ▲                  │
      └──── 行为一致? ◀───────┘              └──── 误差<容差? ◀──┘
```

- **算法移植闭环**：仿真里的 `Controller.compute` 逻辑 → MCU 固件，验证「同输入同行为」。
- **模型对齐闭环**：真机录波 → 反推/标定 → 修正 `ElectricalParams/MechanicalParams/ThermalParams` → 仿真更准。
- 两闭环共享同一份**接口契约**（`docs/01_architecture.md` 的三个 Protocol），所以仿真验证过的控制律可以「按位移植」。

---

## 2. 部署到真实硬件

### 2.1 移植映射（Protocol → 固件）

| 仿真侧 | 真机侧 | 注意 |
|--------|--------|------|
| `Controller.compute(meas,setpoint,dt)` | 电流环 ISR（PWM 中点触发） | 定点/浮点一致性、`dt` = 1/f_pwm |
| `VoltageCommand(v_a,v_b,v_c)` | SVPWM 占空比 → TIM1 CCR | 过调制/钳位与 `InverterLimits.v_dc` 对齐 |
| `SensorSuite.measure` | ADC + Park | 三分流同步采样（见 §下，J5/J6 跳线） |
| `Inverter.apply`（死区/补偿） | TIM1 死区寄存器 + 软件补偿 | 死区用真实电流、补偿用测量电流 |
| `Observation`（真值） | 真机**无真值**，只有 `Measurements` | 评估时用高精编码器/示波器做近似真值 |

> **已落地实现**：`firmware/ihm07m1_foc/`（PlatformIO，STM32Cube HAL）已把控制律移植成 IHM07M1+F302R8 固件：
> - 有感 FOC（`foc.c` ← `FieldWeakeningFOC` / `01_foc_sensored.py`）
> - **无感 FOC**（`foc_sensorless.c` ← `BackEMFObserver`/`SensorlessFOC` / `02_sensorless_backemf.py`）
> - **参数自整定**（`param_id.c`）：上电静止自测 Rs/Ld/Lq，正是本节 §3.2 标定实验的在线版
>   （Rs=DC 注入差分、Ld/Lq=高频注入测纹波、ψ=旋转段 BEMF）。默认流程
>   `自测参数 → 无感 I/f 起转 → 反电动势观测器闭环`，无需位置传感器、无需预知电气参数。
>
> 算法核心是 `core/motorsim_core.py` 的逐行移植，并有 PC 端逐点回归
> （`bash firmware/ihm07m1_foc/test/run_host_test.sh`：Clarke/Park/SVPWM/电流环/反电动势观测器与 core 逐点对齐，
> 自整定用合成 R-L 电机回收已知参数）守住"固件==仿真"。`firmware/` 下每块硬件一个独立工程，
> 详见 `firmware/README.md`（工程索引）、`firmware/ihm07m1_foc/README.md` 与 `skills/pio/SKILL.md`。

### 2.2 IHM07M1 硬件约束（详见 `04_hardware_ihm07m1.md`）

- **必须三分流**（J5/J6）：低速/低调制下单分流相电流重构不可靠。
- 电流链 0.505 V/A，ADC 12-bit ±3.27A → LSB≈1.6mA；瓶颈是**模拟噪声**，需 PWM 中点同步采样。
- 电流上限被 **2.8A 峰值**卡死（小电机）；HFI 注入 ~17mA ≈ 10 LSB 余量，可用。
- 仿真对齐基线：`CurrentSensor(adc_bits=12,i_range=3.27,noise_std=0.003)` + `InverterLimits(v_dc=24,i_max=2.5)`。

### 2.3 上电与安全清单

1. **开环 I/f 先转**（`09_if_opencontrol.py` 的逻辑）确认相序、极对数 `p`、编码器方向。
2. **限幅先行**：母线限流、`i_max` 软保护、过温（绕组）、堵转超时。
3. **桨/负载脱开**做第一次闭环；逐步加 `setpoint` 与 `t_load`。
4. 每步**录波归档**（见 §3.1），失败也存——失败样本是模型对齐的高价值数据。

---

## 3. 用真机反馈修正 core 准确性

### 3.1 采集什么（与仿真同字段，便于逐点对比）

录波字段对齐 `Observation`/`Measurements`，统一 `dt`、时间戳、工况标签：

```
t, i_a,i_b,i_c, i_d,i_q, v_a,v_b,v_c(或占空比), theta_e(编码器), omega_m,
T_winding(若有热电偶), v_dc, setpoint, t_load(台架转矩), 工况标签
```

存为 `data/real/<工况>_<日期>.csv`，并记录电机/板子序列号、固件版本、室温。

### 3.2 参数 → 标定实验映射（改 core 的依据）

| core 参数 | 真机实验 | 反推方法 |
|-----------|----------|----------|
| `R0`, `a_cu`（温度系数） | 直流注入 / 冷热两态 | V/I；不同温度拟合 ΔR/ΔT |
| `Ld,Lq` | 高频小信号 / 锁转子 ±d±q | 阻抗法或电流斜率 di/dt |
| `psi0`（磁链）, `a_pm` | 反拖测 BEMF、变温 | E=ψ·ω 斜率；变温拟合 |
| `k_cross`（交叉饱和） | 变 `i_q` 下测凸极轴偏移 φ_sat | atan(2Ldq/(Ld−Lq)) 反解 Ldq=k·iq |
| `i_pm_sat,i_knee_sat` | ±d 脉冲电流不对称响应 | IPD 标定数据拟合 Ld(i_d) 曲线 |
| `kfe_h/e/a`（铁损） | 空载拖动损耗分离 | P_fe(ω,ψ) 三项拟合 |
| `J,B,Tc` | 自由减速 / 阶跃 | 减速曲线辨识 J、B、库仑摩擦 Tc |

> 全部为**向后兼容**改动：新现象参数默认值=关闭（`k_cross=0`、`i_pm_sat=0`），改的是默认 `MotorConfig` 预设，老脚本行为不变。

### 3.3 标定流程

1. 跑标定实验 → 录波到 `data/real/`。
2. 离线辨识脚本反推参数 → 产出一份 `MotorConfig` 预设（如 `presets/ihm07m1_motorX.py`）。
3. **同工况同输入**喂给仿真，比对 sim vs real（见 §4 指标）。
4. 误差超容差 → 检查是「参数不准」还是「模型缺现象」：
   - 参数偏置 → 调参数；
   - 系统性、随工况变化的偏差 → core 缺物理项 → 按 `agent.md` 流程**最小、向后兼容**地补 `_deriv`。

---

## 4. 可回归的迭代

目标：**每一次改 core 都能一键量化「离真机更近还是更远」**，杜绝「修了 A 退化了 B」。

### 4.1 黄金数据集（golden set）

- 固定一组覆盖工况的真机录波：低速/高速、轻载/重载、启动、阶跃、变温。
- 版本化在 `data/real/golden/`，附 `manifest`（工况、输入序列、设备/固件版本）。
- 这是回归的「真值」基准，只增不改（改动需新版本号）。

### 4.2 回归脚本与指标

对 golden set 每条记录：用**真机的输入序列**驱动仿真，逐点比对输出。

| 指标 | 定义 | 示例容差 |
|------|------|----------|
| 电流 RMSE | `rms(i_dq_sim − i_dq_real)` | < 5% 额定 |
| 角度偏差 | `|θ_e_sim − θ_e_real|` 稳态 | < 1°(电) |
| 速度阶跃 | 上升时间/超调差 | < 10% |
| 损耗/温升 | `P_fe`、稳态 `T_winding` | < 10% |

输出一份 `report.json`（每工况误差 + 总分），与上一版 diff。

### 4.3 防退化门槛

```bash
python3 tools/regress.py --golden data/real/golden --config presets/ihm07m1_motorX.py
# 退出码非 0 = 任一指标超容差或较基线变差 → 拒绝合入
```

- **基线快照**：每次接受的改动把 `report.json` 存为新基线。
- **门槛规则**：新报告任一指标不得比基线变差超过阈值；改善则更新基线。
- 适合挂到 CI / pre-merge，使 core 演进**单调向真机收敛**。

### 4.4 迭代节奏

```
改 core / 加物理项
   → 回归脚本对 golden set
   → 全绿且不退化? ──否──> 回滚或继续标定
            │是
   → 更新基线 + 更新 docs/02_physics_core.md 与 agent.md「待办/已知边界」
   → 下一轮（用新真机数据扩充 golden set）
```

---

## 5. 与现有待办的衔接

- 闭环无感定位重载 (`iq>1A`) 误差 >1°：先用 §2 部署 + §3.2 的 `k_cross`/IPD 标定取真机查表，
  替换解析交叉饱和补偿；再用 §4 回归确认重载误差收敛且不破坏轻载。
- EKF 隐极简化的 ~2°(电)负载相关偏置：用真机 BEMF 数据验证 EEMF 凸极模型后，作为 core 可选观测基类沉淀。
