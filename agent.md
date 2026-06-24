# Agent 指南（motorsim 项目）

写给在本仓库工作的 AI agent（如 Claude）。`claude.md` 是本文件的软链接。

## 项目目的

模块化 PMSM 仿真 + 无感控制研究。核心信条：**物理归 core，算法归 controller**。
信号链：`Controller → Inverter → Motor(物理) → Sensors → Controller`。

**双向演进**：controller 的探索不是终点，而是发现 core 短板的探针——算法跑不准、对不上硬件、缺接口，
往往暴露 core 在精准性 / 准确性 / 可扩展性 / 功能 / 生态上的缺口。探索要**反哺 core**，让 core 越用越强。
两条主线并行：① 横向拓展 controller 算法空间；② 纵向沉淀共性能力回 core。详见「core 反哺与演进」。

## 工作约定

- **语言**：与用户对话、代码注释、文档一律用**中文**；matplotlib 标签用**英文**（环境无 CJK 字体），正文中文。
- **不改 core 原则（默认）**：新增控制/观测算法 = 新 Controller，放 `control/` 或 `extensions/`，**默认不动 `core/`**。
  这是为避免「为单个算法私改 core」的随意改动，**不是禁止改进 core**——见下方「core 反哺与演进」。
  只有引入**新物理现象**（如磁极饱和）才改 `core/motorsim_core.py` 的 `_deriv`，且必须保持向后兼容（新参数默认值=关闭）。
- **三个 Protocol**：Controller / Inverter / SensorSuite，见 `docs/01_architecture.md`。实现其一即可插入。
- **运行**：`control/` 和 `extensions/` 的脚本顶部都有 sys.path 引导指向 `core/`，可直接 `python3 xxx.py`。
- **pip**：环境装包需 `--break-system-packages`。

## core 反哺与演进

controller 探索中暴露的 core 短板，要按下面五个维度沉淀回 core，让框架越用越强。
**判据**：凡是「多个 controller 都需要、且属于物理/平台/接口的共性能力」→ 入 core；只服务单算法的逻辑 → 留 controller。

- **精准性（数值）**：积分器精度、步长/刚性、状态量守恒。若 controller 的偏差源自 core 数值误差（而非算法本身），
  优先修 core 的 `_deriv`/积分器，而不是在 controller 里凑补偿。
- **准确性（物理保真）**：补全被简化的真实现象（交叉饱和查表、磁路饱和、铁损、温漂、死区/管压降细节），
  使仿真与 IHM07M1 等真实硬件对得上。新现象一律**向后兼容**（新参数默认关闭，老脚本行为不变）。
- **可扩展性（接口）**：当某类算法插不进现有 Protocol（Controller/Inverter/SensorSuite），
  应**扩展接口/新增 Protocol**回 core，而非在 controller 里绕过；保持「实现 Protocol 即可插入」的契约。
- **功能（共性能力）**：把多个 demo 重复造的轮子（标定子程序、查表补偿、状态估计基类、日志/绘图/指标工具）
  下沉为 core 可复用组件，避免 controller 间复制粘贴。
- **生态（可用性）**：稳定的参数预设（硬件对齐配置）、清晰的 Protocol 文档、可被外部 agent 调用的 skill、
  统一的实验/评测脚手架——降低新 controller 与新贡献者的接入成本。

**反哺流程**：① 在 controller 中复现并定位短板 → ② 判断归属（core 共性 / 算法私有）→
③ 若归 core，做**最小、向后兼容**的改动并更新 `docs/` 与本文件「待办/已知边界」→ ④ 用既有 demo 回归验证不破坏老行为。

## 目录

```
core/        motorsim_core.py(物理+编排+接口) / motorsim_inverter.py / motorsim_sensors.py
control/     01..10 控制算法 demo（见 docs/03_control_methods.md）
docs/        00 总览 / 01 架构 / 02 物理 / 03 控制方法 / 04 硬件
extensions/  custom_controller.py / custom_sensor.py 模板
skills/motorsim/SKILL.md   供外部 agent 调用
```

## 关键经验（改代码前必读）

- **方波 HFI 解调**：注入符号**周期初翻转**，注入与解调用**同一符号**（曾因周期末翻转导致锁偏跑飞）；
  相邻两周期解调取平均，抵消基波 iq 变化的污染。
- **交叉饱和**：高负载偏移凸极轴 φ_sat=½·atan(2·Ldq/(Ld−Lq))，必须补偿；重载需离线标定查表（解析式不够准）。
- **带宽分离**：电流环 ≫ PLL ≫ 速度环。
- **死区**用真实电流（物理），**补偿**用测量电流（控制）。
- **可观测性物理底线**：零速只有凸极(注入)可观测，高速只有基波可行；全速域必须融合，加权按 ωeψ/(Ri)。
- **硬件对齐**：IHM07M1 → `CurrentSensor(adc_bits=12,i_range=3.27,noise_std=0.003)`、`InverterLimits(24,2.5)`。

## 待办/已知边界

- 闭环无感定位重载下 (~iq>1A) 误差 >1°，因解析交叉饱和补偿不足；需调试离线标定子程序(HFI 在标定时可靠捕获)后改查表补偿。
- EKF 用隐极简化模型，有 ~2°(电)负载相关偏置；可换 EEMF 凸极模型消除。
