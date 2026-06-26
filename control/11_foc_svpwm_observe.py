import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))
# -*- coding: utf-8 -*-
"""有感 FOC + SVPWM 波形观测。

经真实 SVPWMInverter（零序注入 + 载波比较 + 开关）驱动电机，
逐点录波并绘图观测三类信号：
  ① SVPWM 波形：调制参考(马鞍波) + 载波 + 三相桥臂开关输出
  ② 电流：三相 i_a/i_b/i_c + dq 轴 i_d/i_q(测量 vs 参考)
  ③ 电机状态：转速跟踪、电磁转矩、电角度

为分辨 PWM 开关，用 DT=2µs(500kHz) 仿真，f_pwm=10kHz → 每周期 50 点。
控制器按真实数字 FOC 在 PWM 频率(10kHz)更新一次并对指令零阶保持(ZOH)，
仿真物理在 2µs 细步积分——避免控制器逐细步反应 PWM 纹波而污染调制波。

本 demo 用理想电流/位置传感器，目的是看清 SVPWM 与电流的"教科书"波形：
唯一残留纹波即逆变器开关产生的真实 PWM 电流纹波。真实编码器/电流传感器
的量化与噪声引入的额外纹波见 01_foc_sensored.py 等(用真 Encoder)。
matplotlib 标签用英文(环境无 CJK 字体)，终端输出用中文。
"""
from motorsim_core import *
from motorsim_sensors import *
from motorsim_inverter import SVPWMInverter
import math
from dataclasses import replace
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------- 场景选择 ----------------
# clean : 48V 留裕度 + 理想传感器 -> 教科书清晰波形(默认)
# real  : 24V(IHM07M1) + 真实编码器 -> 含电压饱和极限环 + 编码器量化纹波(贴近硬件)
SCENARIO = sys.argv[1] if len(sys.argv) > 1 else "clean"
assert SCENARIO in ("clean", "real"), "用法: python3 11_foc_svpwm_observe.py [clean|real]"

# ---------------- 电机 / 逆变器 / 控制器 装配 ----------------
cfg = MotorConfig(electrical=ElectricalParams(R0=0.5, Ld=4e-3, Lq=6e-3, psi0=0.03, p=4),
                  thermal=ThermalParams(enabled=False))
DT = 2e-6                       # 500kHz 物理积分步长，足够分辨 10kHz PWM 开关
F_PWM = 10000.0
CTRL_DECIM = round((1.0 / F_PWM) / DT)   # 控制更新分频：每 50 细步=1 个 PWM 周期更新一次
T_END = 0.30
T_STEP = 0.15                   # 速度阶跃时刻

if SCENARIO == "clean":
    V_DC = 48.0                 # 母线留足裕度，避免弱磁/电压饱和的极限环纹波
    lim = InverterLimits(V_DC, 8.0)
    SPD_LO, SPD_HI = 30.0, 60.0
    T_LOAD = 0.30
    sens = SensorSuite(current=IdealCurrentSensor(), position=IdealEncoder())
else:  # real：IHM07M1 对齐(24V/2.5A) + 真实增量编码器
    V_DC = 24.0
    lim = InverterLimits(V_DC, 2.5)
    SPD_LO, SPD_HI = 30.0, 50.0
    T_LOAD = 0.10
    sens = SensorSuite(current=IdealCurrentSensor(), position=Encoder(2500, p=4))

plant = MotorPlant(cfg, init_state=MotorState(omega_m=0.0))
foc = FieldWeakeningFOC(cfg, lim)
inv = SVPWMInverter(v_dc=V_DC, f_pwm=F_PWM, dead_time=0.0)

# ---------------- 录波缓冲 ----------------
N = int(T_END / DT)
t_buf      = np.empty(N)
ia, ib, ic = np.empty(N), np.empty(N), np.empty(N)
idt, iqt   = np.empty(N), np.empty(N)     # 真值 dq 电流(真电角变换，干净)
idr, iqr   = np.empty(N), np.empty(N)     # 参考 dq 电流
sp_buf     = np.empty(N)                   # 速度设定
wm_buf     = np.empty(N)                   # 实际机械转速
tq_buf     = np.empty(N)                   # 电磁转矩
the_buf    = np.empty(N)                   # 电角度
vma, vmb, vmc = np.empty(N), np.empty(N), np.empty(N)   # SVPWM 调制参考(马鞍波)
carr_buf   = np.empty(N)                   # 三角载波
la, lb, lc = np.empty(N), np.empty(N), np.empty(N)      # 三相桥臂开关输出

# ---------------- 主循环 ----------------
DT_CTRL = CTRL_DECIM * DT                               # 控制周期 = 1/f_pwm
cmd = VoltageCommand()                                  # ZOH 保持的三相参考电压
meas = sens.measure(plant.observe(), DT_CTRL)           # 首次测量(供首个 PWM 周期记录)
sp = SPD_LO
for k in range(N):
    t = plant.t
    to = plant.observe()
    if k % CTRL_DECIM == CTRL_DECIM // 2:               # 在载波中点(谷)采样+更新：纹波平均，真实数字 FOC 做法
        meas = sens.measure(to, DT_CTRL)
        sp = SPD_LO if t < T_STEP else SPD_HI
        cmd = foc.compute(meas, sp, DT_CTRL)
    minp = inv.apply(cmd, to, DT)                       # SVPWM 物理：每细步开关，写入 inv.probe
    plant.step(replace(minp, t_load=T_LOAD), DT)

    t_buf[k] = t
    ia[k], ib[k], ic[k] = to.i_a, to.i_b, to.i_c
    idt[k], iqt[k] = to.i_d, to.i_q
    idr[k], iqr[k] = foc.id_ref, foc.iq_ref
    sp_buf[k] = sp
    wm_buf[k] = to.state.omega_m
    tq_buf[k] = to.torque
    the_buf[k] = to.theta_e % (2 * math.pi)
    (vma[k], vmb[k], vmc[k]) = inv.probe['v_mod']
    carr_buf[k] = inv.probe['carrier']
    (la[k], lb[k], lc[k]) = inv.probe['leg']

# ---------------- 控制台摘要 ----------------
ss = t_buf > (T_END - 0.02)                              # 末段 20ms 稳态窗
print("=" * 60)
print("有感 FOC + SVPWM 波形观测")
print("=" * 60)
print(f"仿真: DT={DT*1e6:.0f}µs, f_pwm={F_PWM/1e3:.0f}kHz, V_dc={V_DC}V, 时长={T_END}s")
print(f"速度阶跃: {SPD_LO:.0f} -> {SPD_HI:.0f} rad/s @ {T_STEP}s, 负载={T_LOAD}N·m")
print("-" * 60)
print(f"末速        : {wm_buf[-1]:.2f} rad/s (目标 {SPD_HI:.0f})")
print(f"稳态均速    : {wm_buf[ss].mean():.2f} rad/s, 速度纹波 ±{wm_buf[ss].std():.3f}")
print(f"稳态 i_q    : {iqt[ss].mean():.3f} A (参考 {iqr[ss].mean():.3f})")
print(f"稳态 i_d    : {idt[ss].mean():.3f} A (参考 {idr[ss].mean():.3f}, MTPA<0)")
print(f"稳态转矩    : {tq_buf[ss].mean():.4f} N·m")
print(f"三相电流峰值: {np.max(np.abs([ia[ss],ib[ss],ic[ss]])):.3f} A")
print(f"调制马鞍波峰值: {np.max(np.abs([vma,vmb,vmc])):.3f} V (V_dc/√3={V_DC/math.sqrt(3):.2f})")

# ---------------- 绘图 ----------------
plt.rcParams.update({
    "font.size": 12, "axes.titlesize": 13, "axes.titleweight": "bold",
    "axes.labelsize": 12, "legend.fontsize": 10, "lines.antialiased": True,
})
# 全时程面板(①②④)逐细步=15万点会糊成墨带；按控制率抽取(每 PWM 周期 1 点)→ 干净
D = CTRL_DECIM
td = t_buf[::D]
fig = plt.figure(figsize=(17, 12))
_sub = ("clean: 48V + ideal sensors" if SCENARIO == "clean"
        else "real: 24V (IHM07M1) + quantized encoder")
fig.suptitle(f"Sensored FOC with SVPWM Inverter — Waveform Observation [{_sub}]",
             fontsize=17, fontweight="bold")

# 选稳态电气周期窗口用于细节观测（电频率 = p*wm/2pi ≈ 4*45.8/2pi ≈ 29Hz）
w_el = cfg.electrical.p * float(wm_buf[-1])              # 末态电角速度 rad/s
T_el = 2 * math.pi / w_el                                # 电气周期 ~34ms
N_EL = 1.5                                               # 取末 1.5 个电周期(形状清晰)
z0 = T_END - N_EL * T_el
zoom = (t_buf >= z0) & (t_buf <= z0 + N_EL * T_el)
# PWM 开关细节：取更窄窗口(~3 个 PWM 周期)
p0 = T_END - 0.005
pwm = (t_buf >= p0) & (t_buf <= p0 + 3 / F_PWM)

ax = plt.subplot(3, 2, 1)                                # ① 转速跟踪
ax.plot(td, sp_buf[::D], "k--", lw=1.6, label="setpoint")
ax.plot(td, wm_buf[::D], "b", lw=1.6, label="omega_m")
ax.set_title("Motor State: Speed Tracking"); ax.set_xlabel("t [s]"); ax.set_ylabel("omega_m [rad/s]")
ax.legend(loc="lower right"); ax.grid(alpha=0.3)

ax = plt.subplot(3, 2, 2)                                # ② 转矩(控制率抽取 + 包络)
ax.plot(td, tq_buf[::D], "g", lw=1.2, label="Te")
ax.axhline(T_LOAD, color="r", ls="--", lw=1.4, label=f"load {T_LOAD} N·m")
ax.set_title("Motor State: Electromagnetic Torque"); ax.set_xlabel("t [s]"); ax.set_ylabel("Te [N·m]")
ax.legend(loc="upper right"); ax.grid(alpha=0.3)

ax = plt.subplot(3, 2, 3)                                # ③ 三相电流(稳态周期窗)
ax.plot(t_buf[zoom]*1e3, ia[zoom], "r", lw=1.6, label="i_a")
ax.plot(t_buf[zoom]*1e3, ib[zoom], "g", lw=1.6, label="i_b")
ax.plot(t_buf[zoom]*1e3, ic[zoom], "b", lw=1.6, label="i_c")
ax.set_title("Current: 3-Phase (steady-state, 1.5 electrical periods)")
ax.set_xlabel("t [ms]"); ax.set_ylabel("i [A]"); ax.legend(loc="upper right", ncol=3); ax.grid(alpha=0.3)

ax = plt.subplot(3, 2, 4)                                # ④ dq 电流(真值抽取 vs 参考)
ax.plot(td, iqt[::D], "b", lw=1.4, label="i_q (true)")
ax.plot(td, iqr[::D], "c--", lw=1.6, label="i_q ref")
ax.plot(td, idt[::D], "r", lw=1.4, label="i_d (true)")
ax.plot(td, idr[::D], "m--", lw=1.6, label="i_d ref")
ax.set_title("Current: dq-axis (true vs reference)")
ax.set_xlabel("t [s]"); ax.set_ylabel("i [A]"); ax.legend(loc="center right", ncol=2); ax.grid(alpha=0.3)

ax = plt.subplot(3, 2, 5)                                # ⑤ SVPWM 调制马鞍波(稳态周期窗)
ax.plot(t_buf[zoom]*1e3, vma[zoom], "r", lw=1.6, label="v_a* (mod)")
ax.plot(t_buf[zoom]*1e3, vmb[zoom], "g", lw=1.6, label="v_b* (mod)")
ax.plot(t_buf[zoom]*1e3, vmc[zoom], "b", lw=1.6, label="v_c* (mod)")
ax.set_title("SVPWM: Modulation Reference (saddle, zero-seq injected)")
ax.set_xlabel("t [ms]"); ax.set_ylabel("v* [V]"); ax.legend(loc="upper right", ncol=3); ax.grid(alpha=0.3)

ax = plt.subplot(3, 2, 6)                                # ⑥ SVPWM 开关细节(窄窗)
tp = t_buf[pwm]*1e6
ax.plot(tp, carr_buf[pwm], color="0.55", lw=1.4, label="carrier")
ax.plot(tp, vma[pwm], "r", lw=2.0, label="v_a* mod")
ax.step(tp, la[pwm], "k", lw=1.8, where="post", label="leg_a (switch)")
ax.set_title("SVPWM: Carrier Compare & Leg Switching (phase A, 3 PWM periods)")
ax.set_xlabel("t [µs]"); ax.set_ylabel("v [V]"); ax.legend(loc="upper right"); ax.grid(alpha=0.3)

plt.tight_layout(rect=[0, 0, 1, 0.975])
out_dir = os.path.join(os.path.dirname(__file__), "..", "data", "sim")
os.makedirs(out_dir, exist_ok=True)
_name = "11_foc_svpwm_observe.png" if SCENARIO == "clean" else f"11_foc_svpwm_observe_{SCENARIO}.png"
out = os.path.abspath(os.path.join(out_dir, _name))
plt.savefig(out, dpi=200)    # 17x12in @200dpi = 3400x2400 px (>2K)
print("-" * 60)
print(f"波形已保存: {out}")
