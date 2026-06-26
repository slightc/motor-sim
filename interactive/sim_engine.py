# -*- coding: utf-8 -*-
"""
交互式仿真引擎：后台线程跑电机物理 + 可选控制算法，把"完整状态"以满采样率
写入一个固定容量的环形缓冲（保留最近 window_s 秒，超出即覆盖最旧）。

设计要点：
  - 纯标准库（无 numpy）。环形缓冲用 array('d')（每通道 8 字节/样本，C 连续）。
  - 录制满积分率（record 每个 RK4 步），10s 滚动窗口。
  - 查询用"按列 min/max 抽稀"（浏览器性能工具的渲染套路）：无论窗口多宽，
    每个像素列只回传 (min,max)，既不漏尖峰也不爆带宽。
  - 仿真线程与 HTTP 读线程并发：写指针在锁内快照，min/max 扫描在锁外做
    （读到正在被覆盖的最旧样本只会让最左边缘有极小毛刺，可接受）。
"""
import sys, os, time, threading, array, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))
from dataclasses import replace
from motorsim_core import (
    MotorConfig, ElectricalParams, MechanicalParams, ThermalParams,
    MotorPlant, MotorState, MotorInput, VoltageCommand, InverterLimits, Measurements,
    IdealInverter, FieldWeakeningFOC, SensorlessFOC, clarke, park, inv_clarke, inv_park,
)
from motorsim_sensors import SensorSuite, IdealCurrentSensor, IdealEncoder
from motorsim_inverter import SVPWMInverter

DT = 20e-6  # 仿真步长（固定 20µs -> 50kHz 满积分率）

# ---------------- 可记录的"完整状态"通道定义 ----------------
# (key, 标签, 单位, 分组, 颜色)
CHANNELS = [
    ("i_a",       "i_a",        "A",    "相电流", "#e6194b"),
    ("i_b",       "i_b",        "A",    "相电流", "#3cb44b"),
    ("i_c",       "i_c",        "A",    "相电流", "#4363d8"),
    ("i_d",       "i_d",        "A",    "dq电流", "#f58231"),
    ("i_q",       "i_q",        "A",    "dq电流", "#911eb4"),
    ("omega_m",   "ω_m 实际",   "rad/s","转速",   "#42d4f4"),
    ("ref_speed", "ω_m 给定",   "rad/s","转速",   "#bfef45"),
    ("omega_e",   "ω_e 电角速", "rad/s","转速",   "#469990"),
    ("theta_e",   "θ_e 电角",   "rad",  "角度",   "#f032e6"),
    ("torque",    "电磁转矩",   "N·m",  "转矩",   "#e6194b"),
    ("load",      "负载转矩",   "N·m",  "转矩",   "#808000"),
    ("p_copper",  "铜损",       "W",    "功率",   "#ffe119"),
    ("p_iron",    "铁损",       "W",    "功率",   "#f58231"),
    ("back_emf",  "反电动势",   "V",    "电压",   "#000075"),
    ("v_mag",     "电压幅值",   "V",    "电压",   "#9a6324"),
    ("T_winding", "绕组温度",   "°C",   "温度",   "#e6194b"),
    ("T_magnet",  "磁体温度",   "°C",   "温度",   "#f58231"),
    ("T_housing", "壳体温度",   "°C",   "温度",   "#4363d8"),
]
CHANNEL_KEYS = [c[0] for c in CHANNELS]


def default_config():
    return MotorConfig(
        name="PMSM",
        electrical=ElectricalParams(R0=0.5, Ld=4.0e-3, Lq=6.0e-3, psi0=0.05, p=4),
        mechanical=MechanicalParams(J=6.0e-4, B=1.5e-4, Tc=0.02),
        thermal=ThermalParams(enabled=True),
    )


# ---------------- 开环 V/Hz 控制器（无反馈，演示启停） ----------------
class OpenLoopVHz:
    """开环压频比：电角度按给定速度积分，沿 q 轴施加正比于速度的电压。无位置反馈。"""
    def __init__(self, cfg, lim):
        self.p = cfg.electrical.p
        self.lim = lim
        self.theta = 0.0
        self.v_min = 1.0       # 启动励磁
        self.v_per_w = 0.06    # 压频比斜率

    def compute(self, meas, setpoint, dt):
        self.theta = (self.theta + self.p * setpoint * dt) % (2 * math.pi)
        v_q = min(self.lim.v_max, self.v_min + self.v_per_w * abs(setpoint)) * (1 if setpoint >= 0 else -1)
        va, vb, vc = inv_clarke(*inv_park(0.0, v_q, self.theta))
        return VoltageCommand(va, vb, vc)


# 控制器注册表：name -> 工厂(cfg, lim) -> controller
CONTROLLERS = {
    "foc_sensored":   ("有感 FOC（弱磁）",      lambda cfg, lim: FieldWeakeningFOC(cfg, lim)),
    "foc_sensorless": ("无感 FOC（反电动势）",  lambda cfg, lim: SensorlessFOC(cfg, lim)),
    "openloop_vhz":   ("开环 V/Hz",             lambda cfg, lim: OpenLoopVHz(cfg, lim)),
}
INVERTERS = {
    "ideal": "理想逆变器",
    "svpwm": "SVPWM（10kHz 开关）",
}


# ---------------- 环形缓冲 ----------------
class RingBuffer:
    def __init__(self, keys, capacity):
        self.keys = keys
        self.N = capacity
        self.t = array.array('d', [0.0]) * self.N
        self.buf = {k: array.array('d', [0.0]) * self.N for k in keys}
        self.head = 0      # 下一个写入位置
        self.count = 0
        self.lock = threading.Lock()

    def push(self, t, vals):
        with self.lock:
            p = self.head
            self.t[p] = t
            for k in self.keys:
                self.buf[k][p] = vals[k]
            self.head = (p + 1) % self.N
            if self.count < self.N:
                self.count += 1

    def clear(self):
        with self.lock:
            self.head = 0
            self.count = 0

    def snapshot_meta(self):
        with self.lock:
            head, count, N = self.head, self.count, self.N
            latest = self.t[(head - 1) % N] if count else 0.0
            oldest = self.t[(head - count) % N] if count else 0.0
        return head, count, N, oldest, latest

    def query(self, t0, t1, keys, width):
        """按 [t0,t1] 窗口、width 个像素列返回每通道的 (min,max) 抽稀序列。"""
        head, count, N, oldest, latest = self.snapshot_meta()
        if count == 0 or width <= 0 or t1 <= t0:
            return {k: {"min": [], "max": [], "last": None} for k in keys}, oldest, latest
        start = (head - count) % N  # 逻辑 0 对应的环位置
        t = self.t

        def t_at(i):
            return t[(start + i) % N]

        # 找第一个 t_at(i) >= tt 的逻辑下标
        def lower(tt):
            lo, hi = 0, count
            while lo < hi:
                mid = (lo + hi) >> 1
                if t_at(mid) < tt:
                    lo = mid + 1
                else:
                    hi = mid
            return lo

        span = t1 - t0
        edges = [lower(t0 + span * c / width) for c in range(width + 1)]
        out = {}
        for k in keys:
            b = self.buf[k]
            mn = [None] * width
            mx = [None] * width
            for c in range(width):
                a = edges[c]; e = edges[c + 1]
                if e <= a:
                    continue
                pa = (start + a) % N
                length = e - a
                if pa + length <= N:                 # 不跨环：单段 C 级 min/max
                    s = b[pa:pa + length]
                    mn[c] = round(min(s), 4); mx[c] = round(max(s), 4)
                else:                                # 跨环：两段合并
                    s1 = b[pa:N]; s2 = b[0:length - (N - pa)]
                    lo_v = min(min(s1), min(s2)); hi_v = max(max(s1), max(s2))
                    mn[c] = round(lo_v, 4); mx[c] = round(hi_v, 4)
            last_pos = (head - 1) % N
            out[k] = {"min": mn, "max": mx, "last": round(b[last_pos], 4)}
        return out, oldest, latest


# ---------------- 仿真引擎 ----------------
class SimEngine:
    def __init__(self, window_s=10.0):
        self.window_s = window_s
        self.dt = DT
        capacity = int(round(window_s / self.dt)) + 1   # 满采样率容量
        self.ring = RingBuffer(CHANNEL_KEYS, capacity)

        # 可调运行参数（受 state_lock 保护）
        self.state_lock = threading.Lock()
        self.controller_name = "foc_sensored"
        self.inverter_name = "ideal"
        self.v_dc = 48.0
        self.i_max = 10.0
        self.ref_speed = 0.0
        self.load_torque = 0.0
        self.speed_scale = 1.0     # 仿真时间相对墙钟的倍率 0.25~2
        self.enabled = False       # 电机驱动启停
        self.paused = False        # 仿真时钟冻结

        self._rt_ratio = 0.0       # 实测实时倍率
        self.t_clock = 0.0         # 单调录制时钟（独立于 plant 重建，保证缓冲时间单调）
        self._build()              # 构建 plant/controller/...

        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)

    # ---- 构建/重置 ----
    def _build(self):
        self.cfg = default_config()
        self.lim = InverterLimits(self.v_dc, self.i_max)
        self.plant = MotorPlant(self.cfg, init_state=MotorState(omega_m=0.0))
        self.controller = CONTROLLERS[self.controller_name][1](self.cfg, self.lim)
        self.sensors = SensorSuite(IdealCurrentSensor(), IdealEncoder())
        self.f_pwm = 10000.0
        if self.inverter_name == "svpwm":
            self.inverter = SVPWMInverter(v_dc=self.v_dc, f_pwm=self.f_pwm, dead_time=0.0)
        else:
            self.inverter = IdealInverter()
        # 数字控制环：以 PWM 速率运行（控制 ISR），电压指令在两次更新之间保持，
        # 这是真实 FOC 的工作方式，也避免在每个仿真步上让 SVPWM 开关纹波进入无感观测器。
        self.f_ctrl = self.f_pwm
        self._ctrl_nsub = max(1, int(round((1.0 / self.f_ctrl) / self.dt)))
        # 电流抗混叠低通：仅 SVPWM（有开关纹波）启用；理想逆变器无纹波，直通避免观测器偏置
        self._i_fc = 1500.0 if self.inverter_name == "svpwm" else 0.0
        self._held_cmd = VoltageCommand(0.0, 0.0, 0.0)
        self._fi = [0.0, 0.0]   # αβ 电流低通状态
        self._ctrl_k = 0

    def reset(self):
        with self.state_lock:
            self.enabled = False
            self._build()
        # 清空环形缓冲（录制时钟保持单调，不回退）
        self.ring.clear()

    def start(self):
        self._thread.start()

    # ---- 控制接口 ----
    def apply_config(self, controller=None, inverter=None, v_dc=None, i_max=None):
        with self.state_lock:
            if controller in CONTROLLERS:
                self.controller_name = controller
            if inverter in INVERTERS:
                self.inverter_name = inverter
            if v_dc is not None:
                self.v_dc = float(v_dc)
            if i_max is not None:
                self.i_max = float(i_max)
        self.reset()

    def set_cmd(self, enabled=None, ref=None, load=None, speed=None, paused=None):
        with self.state_lock:
            if enabled is not None:
                self.enabled = bool(enabled)
            if ref is not None:
                self.ref_speed = float(ref)
            if load is not None:
                self.load_torque = float(load)
            if speed is not None:
                self.speed_scale = max(0.25, min(2.0, float(speed)))
            if paused is not None:
                self.paused = bool(paused)

    def status(self):
        head, count, N, oldest, latest = self.ring.snapshot_meta()
        with self.state_lock:
            st = {
                "controller": self.controller_name,
                "inverter": self.inverter_name,
                "v_dc": self.v_dc, "i_max": self.i_max,
                "ref_speed": self.ref_speed, "load_torque": self.load_torque,
                "speed_scale": self.speed_scale,
                "enabled": self.enabled, "paused": self.paused,
            }
        st.update({
            "sim_t": latest, "buf_oldest": oldest,
            "buf_count": count, "buf_capacity": N,
            "window_s": self.window_s, "dt": self.dt,
            "rt_ratio": round(self._rt_ratio, 3),
            "mem_mb": round(N * (len(CHANNEL_KEYS) + 1) * 8 / 1e6, 1),
        })
        return st

    def query(self, t0, t1, keys, width):
        keys = [k for k in keys if k in CHANNEL_KEYS] or CHANNEL_KEYS
        return self.ring.query(t0, t1, keys, width)

    # ---- 后台仿真循环 ----
    def _loop(self):
        tick_wall = 0.02  # 每 20ms 墙钟做一批 + 配速
        while not self._stop:
            t_start = time.perf_counter()
            with self.state_lock:
                paused = self.paused
                speed = self.speed_scale
                enabled = self.enabled
                ref = self.ref_speed
                load = self.load_torque
                controller = self.controller
                inverter = self.inverter
                sensors = self.sensors
                plant = self.plant
            if paused:
                time.sleep(tick_wall)
                continue

            nsub = self._ctrl_nsub
            Tc = 1.0 / self.f_ctrl
            i_fc = self._i_fc
            a_i = (self.dt / (1.0 / (2 * math.pi * i_fc) + self.dt)) if i_fc > 0 else 1.0
            pos = sensors.position
            steps = max(1, int(round(tick_wall * speed / self.dt)))
            for _ in range(steps):
                obs = plant.observe()
                # 连续抗混叠低通后的 αβ 电流（模拟 ADC 前端；理想逆变器 a_i=1 即直通）
                ial, ibe = clarke(obs.i_a, obs.i_b, obs.i_c)
                self._fi[0] += a_i * (ial - self._fi[0])
                self._fi[1] += a_i * (ibe - self._fi[1])
                # 数字控制环：每 nsub 个仿真步（= 一个 PWM 周期）更新一次，期间保持电压
                if self._ctrl_k % nsub == 0:
                    if enabled:
                        theta_e, omega_m = pos.read(obs, Tc)
                        fial, fibe = self._fi[0], self._fi[1]
                        ia_f, ib_f, ic_f = inv_clarke(fial, fibe)
                        i_d, i_q = park(fial, fibe, theta_e)
                        meas = Measurements(t=obs.t, i_a=ia_f, i_b=ib_f, i_c=ic_f,
                                            i_d=i_d, i_q=i_q, theta_e=theta_e, omega_m=omega_m,
                                            theta_e_true=obs.theta_e)
                        self._held_cmd = controller.compute(meas, ref, Tc)
                    else:
                        self._held_cmd = VoltageCommand(0.0, 0.0, 0.0)
                self._ctrl_k += 1
                minp = inverter.apply(self._held_cmd, obs, self.dt)
                minp = replace(minp, t_load=load)
                plant.step(minp, self.dt)
                o = plant.observe()
                self.t_clock += self.dt
                self.ring.push(self.t_clock, {
                    "i_a": o.i_a, "i_b": o.i_b, "i_c": o.i_c,
                    "i_d": o.i_d, "i_q": o.i_q,
                    "omega_m": o.state.omega_m, "ref_speed": ref, "omega_e": o.omega_e,
                    "theta_e": o.theta_e % (2 * math.pi),  # 包裹成锯齿（电角度，便于示波观察）
                    "torque": o.torque, "load": load,
                    "p_copper": o.p_copper, "p_iron": o.p_iron,
                    "back_emf": o.back_emf, "v_mag": o.v_mag,
                    "T_winding": o.T_winding, "T_magnet": o.T_magnet, "T_housing": o.T_housing,
                })

            elapsed = time.perf_counter() - t_start
            sim_advanced = steps * self.dt
            # 实测实时倍率（仿真秒/墙钟秒）
            self._rt_ratio = sim_advanced / elapsed if elapsed > 0 else 0.0
            target = sim_advanced / speed   # 这批本应占用的墙钟
            slack = target - elapsed
            if slack > 0:
                time.sleep(slack)
