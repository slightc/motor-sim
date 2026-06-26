#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""regress.py —— 仿真↔实测防退化回归门槛（docs/05 §4）

对一组真机黄金录波（golden set），用**真机的输入序列**（施加电压 v_abc + 负载 t_load）
驱动仿真 core，逐点比对仿真输出 vs 真机测量。这隔离的是**电机模型(core 物理)的准确性**
——控制器不参与，纯粹考"同样的电压/负载下，仿真电流/角度/速度像不像真机"。

输出机读 report.json（每工况指标 + 是否在容差内 + 总判定）。可与上一版基线 diff：
任一指标超容差、或较基线变差超过 margin → 退出码非 0（拒绝合入）。

用法：
  python3 tools/regress.py --golden data/real/golden [--config presets/motorX.py]
                           [--baseline report.json] [--out report.json] [--i-rated 2.5]
  python3 tools/regress.py --selftest        # 用合成黄金自检整条链路（无需硬件）

指标与示例容差（docs/05 §4.2）：
  电流 RMSE  rms(i_dq_sim−i_dq_real)        < 5% 额定电流
  角度偏差   |θ_e_sim−θ_e_real| 稳态        < 1°(电)
  速度阶跃   上升时间/超调差                 < 10%
  温升       稳态 T_winding 误差             < 10%（有热电偶且开热模型时）
"""
import sys, os, csv, json, math, argparse, importlib.util, glob, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "core"))
from dataclasses import replace
from motorsim_core import (MotorPlant, MotorState, MotorConfig, ElectricalParams,
                           ThermalParams, MotorInput, clarke, park)

TWO_PI = 2.0 * math.pi

# 默认容差（docs/05 §4.2 的示例值）
TOL = {
    "i_dq_rmse_pct": 5.0,    # % 额定电流
    "theta_deg":     1.0,    # 电角度
    "speed_step_pct": 10.0,  # 上升时间/超调差 %
    "temp_pct":      10.0,   # 稳态温升 %
}
BASELINE_MARGIN = 0.05       # 较基线变差 >5% 视为退化


# ----------------------------- 数据加载 -----------------------------
def load_record(path):
    rows = []
    with open(path, newline="") as fp:
        for r in csv.DictReader(fp):
            rows.append(r)
    if len(rows) < 3:
        raise ValueError("录波 %s 行数过少" % path)
    return rows


def _f(row, key, default=float("nan")):
    v = row.get(key, "")
    if v is None or v == "":
        return default
    return float(v)


def load_config(path):
    """从 preset .py 取 MotorConfig：支持模块级 MOTOR 或 motor() 函数；否则默认 small_pmsm。"""
    if not path:
        return MotorConfig(electrical=ElectricalParams(R0=0.5, Ld=4e-3, Lq=6e-3, psi0=0.03, p=4),
                           thermal=ThermalParams(enabled=False)), "default(small_pmsm)"
    spec = importlib.util.spec_from_file_location("preset", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if hasattr(mod, "motor") and callable(mod.motor):
        return mod.motor(), os.path.basename(path)
    if hasattr(mod, "MOTOR"):
        return mod.MOTOR, os.path.basename(path)
    raise AttributeError("%s 需定义 motor()->MotorConfig 或 MOTOR" % path)


# ----------------------------- 仿真重放 -----------------------------
def replay(cfg, rows):
    """用录波里的 v_abc + t_load 驱动 plant，返回与录波对齐的仿真序列。
    初值取录波首行（电流 αβ / 速度 / 角度）。"""
    p = cfg.electrical.p
    r0 = rows[0]
    ia0, ib0, ic0 = _f(r0, "i_a", 0), _f(r0, "i_b", 0), _f(r0, "i_c", 0)
    ial0, ibe0 = clarke(ia0, ib0, ic0)
    thm0 = _f(r0, "theta_e", 0.0) / p
    wm0 = _f(r0, "omega_m", 0.0)
    th = cfg.thermal
    Tw0 = _f(r0, "T_winding", th.T_amb)
    init = MotorState(i_alpha=ial0, i_beta=ibe0, omega_m=wm0, theta_m=thm0,
                      T_w=Tw0, T_m=Tw0, T_s=th.T_amb)
    plant = MotorPlant(cfg, init_state=init)

    sim = {"i_d": [], "i_q": [], "theta_e": [], "omega_m": [], "T_winding": []}
    for i in range(len(rows) - 1):
        row = rows[i]
        dt = _f(rows[i + 1], "t") - _f(row, "t")
        if not (dt > 0):
            dt = 50e-6
        mi = MotorInput(v_a=_f(row, "v_a", 0), v_b=_f(row, "v_b", 0),
                        v_c=_f(row, "v_c", 0), t_load=_f(row, "t_load", 0))
        obs = plant.observe()
        sim["i_d"].append(obs.i_d); sim["i_q"].append(obs.i_q)
        sim["theta_e"].append(obs.theta_e % TWO_PI)
        sim["omega_m"].append(obs.state.omega_m)
        sim["T_winding"].append(obs.T_winding)
        plant.step(mi, dt)
    return sim


# ----------------------------- 指标 -----------------------------
def _ang_err(a, b):
    d = (a - b + math.pi) % TWO_PI - math.pi
    return abs(d)


def _step_metrics(t, y, setpoint):
    """检测最大 setpoint 阶跃，算上升时间(10→90%)与超调%。无明显阶跃返回 None。"""
    steps = [(i, setpoint[i] - setpoint[i - 1]) for i in range(1, len(setpoint))]
    if not steps:
        return None
    i0, mag = max(steps, key=lambda s: abs(s[1]))
    if abs(mag) < 1e-6:
        return None
    y0, yf = y[i0 - 1], setpoint[i0]
    span = yf - y0
    if abs(span) < 1e-6:
        return None
    th10, th90 = y0 + 0.1 * span, y0 + 0.9 * span
    t10 = t90 = None
    for i in range(i0, len(y)):
        if t10 is None and (y[i] - th10) * (1 if span > 0 else -1) >= 0:
            t10 = t[i]
        if t90 is None and (y[i] - th90) * (1 if span > 0 else -1) >= 0:
            t90 = t[i]; break
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else float("nan")
    peak = (max(y[i0:]) if span > 0 else min(y[i0:]))
    overshoot = max(0.0, ((peak - yf) / span) * 100.0)
    return {"rise_s": rise, "overshoot_pct": overshoot}


def metrics(rows, sim, cfg, i_rated):
    n = len(sim["i_d"])
    t = [_f(rows[i], "t") for i in range(n)]
    # 实测
    id_r = [_f(rows[i], "i_d") for i in range(n)]
    iq_r = [_f(rows[i], "i_q") for i in range(n)]
    th_r = [_f(rows[i], "theta_e") % TWO_PI for i in range(n)]
    wm_r = [_f(rows[i], "omega_m") for i in range(n)]
    sp = [_f(rows[i], "setpoint", float("nan")) for i in range(n)]

    # 电流 dq RMSE（合并 d、q）
    se = 0.0
    for i in range(n):
        se += (sim["i_d"][i] - id_r[i]) ** 2 + (sim["i_q"][i] - iq_r[i]) ** 2
    i_rmse = math.sqrt(se / (2 * n))
    i_rmse_pct = 100.0 * i_rmse / i_rated if i_rated > 0 else float("nan")

    # 稳态窗口 = 后 50%
    h = n // 2
    ang = sum(math.degrees(_ang_err(sim["theta_e"][i], th_r[i])) for i in range(h, n)) / (n - h)

    # 速度阶跃（real vs sim 的上升/超调差）
    spd_diff_pct = float("nan")
    if not any(math.isnan(x) for x in sp):
        sm_r = _step_metrics(t, wm_r, sp)
        sm_s = _step_metrics(t, sim["omega_m"], sp)
        if sm_r and sm_s and sm_r["rise_s"] > 0:
            rise_diff = abs(sm_s["rise_s"] - sm_r["rise_s"]) / sm_r["rise_s"] * 100.0
            ov_diff = abs(sm_s["overshoot_pct"] - sm_r["overshoot_pct"])
            spd_diff_pct = max(rise_diff, ov_diff)

    # 温升（有列且开热模型）
    temp_pct = None
    if cfg.thermal.enabled and not math.isnan(_f(rows[0], "T_winding")):
        Tr = sum(_f(rows[i], "T_winding") for i in range(h, n)) / (n - h)
        Ts = sum(sim["T_winding"][i] for i in range(h, n)) / (n - h)
        rise_r = max(1e-6, Tr - cfg.thermal.T_amb)
        temp_pct = 100.0 * abs(Ts - Tr) / rise_r

    m = {
        "n": n,
        "i_dq_rmse": round(i_rmse, 6),
        "i_dq_rmse_pct": round(i_rmse_pct, 3),
        "theta_deg": round(ang, 4),
    }
    if not math.isnan(spd_diff_pct):
        m["speed_step_pct"] = round(spd_diff_pct, 3)
    if temp_pct is not None:
        m["temp_pct"] = round(temp_pct, 3)
    return m


def judge(m):
    """逐指标对容差判定，返回 (pass_bool, fails:list)。"""
    fails = []
    if m["i_dq_rmse_pct"] > TOL["i_dq_rmse_pct"]:
        fails.append("i_dq_rmse_pct %.2f>%.1f" % (m["i_dq_rmse_pct"], TOL["i_dq_rmse_pct"]))
    if m["theta_deg"] > TOL["theta_deg"]:
        fails.append("theta_deg %.2f>%.1f" % (m["theta_deg"], TOL["theta_deg"]))
    if "speed_step_pct" in m and m["speed_step_pct"] > TOL["speed_step_pct"]:
        fails.append("speed_step_pct %.1f>%.1f" % (m["speed_step_pct"], TOL["speed_step_pct"]))
    if "temp_pct" in m and m["temp_pct"] > TOL["temp_pct"]:
        fails.append("temp_pct %.1f>%.1f" % (m["temp_pct"], TOL["temp_pct"]))
    return (len(fails) == 0), fails


def vs_baseline(report, baseline):
    """与基线 diff：任一指标变差 > margin 视为退化。返回退化项列表。"""
    regress = []
    base = {r["record"]: r["metrics"] for r in baseline.get("records", [])}
    for rec in report["records"]:
        bm = base.get(rec["record"])
        if not bm:
            continue
        for k in ("i_dq_rmse_pct", "theta_deg", "speed_step_pct", "temp_pct"):
            if k in rec["metrics"] and k in bm:
                new, old = rec["metrics"][k], bm[k]
                if new > old * (1.0 + BASELINE_MARGIN) and new > old + 1e-9:
                    regress.append("%s: %s %.3f→%.3f" % (rec["record"], k, old, new))
    return regress


# ----------------------------- 主流程 -----------------------------
def run(golden_dir, config_path, i_rated, out_path, baseline_path):
    cfg, cfg_name = load_config(config_path)
    csvs = sorted(glob.glob(os.path.join(golden_dir, "*.csv")))
    if not csvs:
        print("未找到黄金录波 CSV：%s" % golden_dir); return 2

    report = {"config": cfg_name, "i_rated": i_rated, "tolerances": TOL, "records": []}
    all_pass = True
    for path in csvs:
        rows = load_record(path)
        sim = replay(cfg, rows)
        m = metrics(rows, sim, cfg, i_rated)
        ok, fails = judge(m)
        all_pass = all_pass and ok
        report["records"].append({
            "record": os.path.basename(path), "pass": ok, "fails": fails, "metrics": m,
        })
        flag = "PASS" if ok else "FAIL"
        extra = ("" if ok else "  <%s>" % "; ".join(fails))
        print("[%s] %-34s i_dq=%.2f%% θ=%.2f°%s" %
              (flag, os.path.basename(path), m["i_dq_rmse_pct"], m["theta_deg"], extra))

    regress = []
    if baseline_path and os.path.exists(baseline_path):
        with open(baseline_path) as fp:
            regress = vs_baseline(report, json.load(fp))
        for r in regress:
            print("[退化] %s" % r)

    report["overall_pass"] = bool(all_pass and not regress)
    if out_path:
        with open(out_path, "w") as fp:
            json.dump(report, fp, ensure_ascii=False, indent=2)
        print("写出 %s" % out_path)

    print("总判定：%s" % ("PASS" if report["overall_pass"] else "FAIL"))
    return 0 if report["overall_pass"] else 1


def selftest():
    """无硬件自检：合成黄金 → 同模型 replay 应近零误差(PASS)；扰动模型应被门槛拦下(FAIL)。"""
    import make_synthetic_golden
    print("== selftest: 生成合成黄金（临时目录，不入 repo）==")
    gdir = make_synthetic_golden.main(os.path.join(tempfile.mkdtemp(), "golden"))

    print("\n== A) 同模型 replay（应 PASS，误差≈0）==")
    ra = run(gdir, None, 2.5, None, None)

    print("\n== B) 扰动模型 R0 0.5→0.9（应 FAIL，门槛拦截）==")
    pert = os.path.join(tempfile.gettempdir(), "preset_pert.py")
    with open(pert, "w") as fp:
        fp.write("import sys, os\n"
                 "sys.path.insert(0, os.path.join(%r, '..', 'core'))\n"
                 "from motorsim_core import MotorConfig, ElectricalParams, ThermalParams\n"
                 "def motor():\n"
                 "    return MotorConfig(electrical=ElectricalParams(R0=0.9, Ld=4e-3, Lq=6e-3, psi0=0.03, p=4),\n"
                 "                       thermal=ThermalParams(enabled=False))\n" % HERE)
    rb = run(gdir, pert, 2.5, None, None)

    ok = (ra == 0 and rb == 1)
    print("\nselftest %s（同模型 PASS=%s，扰动 FAIL=%s）" %
          ("PASS" if ok else "FAIL", ra == 0, rb == 1))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="仿真↔实测防退化回归（docs/05 §4）")
    ap.add_argument("--golden", help="黄金录波目录（*.csv）")
    ap.add_argument("--config", help="MotorConfig preset .py（motor()->MotorConfig 或 MOTOR）")
    ap.add_argument("--baseline", help="基线 report.json，用于防退化 diff")
    ap.add_argument("--out", default="report.json", help="输出 report.json 路径")
    ap.add_argument("--i-rated", type=float, default=2.5, help="额定电流(A)，电流 RMSE 归一化用")
    ap.add_argument("--selftest", action="store_true", help="无硬件自检整条链路")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(selftest())
    if not a.golden:
        ap.error("需要 --golden <目录> 或 --selftest")
    sys.exit(run(a.golden, a.config, a.i_rated, a.out, a.baseline))


if __name__ == "__main__":
    main()
