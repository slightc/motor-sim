#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成**合成**黄金录波（不是真机数据，仅供 tools/regress.py 自检/演示）。

跑一段有感 FOC 仿真，按 docs/05 §3.1 的字段逐拍写 CSV，模拟一条"真机录波"。
真机录波应由实际台架采集替换（同字段、同 dt），存到 data/real/golden/（版本化）。

用法：python3 tools/make_synthetic_golden.py [输出目录]
  默认写到系统临时目录（不污染 repo）；regress.py --selftest 即用它。
"""
import sys, os, csv, json, math, tempfile
HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "core"))
from dataclasses import replace
from motorsim_core import (MotorPlant, MotorState, MotorConfig, ElectricalParams,
                           ThermalParams, InverterLimits, FieldWeakeningFOC,
                           IdealInverter, clarke, park)

FIELDS = ["t", "i_a", "i_b", "i_c", "i_d", "i_q",
          "v_a", "v_b", "v_c", "theta_e", "omega_m",
          "T_winding", "v_dc", "setpoint", "t_load", "label"]


def main(out_dir=None):
    OUT_DIR = out_dir or os.path.join(tempfile.gettempdir(), "motorsim_synth_golden")
    OUT_CSV = os.path.join(OUT_DIR, "synthetic_smallpmsm_v0.csv")
    cfg = MotorConfig(electrical=ElectricalParams(R0=0.5, Ld=4e-3, Lq=6e-3, psi0=0.03, p=4),
                      thermal=ThermalParams(enabled=False))
    lim = InverterLimits(24, 2.5)
    DT = 50e-6
    plant = MotorPlant(cfg, init_state=MotorState(omega_m=0.0))
    foc = FieldWeakeningFOC(cfg, lim)
    inv = IdealInverter()

    os.makedirs(OUT_DIR, exist_ok=True)
    n = int(0.10 / DT)            # 0.1s，逐拍记录（replay 需同 dt 同输入）
    with open(OUT_CSV, "w", newline="") as fp:
        w = csv.DictWriter(fp, fieldnames=FIELDS)
        w.writeheader()
        for k in range(n):
            t = plant.t
            obs = plant.observe()
            sp = 30.0 if t < 0.05 else 50.0      # 0.05s 速度阶跃
            cmd = foc.compute(_meas(obs), sp, DT)
            mi = replace(inv.apply(cmd, obs, DT), t_load=0.10)
            # 记录"施加电压"为本拍 MotorInput（replay 的驱动输入）
            w.writerow({
                "t": "%.7f" % t,
                "i_a": "%.6f" % obs.i_a, "i_b": "%.6f" % obs.i_b, "i_c": "%.6f" % obs.i_c,
                "i_d": "%.6f" % obs.i_d, "i_q": "%.6f" % obs.i_q,
                "v_a": "%.6f" % mi.v_a, "v_b": "%.6f" % mi.v_b, "v_c": "%.6f" % mi.v_c,
                "theta_e": "%.6f" % obs.theta_e, "omega_m": "%.6f" % obs.state.omega_m,
                "T_winding": "%.3f" % obs.T_winding,
                "v_dc": "24.0", "setpoint": "%.3f" % sp, "t_load": "0.100",
                "label": "speedstep_30to50",
            })
            plant.step(mi, DT)

    manifest = {
        "note": "SYNTHETIC golden for regress.py self-test — NOT real hardware data.",
        "records": [os.path.basename(OUT_CSV)],
        "dt": DT, "motor": "small_pmsm(R0=0.5,Ld=4e-3,Lq=6e-3,psi0=0.03,p=4)",
        "inverter": "ideal", "v_dc": 24.0, "i_rated": 2.5,
        "conditions": "speed step 30->50 rad/s @0.05s, load 0.10 N·m",
    }
    with open(os.path.join(OUT_DIR, "manifest.json"), "w") as fp:
        json.dump(manifest, fp, ensure_ascii=False, indent=2)
    print("wrote %s (%d rows)" % (OUT_CSV, n))
    print("wrote %s" % os.path.join(OUT_DIR, "manifest.json"))
    return OUT_DIR


def _meas(obs):
    """理想测量（合成时控制器吃真值），仅供生成用。"""
    from motorsim_core import Measurements
    return Measurements(t=obs.t, i_a=obs.i_a, i_b=obs.i_b, i_c=obs.i_c,
                        i_d=obs.i_d, i_q=obs.i_q, theta_e=obs.theta_e,
                        omega_m=obs.state.omega_m, theta_e_true=obs.theta_e)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
