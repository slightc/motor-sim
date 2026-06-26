# -*- coding: utf-8 -*-
"""示例 MotorConfig 预设 —— 供 tools/regress.py 的 --config 用。

真机标定（docs/05 §3.2）后，把反推的电气/机械/热参数回填到这里，产出一份命名预设
（如 presets/ihm07m1_motorX.py），回归就用它驱动仿真，量化"仿真离这台真机多近"。

格式：定义 motor() 返回 MotorConfig（或定义模块级 MOTOR）。
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))
from motorsim_core import MotorConfig, ElectricalParams, MechanicalParams, ThermalParams


def motor() -> MotorConfig:
    # 占位默认（IHM07M1 量级，small_pmsm）；真机标定后替换这些数。
    return MotorConfig(
        name="ihm07m1_example",
        electrical=ElectricalParams(R0=0.5, Ld=4.0e-3, Lq=6.0e-3, psi0=0.03, p=4),
        mechanical=MechanicalParams(J=6.0e-4, B=1.5e-4, Tc=0.02),
        thermal=ThermalParams(enabled=False),   # 有热电偶录 T_winding 时设 True
    )
