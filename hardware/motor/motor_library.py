# -*- coding: utf-8 -*-
"""
motorsim.hardware.motor —— 可复用电机配置库

把散落在各处的电机参数集中成**命名的、可复用的 MotorConfig 工厂**。
硬件 Profile（如 ihm07m1）和 control/ demo 都从这里取电机，不再各自硬编码一份。

每个工厂返回**全新**的 MotorConfig（dataclass 可变，避免多个 plant 共享同一实例），
并支持关键字覆盖：

    from motor_library import get_motor, small_pmsm
    m = small_pmsm()                          # 基础小型 PMSM
    m = small_pmsm(thermal=False)             # 关热模型
    m = get_motor("small_pmsm_salient")       # 带凸极/饱和（HFI/磁极辨识用）
    m = get_motor("small_pmsm", R0=0.42, p=4) # 覆盖电气参数（真机标定回填）

新增电机 = 在此加一个工厂并登记到 MOTORS。电机电磁参数应由真机标定回填
（见 docs/05_hardware_deployment.md）。
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "core"))

from dataclasses import replace
from motorsim_core import (
    MotorConfig, ElectricalParams, MechanicalParams, ThermalParams,
)


def _make(name, electrical, mechanical=None, thermal=True, **e_overrides):
    """统一装配：electrical 覆盖 + 机械默认 + 热模型开关。"""
    if e_overrides:
        electrical = replace(electrical, **e_overrides)
    th = ThermalParams(enabled=bool(thermal), T_amb=25.0) if not isinstance(thermal, ThermalParams) \
        else thermal
    return MotorConfig(
        name=name,
        electrical=electrical,
        mechanical=mechanical or MechanicalParams(J=6.0e-4, B=1.5e-4, Tc=0.02),
        thermal=th,
    )


# ---------------- 电机预设 ----------------
def small_pmsm(thermal=True, **e_overrides) -> MotorConfig:
    """基础小型 PMSM（IHM07M1 量级）。control/ 多数 demo 与硬件默认用它。
    R0=0.5Ω Ld=4mH Lq=6mH ψ=0.03Wb p=4。凸极/饱和默认关闭。"""
    return _make("small_pmsm",
                 ElectricalParams(R0=0.5, Ld=4.0e-3, Lq=6.0e-3, psi0=0.03, p=4),
                 thermal=thermal, **e_overrides)


def small_pmsm_salient(thermal=False, k_cross=-3.2e-4,
                       i_pm_sat=4.0, i_knee_sat=6.0, **e_overrides) -> MotorConfig:
    """带交叉饱和 + d 轴极性饱和的小型 PMSM。
    用于 HFI 凸极无感、磁极辨识(IPD)、闭环无感位置伺服等需要真实凸极物理的研究。
    （对应 control/06,07,10 的参数）"""
    return _make("small_pmsm_salient",
                 ElectricalParams(R0=0.5, Ld=4.0e-3, Lq=6.0e-3, psi0=0.03, p=4,
                                  k_cross=k_cross, i_pm_sat=i_pm_sat, i_knee_sat=i_knee_sat),
                 thermal=thermal, **e_overrides)


def hfi_high_flux(thermal=False, k_cross=-3.2e-4, **e_overrides) -> MotorConfig:
    """高磁链凸极电机（ψ=0.08Wb），对应 control/03 脉振 HFI 标定场景。"""
    return _make("hfi_high_flux",
                 ElectricalParams(Ld=4.0e-3, Lq=6.0e-3, psi0=0.08, p=4, k_cross=k_cross),
                 thermal=thermal, **e_overrides)


# ---------------- 注册表 ----------------
MOTORS = {
    "small_pmsm": small_pmsm,
    "small_pmsm_salient": small_pmsm_salient,
    "hfi_high_flux": hfi_high_flux,
}


def get_motor(name: str, **kw) -> MotorConfig:
    """按名取电机工厂并构造（带关键字覆盖）。"""
    if name not in MOTORS:
        raise KeyError("未知电机 %r，可选: %s" % (name, list(MOTORS)))
    return MOTORS[name](**kw)


def list_motors():
    """返回 (名称, 一行说明) 列表。"""
    return [(n, (f.__doc__ or "").strip().splitlines()[0]) for n, f in MOTORS.items()]


if __name__ == "__main__":
    for n, doc in list_motors():
        m = get_motor(n)
        e = m.electrical
        print("%-20s R0=%-4s Ld=%.1fmH Lq=%.1fmH ψ=%-5s p=%d k_cross=%s i_pm=%s | %s"
              % (n, e.R0, e.Ld*1e3, e.Lq*1e3, e.psi0, e.p, e.k_cross, e.i_pm_sat, doc))
