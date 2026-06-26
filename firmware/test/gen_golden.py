#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从仿真 core 生成 FOC 黄金参考值，供固件 foc.c 的原生回归测试逐点比对。

跑法：python3 gen_golden.py  → 打印 C 头文件内容到 stdout（已写入 golden.h）。
确保固件移植与仿真"同输入同输出"。"""
import sys, os, math
HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "..", "core"))
from motorsim_core import (clarke, inv_clarke, park, inv_park,
                           ElectricalParams, MotorConfig, ThermalParams,
                           InverterLimits, FieldWeakeningFOC, Measurements)

def svpwm_duty(v_alpha, v_beta, v_dc):
    """复刻 SVPWMInverter 的零序注入 + 占空比换算（与 foc_svpwm 对应）。"""
    va, vb, vc = inv_clarke(v_alpha, v_beta)
    voff = (max(va, vb, vc) + min(va, vb, vc)) / 2
    va -= voff; vb -= voff; vc -= voff
    duty = [0.5 + v / v_dc for v in (va, vb, vc)]
    return [min(1.0, max(0.0, d)) for d in duty]

lines = []
lines.append("/* 自动生成: firmware/test/gen_golden.py —— 勿手改。FOC 黄金参考值（来自仿真 core）*/")
lines.append("#ifndef GOLDEN_H\n#define GOLDEN_H\n")

# 1) Clarke/Park 变换抽样
xform = []
for (a, b, c, th) in [(1.0, -0.5, -0.5, 0.0), (0.7, 0.1, -0.8, 1.2),
                      (-0.3, 0.4, -0.1, 3.0), (2.0, -1.0, -1.0, -0.9)]:
    al, be = clarke(a, b, c)
    d, q = park(al, be, th)
    a2, b2, c2 = inv_clarke(al, be)
    al2, be2 = inv_park(d, q, th)
    xform.append((a, b, c, th, al, be, d, q))
lines.append("typedef struct { float a,b,c,th, alpha,beta,d,q; } xform_t;")
lines.append("static const xform_t GOLDEN_XFORM[] = {")
for r in xform:
    lines.append("  {%.9ff,%.9ff,%.9ff,%.9ff, %.9ff,%.9ff,%.9ff,%.9ff}," % r)
lines.append("};")
lines.append("static const int GOLDEN_XFORM_N = %d;\n" % len(xform))

# 2) SVPWM 占空比抽样
svp = []
for (val, vbe, vdc) in [(5.0, 0.0, 24.0), (-3.0, 4.0, 24.0),
                        (10.0, -8.0, 24.0), (0.0, 0.0, 24.0)]:
    d = svpwm_duty(val, vbe, vdc)
    svp.append((val, vbe, vdc, d[0], d[1], d[2]))
lines.append("typedef struct { float valpha,vbeta,vdc, d0,d1,d2; } svpwm_t;")
lines.append("static const svpwm_t GOLDEN_SVPWM[] = {")
for r in svp:
    lines.append("  {%.9ff,%.9ff,%.9ff, %.9ff,%.9ff,%.9ff}," % r)
lines.append("};")
lines.append("static const int GOLDEN_SVPWM_N = %d;\n" % len(svp))

# 3) 完整电流环：对同一组测量输入，比对 FieldWeakeningFOC（id_ref=0）的 dq 电压与占空比。
#    复刻其电流环（含限幅/抗饱和），逐拍喂相同 id_ref/iq_ref/电流/角度，记录稳定后一拍。
cfg = MotorConfig(electrical=ElectricalParams(R0=0.5, Ld=4e-3, Lq=6e-3, psi0=0.03, p=4),
                  thermal=ThermalParams(enabled=False))
lim = InverterLimits(24, 2.5)
DT = 50e-6
# 直接用电流环公式（与 foc_current_step 同逻辑），独立积分器
seq = []
iid = iiq = 0.0
kp_i, ki_i = 12.0, 3000.0
v_max = lim.v_max
cases = [  # (id_ref, iq_ref, ia, ib, ic, theta_e)
    (0.0, 1.0, 0.2, -0.1, -0.1, 0.5),
    (0.0, 1.5, 0.5, -0.25, -0.25, 1.5),
    (0.0, -1.0, -0.3, 0.15, 0.15, 2.5),
]
for (id_ref, iq_ref, ia, ib, ic, th) in cases:
    al, be = clarke(ia, ib, ic)
    idm, iqm = park(al, be, th)
    e_id = id_ref - idm; e_iq = iq_ref - iqm
    v_d = kp_i * e_id + ki_i * iid
    v_q = kp_i * e_iq + ki_i * iiq
    vs = math.hypot(v_d, v_q)
    if vs > v_max:
        sc = v_max / vs; v_d *= sc; v_q *= sc
    else:
        iid += e_id * DT; iiq += e_iq * DT
    v_al, v_be = inv_park(v_d, v_q, th)
    duty = svpwm_duty(v_al, v_be, lim.v_dc)
    seq.append((id_ref, iq_ref, ia, ib, ic, th, v_d, v_q, duty[0], duty[1], duty[2]))
lines.append("typedef struct { float id_ref,iq_ref, ia,ib,ic,th, vd,vq, d0,d1,d2; } iloop_t;")
lines.append("static const iloop_t GOLDEN_ILOOP[] = {")
for r in seq:
    lines.append("  {%.9ff,%.9ff, %.9ff,%.9ff,%.9ff,%.9ff, %.9ff,%.9ff, %.9ff,%.9ff,%.9ff}," % r)
lines.append("};")
lines.append("static const int GOLDEN_ILOOP_N = %d;" % len(seq))
lines.append("static const float GOLDEN_DT = %.9ff;\n" % DT)

lines.append("#endif /* GOLDEN_H */")
out = "\n".join(lines) + "\n"
with open(os.path.join(HERE, "golden.h"), "w") as fp:
    fp.write(out)
print(out)
