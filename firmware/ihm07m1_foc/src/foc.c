/* -*- coding: utf-8 -*-
 * foc.c —— 基础 FOC 算法实现（纯 C，零硬件依赖）
 *
 * 逐行对应 core/motorsim_core.py：clarke/park/inv_park/inv_clarke、FieldWeakeningFOC
 * 的电流/速度双环、SVPWMInverter 的零序注入。区别仅在输出：仿真把"理想电压"直接
 * 施加，固件把电压换算成占空比写 TIM1 CCR。基础 FOC 固定 id_ref=0（无弱磁/MTPA）。
 */
#include "foc.h"
#include <math.h>

#define SQRT3      1.7320508075688772f
#define INV_SQRT3  0.5773502691896258f

/* ---------------- Clarke / Park（与仿真同式）---------------- */
void foc_clarke(float a, float b, float c, float *alpha, float *beta) {
    *alpha = (2.0f / 3.0f) * (a - 0.5f * b - 0.5f * c);
    *beta  = INV_SQRT3 * (b - c);
}
void foc_inv_clarke(float alpha, float beta, float *a, float *b, float *c) {
    *a = alpha;
    *b = -0.5f * alpha + (SQRT3 / 2.0f) * beta;
    *c = -0.5f * alpha - (SQRT3 / 2.0f) * beta;
}
void foc_park(float alpha, float beta, float th, float *d, float *q) {
    float c = cosf(th), s = sinf(th);
    *d =  alpha * c + beta * s;
    *q = -alpha * s + beta * c;
}
void foc_inv_park(float d, float q, float th, float *alpha, float *beta) {
    float c = cosf(th), s = sinf(th);
    *alpha = d * c - q * s;
    *beta  = d * s + q * c;
}

/* ---------------- SVPWM：αβ 电压 → 三相占空比（对应 SVPWMInverter.apply）------------
 * 仿真里：inv_clarke 得三相参考 → 零序注入 voff=(max+min)/2 → 载波比较。
 * 占空比（上桥导通占比）= 0.5 + v_phase / v_dc，钳位 [0,1]。 */
void foc_svpwm(float v_alpha, float v_beta, float v_dc, float duty[3]) {
    float va, vb, vc;
    foc_inv_clarke(v_alpha, v_beta, &va, &vb, &vc);

    float vmax = va, vmin = va;
    if (vb > vmax) vmax = vb;
    if (vb < vmin) vmin = vb;
    if (vc > vmax) vmax = vc;
    if (vc < vmin) vmin = vc;
    float voff = 0.5f * (vmax + vmin);   /* 零序注入，提升母线利用率 */
    va -= voff; vb -= voff; vc -= voff;

    float inv_vdc = (v_dc > 1e-6f) ? (1.0f / v_dc) : 0.0f;
    duty[0] = 0.5f + va * inv_vdc;
    duty[1] = 0.5f + vb * inv_vdc;
    duty[2] = 0.5f + vc * inv_vdc;
    for (int i = 0; i < 3; ++i) {
        if (duty[i] < 0.0f) duty[i] = 0.0f;
        if (duty[i] > 1.0f) duty[i] = 1.0f;
    }
}

/* ---------------- 初始化 / 复位 ---------------- */
void foc_init(foc_t *f, const motor_params_t *m, float v_dc, float i_max) {
    f->m = *m;
    /* PI 增益取仿真 FieldWeakeningFOC 默认值 */
    f->kp_i = 12.0f;  f->ki_i = 3000.0f;
    f->kp_w = 0.6f;   f->ki_w = 10.0f;
    f->v_dc = v_dc;
    f->v_max = v_dc * INV_SQRT3;   /* 对应 InverterLimits.v_max = v_dc/sqrt(3) */
    f->i_max = i_max;
    foc_reset(f);
}

void foc_reset(foc_t *f) {
    f->iid = f->iiq = f->iw = 0.0f;
    f->id_ref = f->iq_ref = 0.0f;
    f->i_d = f->i_q = f->v_d = f->v_q = 0.0f;
    f->duty[0] = f->duty[1] = f->duty[2] = 0.5f;  /* 0.5 = 零矢量（三相等占空，无线电压）*/
}

/* ---------------- 电流内环（对应 FieldWeakeningFOC.compute 电流环）---------------- */
void foc_current_step(foc_t *f, float id_ref, float iq_ref,
                      float ia, float ib, float ic, float theta_e, float dt) {
    /* 测量三相电流 → αβ → dq（用电角度 Park，与数字 FOC 一致）*/
    float ialpha, ibeta, id, iq;
    foc_clarke(ia, ib, ic, &ialpha, &ibeta);
    foc_park(ialpha, ibeta, theta_e, &id, &iq);
    f->i_d = id; f->i_q = iq;
    f->id_ref = id_ref; f->iq_ref = iq_ref;

    /* d/q 电流 PI */
    float e_id = id_ref - id;
    float e_iq = iq_ref - iq;
    float v_d = f->kp_i * e_id + f->ki_i * f->iid;
    float v_q = f->kp_i * e_iq + f->ki_i * f->iiq;

    /* 电压矢量幅值限幅 + 抗积分饱和（仅未饱和时积分，与仿真一致）*/
    float vs = sqrtf(v_d * v_d + v_q * v_q);
    if (vs > f->v_max) {
        float sc = f->v_max / vs;
        v_d *= sc; v_q *= sc;
    } else {
        f->iid += e_id * dt;
        f->iiq += e_iq * dt;
    }
    f->v_d = v_d; f->v_q = v_q;

    /* dq 电压 → αβ → SVPWM 占空比 */
    float v_alpha, v_beta;
    foc_inv_park(v_d, v_q, theta_e, &v_alpha, &v_beta);
    foc_svpwm(v_alpha, v_beta, f->v_dc, f->duty);
}

/* ---------------- 速度外环（对应 FieldWeakeningFOC.compute 速度环）---------------- */
float foc_speed_step(foc_t *f, float omega_ref, float omega_meas, float dt) {
    float e_w = omega_ref - omega_meas;
    float iq_cmd = f->kp_w * e_w + f->ki_w * f->iw;
    /* 抗饱和：仅当输出未触限时积分 */
    if (iq_cmd > -f->i_max && iq_cmd < f->i_max) {
        f->iw += e_w * dt;
    }
    if (iq_cmd >  f->i_max) iq_cmd =  f->i_max;
    if (iq_cmd < -f->i_max) iq_cmd = -f->i_max;
    return iq_cmd;
}
