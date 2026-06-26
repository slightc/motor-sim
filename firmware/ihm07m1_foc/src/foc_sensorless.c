/* -*- coding: utf-8 -*-
 * foc_sensorless.c —— 无感 FOC：反电动势观测器 + PLL（纯算法，零硬件依赖）
 *
 * 逐行对应 core/motorsim_core.py 的 BackEMFObserver 与 SensorlessFOC：
 *   e_hat = v - R·i - L·di/dt → 低通 → PLL 锁相 → 估计 θ/ω。
 * 与 foc.c 一样可在 PC 上独立编译跑回归（见 firmware/test/）。
 */
#include "foc.h"
#include <math.h>

#define TWO_PI 6.283185307179586f

/* ---------------- 反电动势观测器 ---------------- */
void bemf_obs_init(bemf_obs_t *o, float R, float L, int pole_pairs, float f_lp) {
    o->R = R; o->L = L; o->pole_pairs = pole_pairs;
    o->kp_pll = 300.0f; o->ki_pll = 20000.0f;   /* 仿真默认 */
    o->f_lp = f_lp;
    o->theta = 0.0f; o->omega_e = 0.0f; o->pll_i = 0.0f;
    o->i_prev_a = o->i_prev_b = 0.0f; o->has_prev = 0;
    o->e_lp_a = o->e_lp_b = 0.0f;
}

void bemf_obs_preset(bemf_obs_t *o, float theta0, float omega_e0) {
    o->theta = theta0;
    o->omega_e = omega_e0;
    o->pll_i = omega_e0;          /* PLL 积分初值 = 电角速度（对应仿真预置）*/
}

void bemf_obs_update(bemf_obs_t *o, float ia, float ib, float va, float vb,
                     float dt, float *theta, float *omega_e) {
    if (!o->has_prev) { o->i_prev_a = ia; o->i_prev_b = ib; o->has_prev = 1; }
    float di_a = (ia - o->i_prev_a) / dt;
    float di_b = (ib - o->i_prev_b) / dt;
    o->i_prev_a = ia; o->i_prev_b = ib;

    /* 反电动势 = 端电压 − 电阻压降 − 电感压降 */
    float e_a = va - o->R * ia - o->L * di_a;
    float e_b = vb - o->R * ib - o->L * di_b;

    /* 一阶低通抑制 di/dt 噪声（与仿真同式：a = dt/(1/(2πf)+dt)）*/
    float a = dt / (1.0f / (TWO_PI * o->f_lp) + dt);
    o->e_lp_a += a * (e_a - o->e_lp_a);
    o->e_lp_b += a * (e_b - o->e_lp_b);
    float ea = o->e_lp_a, eb = o->e_lp_b;

    /* PLL：e = |e|·[−sinθ, cosθ]，误差 ∝ sin(θ−θ_est)，按 |e| 归一化 */
    float emag = sqrtf(ea * ea + eb * eb) + 1e-6f;
    float eps = -(ea * cosf(o->theta) + eb * sinf(o->theta)) / emag;
    o->pll_i += o->ki_pll * eps * dt;
    o->omega_e = o->kp_pll * eps + o->pll_i;
    o->theta += o->omega_e * dt;
    o->theta = fmodf(o->theta, TWO_PI);
    if (o->theta < 0.0f) o->theta += TWO_PI;

    *theta = o->theta;
    *omega_e = o->omega_e;
}

/* ---------------- 无感 FOC ---------------- */
void foc_set_sensorless_gains(foc_t *f) {
    /* 仿真 SensorlessFOC：速度环 0.5/8，电流环沿用 12/3000 */
    f->kp_w = 0.5f; f->ki_w = 8.0f;
    f->kp_i = 12.0f; f->ki_i = 3000.0f;
}

void sensorless_init(sensorless_t *s, const foc_t *f, float f_lp) {
    /* 观测器用 R=Rs, L=Lq（与仿真一致：凸极取 q 轴电感做平均近似）*/
    bemf_obs_init(&s->obs, f->m.Rs, f->m.Lq, f->m.pole_pairs, f_lp);
    s->v_prev_a = s->v_prev_b = 0.0f;
    s->omega_m_est = 0.0f;
}

float sensorless_step(sensorless_t *s, foc_t *f, float omega_ref,
                      float ia, float ib, float ic, float dt) {
    float ialpha, ibeta;
    foc_clarke(ia, ib, ic, &ialpha, &ibeta);

    /* 1) 观测器：用上一拍施加电压解算转子角 */
    float theta_est, omega_e_est;
    bemf_obs_update(&s->obs, ialpha, ibeta, s->v_prev_a, s->v_prev_b,
                    dt, &theta_est, &omega_e_est);
    s->omega_m_est = omega_e_est / (float)f->m.pole_pairs;

    /* 2) 速度外环 → iq_ref（id_ref=0），3) 电流内环 + SVPWM（复用 foc.c）*/
    float iq_ref = foc_speed_step(f, omega_ref, s->omega_m_est, dt);
    foc_current_step(f, 0.0f, iq_ref, ia, ib, ic, theta_est, dt);

    /* 4) 记录本拍施加的 αβ 参考电压，供下一拍观测器 */
    foc_inv_park(f->v_d, f->v_q, theta_est, &s->v_prev_a, &s->v_prev_b);
    return s->omega_m_est;
}
