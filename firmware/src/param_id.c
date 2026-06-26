/* -*- coding: utf-8 -*-
 * param_id.c —— 电机参数自整定状态机（静止测 Rs/Ld/Lq）
 *
 * 全程 θ_e=0：d 轴 = α 轴，q 轴 = β 轴（park/inv_park 退化），故
 *   id = i_alpha, iq = i_beta；施加 v_alpha=Vd, v_beta=Vq。
 * 纯公式 pid_calc_* 可主机回归（firmware/test/）。
 */
#include "param_id.h"
#include <math.h>

/* ---- 纯公式 ---- */
float pid_calc_rs(float dV, float dI) {
    if (fabsf(dI) < 1e-6f) return 0.0f;
    return dV / dI;
}
float pid_calc_L(float v_inj, float dt, float mean_abs_di) {
    if (mean_abs_di < 1e-9f) return 0.0f;
    return v_inj * dt / mean_abs_di;          /* 方波注入：|Δi|/拍 ≈ V·dt/L */
}
float pid_calc_psi(float v_q, float Rs, float i_q, float omega_e) {
    if (fabsf(omega_e) < 1e-3f) return 0.0f;
    return (v_q - Rs * i_q) / omega_e;        /* 稳态 q 轴电压方程 */
}

/* ---- 相位时长（秒）---- */
#define T_ALIGN   0.5f
#define T_RS      0.3f
#define T_IND     0.15f
#define T_SETTLE  0.03f      /* 每相位前段不计入平均（等待稳态）*/

void param_id_init(param_id_t *p, float v_dc, float i_max, float dt) {
    p->v_dc = v_dc; p->dt = dt;
    p->v_max = v_dc * 0.5773502691896258f;    /* v_dc/sqrt(3) */
    /* 保守默认：电流远低于 i_max，注入电压取母线一小部分 */
    p->i_align    = 0.5f * i_max;
    p->i_rs_low   = 0.3f * i_max;
    p->i_rs_high  = 0.6f * i_max;
    p->v_inj      = 3.0f;                      /* 高频注入幅值 (V) */
    p->kp = 3.0f; p->ki = 600.0f; p->integ = 0.0f;

    p->phase = PID_ALIGN; p->tick = 0; p->inj_sign = 1; p->prev_i = 0.0f;
    p->v_acc = 0.0f; p->v_cnt = 0; p->i_acc = 0.0f;
    p->di_acc = 0.0f; p->di_cnt = 0;
    p->v1 = p->i1 = 0.0f;
    p->Rs = p->Ld = p->Lq = 0.0f; p->ok = 0;
}

/* 自带 d 轴电流 PI（保持/Rs 用）：返回该轴参考电压，带限幅+抗饱和 */
static float pid_pi(param_id_t *p, float i_ref, float i_meas) {
    float e = i_ref - i_meas;
    float v = p->kp * e + p->ki * p->integ;
    if (v > p->v_max) v = p->v_max;
    else if (v < -p->v_max) v = -p->v_max;
    else p->integ += e * p->dt;               /* 未饱和才积分 */
    return v;
}

static void enter(param_id_t *p, pid_phase_t ph) {
    p->phase = ph; p->tick = 0; p->integ = 0.0f;
    p->v_acc = 0.0f; p->v_cnt = 0; p->i_acc = 0.0f;
    p->di_acc = 0.0f; p->di_cnt = 0; p->inj_sign = 1;
}

pid_phase_t param_id_step(param_id_t *p, float ia, float ib, float ic, float duty[3]) {
    float ialpha, ibeta;
    foc_clarke(ia, ib, ic, &ialpha, &ibeta);
    float id = ialpha, iq = ibeta;            /* θ=0 退化 */
    float v_alpha = 0.0f, v_beta = 0.0f;

    const int n_align = (int)(T_ALIGN / p->dt);
    const int n_rs    = (int)(T_RS / p->dt);
    const int n_ind   = (int)(T_IND / p->dt);
    const int n_settle= (int)(T_SETTLE / p->dt);

    switch (p->phase) {
    case PID_ALIGN:
        v_alpha = pid_pi(p, p->i_align, id);
        if (++p->tick >= n_align) enter(p, PID_RS_LOW);
        break;

    case PID_RS_LOW:
        v_alpha = pid_pi(p, p->i_rs_low, id);
        if (p->tick >= n_settle) { p->v_acc += v_alpha; p->i_acc += id; p->v_cnt++; }
        if (++p->tick >= n_rs) {
            p->v1 = p->v_acc / p->v_cnt; p->i1 = p->i_acc / p->v_cnt;
            enter(p, PID_RS_HIGH);
        }
        break;

    case PID_RS_HIGH:
        v_alpha = pid_pi(p, p->i_rs_high, id);
        if (p->tick >= n_settle) { p->v_acc += v_alpha; p->i_acc += id; p->v_cnt++; }
        if (++p->tick >= n_rs) {
            float v2 = p->v_acc / p->v_cnt, i2 = p->i_acc / p->v_cnt;
            p->Rs = pid_calc_rs(v2 - p->v1, i2 - p->i1);
            enter(p, PID_LD);
            p->prev_i = 0.0f;
        }
        break;

    case PID_LD:
        /* d 轴方波注入（无 PI），测 α 轴电流纹波 */
        v_alpha = (float)p->inj_sign * p->v_inj;
        v_beta = 0.0f;
        if (p->tick >= n_settle) { p->di_acc += fabsf(id - p->prev_i); p->di_cnt++; }
        p->prev_i = id; p->inj_sign = -p->inj_sign;
        if (++p->tick >= n_ind) {
            p->Ld = pid_calc_L(p->v_inj, p->dt, p->di_acc / p->di_cnt);
            enter(p, PID_LQ);
            p->prev_i = 0.0f;
        }
        break;

    case PID_LQ:
        /* d 轴 DC 偏置锁住转子（PI 保持 i_align），q 轴方波注入测 β 轴纹波 */
        v_alpha = pid_pi(p, p->i_align, id);
        v_beta = (float)p->inj_sign * p->v_inj;
        if (p->tick >= n_settle) { p->di_acc += fabsf(iq - p->prev_i); p->di_cnt++; }
        p->prev_i = iq; p->inj_sign = -p->inj_sign;
        if (++p->tick >= n_ind) {
            p->Lq = pid_calc_L(p->v_inj, p->dt, p->di_acc / p->di_cnt);
            p->ok = (p->Rs > 0.0f && p->Ld > 0.0f && p->Lq > 0.0f);
            enter(p, p->ok ? PID_DONE : PID_FAIL);
        }
        break;

    case PID_DONE:
    case PID_FAIL:
    case PID_IDLE:
    default:
        duty[0] = duty[1] = duty[2] = 0.5f;   /* 零矢量，停止注入 */
        return p->phase;
    }

    foc_svpwm(v_alpha, v_beta, p->v_dc, duty);
    return p->phase;
}

void param_id_apply(const param_id_t *p, motor_params_t *m) {
    if (!p->ok) return;
    m->Rs = p->Rs; m->Ld = p->Ld; m->Lq = p->Lq;
    /* ψ、p 保持：ψ 由旋转段估，p 由用户配置 */
}
