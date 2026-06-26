/* -*- coding: utf-8 -*-
 * param_id.h —— 电机参数自整定（自动测量 Rs / Ld / Lq）
 *
 * 对应 docs/05_hardware_deployment.md §3.2 的「参数→标定实验映射」，把离线标定搬到
 * 上电自整定，**电机静止**即可测出无感 FOC 所需的电气参数：
 *   Rs  : d 轴两级 DC 注入，Rs = ΔV/ΔI（差分抵消逆变器死区/管压降偏置）
 *   Ld  : d 轴方波高频电压注入，测电流纹波，Ld = V_inj·dt/ΔI
 *   Lq  : q 轴方波高频注入（DC d 偏置锁住转子），Lq = V_inj·dt/ΔI
 * 反电动势观测器只需 R/L（PLL 按 |e| 归一化，ψ 被抵消），故静止自整定即足以跑无感。
 * ψ 在无感 I/f 旋转段顺带估计并上报（见 main.c）。
 *
 * 设计：状态机每个电流环周期推进一拍（在 ADC 注入中断里调用 pid_step），全程 θ_e=0
 * 把 dq 坍缩成 αβ（d=α, q=β），数学简洁。纯公式（pid_calc_*）可主机回归。
 */
#ifndef PARAM_ID_H
#define PARAM_ID_H

#include "foc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PID_IDLE = 0,
    PID_ALIGN,      /* 拉 d 电流，转子对齐到 θ=0 */
    PID_RS_LOW,     /* Rs 低电流点 */
    PID_RS_HIGH,    /* Rs 高电流点 */
    PID_LD,         /* d 轴方波注入测 Ld */
    PID_LQ,         /* q 轴方波注入测 Lq */
    PID_DONE,
    PID_FAIL
} pid_phase_t;

typedef struct {
    /* 配置 */
    float v_dc, dt, v_max;
    float i_align;      /* 对齐/Lq 保持用的 d 轴 DC 电流 (A) */
    float i_rs_low, i_rs_high;  /* Rs 两级电流 (A) */
    float v_inj;        /* 高频注入电压幅值 (V) */
    /* 简易 d 轴 DC 电流 PI（自整定自带，不依赖 foc_t）*/
    float kp, ki, integ;

    /* 运行 */
    pid_phase_t phase;
    int   tick;         /* 当前相位已运行拍数 */
    int   inj_sign;     /* 方波注入符号 */
    float prev_i;       /* 上一拍轴电流（算纹波）*/
    /* 累加器 */
    float v_acc; int v_cnt;       /* 稳态电压平均 */
    float i_acc;                  /* 稳态电流平均 */
    float di_acc; int di_cnt;     /* |Δi| 平均（电感）*/
    float v1, i1;                 /* Rs 低点 */

    /* 结果 */
    float Rs, Ld, Lq;
    int   ok;
} param_id_t;

/* 初始化自整定。v_max=v_dc/sqrt3。各电流/电压用保守默认（可改字段后再跑）。*/
void  param_id_init(param_id_t *p, float v_dc, float i_max, float dt);

/* 推进一拍：输入三相测量电流，输出本拍三相占空比 duty[3]。每电流环周期调用。
 * 返回当前相位（PID_DONE/PID_FAIL 表示结束）。*/
pid_phase_t param_id_step(param_id_t *p, float ia, float ib, float ic, float duty[3]);

/* 把整定结果写入 FOC 电机参数（Ld/Lq/Rs），ψ/p 保持原值（ψ 由旋转段估）。*/
void  param_id_apply(const param_id_t *p, motor_params_t *m);

/* ---- 纯公式（可独立回归）---- */
float pid_calc_rs(float dV, float dI);                 /* Rs = ΔV/ΔI */
float pid_calc_L(float v_inj, float dt, float mean_abs_di); /* L = V·dt/Δi */
/* 旋转段估磁链：ψ = (v_q − R·i_q) / ω_e */
float pid_calc_psi(float v_q, float Rs, float i_q, float omega_e);

#ifdef __cplusplus
}
#endif

#endif /* PARAM_ID_H */
