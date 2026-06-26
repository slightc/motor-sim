/* -*- coding: utf-8 -*-
 * foc.h —— 基础磁场定向控制（FOC）算法核心
 *
 * 这是仿真 core/motorsim_core.py 的固件移植：Clarke/Park 变换、d/q 电流 PI 双环、
 * 速度 PI 外环、SVPWM（零序注入）—— 全部按位对应仿真里的 FieldWeakeningFOC
 * 与 SVPWMInverter，只是把"理想电压直接施加"换成"占空比写 TIM1 CCR"。
 *
 * 设计约束（与项目信条一致）：
 *   - 本文件**纯算法、零硬件依赖**（只用 <math.h>），可在 PC 上原生编译跑回归，
 *     与 Python 仿真逐点对齐（见 firmware/test/）。
 *   - 硬件相关（ADC 读数→电流、占空比→寄存器、角度来源）全部在 bsp / main 里，
 *     算法只吃"测量电流 + 电角度 + 设定值"，吐"三相占空比 0..1"。
 *
 * 信号链（对应仿真 Controller→Inverter）：
 *   (i_a,i_b,i_c, theta_e) → Clarke → Park → PI(d), PI(q) → inv_Park → SVPWM → duty[3]
 */
#ifndef FOC_H
#define FOC_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 电机/限幅参数（对齐 hardware/ihm07m1.py 占位电机，真机标定回填）---- */
typedef struct {
    float Rs;       /* 相电阻 (Ω)   —— 仿真 small_pmsm R0=0.5 */
    float Ld;       /* d 轴电感 (H) —— 4e-3 */
    float Lq;       /* q 轴电感 (H) —— 6e-3 */
    float psi;      /* 永磁磁链 (Wb)—— 0.03 */
    int   pole_pairs;/* 极对数 p    —— 4 */
} motor_params_t;

/* ---------------- 控制器状态（对应仿真 FieldWeakeningFOC 的成员）------------------- */
typedef struct {
    motor_params_t m;

    /* 电流环 PI（对齐仿真 kp_i=12, ki_i=3000）*/
    float kp_i, ki_i;
    /* 速度环 PI（对齐仿真 kp_w=0.6, ki_w=10）*/
    float kp_w, ki_w;

    /* 限幅：v_max=v_dc/sqrt(3)（对应 InverterLimits.v_max），i_max 峰值电流 */
    float v_dc;     /* 母线电压 (V)，SVPWM 占空比归一化用 */
    float v_max;    /* d/q 电压矢量幅值上限 (V) */
    float i_max;    /* 电流限幅 (A) */

    /* 积分器状态 */
    float iid, iiq; /* 电流环 d/q 积分 */
    float iw;       /* 速度环积分 */

    /* 上一拍参考（调试/弱磁预留）*/
    float id_ref, iq_ref;

    /* 输出探针（调试用，对应仿真各中间量）*/
    float i_d, i_q;         /* 测量 dq 电流 */
    float v_d, v_q;         /* 电流环输出 dq 电压 */
    float duty[3];          /* 三相占空比 0..1 */
} foc_t;

/* 用电机参数 + 母线电压 + 电流上限初始化控制器，PI 增益取仿真默认值。 */
void foc_init(foc_t *f, const motor_params_t *m, float v_dc, float i_max);

/* 复位所有积分器与参考（停机/重启动用）。 */
void foc_reset(foc_t *f);

/* 电流内环：给定 d/q 参考电流 + 测量三相电流 + 电角度，算出三相占空比 duty[3]。
 * 对应仿真 FieldWeakeningFOC.compute 的电流环部分（不含速度外环）。
 * dt: 控制周期 (s) = 1/f_pwm。 */
void foc_current_step(foc_t *f, float id_ref, float iq_ref,
                      float ia, float ib, float ic, float theta_e, float dt);

/* 速度外环：给定速度设定 + 测量机械速度，返回 q 轴电流参考 iq_ref（带限幅+抗饱和）。
 * 对应仿真 FieldWeakeningFOC.compute 的速度环部分。基础 FOC 用 id_ref=0。 */
float foc_speed_step(foc_t *f, float omega_ref, float omega_meas, float dt);

/* ---------------- 纯数学工具（与仿真 clarke/park 同式，导出供测试/BSP 复用）-------- */
void  foc_clarke(float a, float b, float c, float *alpha, float *beta);
void  foc_inv_clarke(float alpha, float beta, float *a, float *b, float *c);
void  foc_park(float alpha, float beta, float th, float *d, float *q);
void  foc_inv_park(float d, float q, float th, float *alpha, float *beta);

/* SVPWM：αβ 参考电压 → 三相占空比 0..1（零序注入，对应仿真 SVPWMInverter）。
 * v_dc 为母线电压。占空比已钳位到 [0,1]。 */
void  foc_svpwm(float v_alpha, float v_beta, float v_dc, float duty[3]);

#ifdef __cplusplus
}
#endif

#endif /* FOC_H */
