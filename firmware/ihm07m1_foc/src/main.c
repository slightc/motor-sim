/* -*- coding: utf-8 -*-
 * main.c —— IHM07M1 + F302R8 FOC 主程序（有感 / 无感 / 参数自整定）
 *
 * 编排对应仿真 Simulator.run 的单拍循环，实时版本：控制律在 ADC 注入完成中断
 * （PWM 中点）里执行，主循环做慢速调度（模式推进、安全监控）。
 *
 * 运行模式（g_mode）：
 *   MODE_PARAM_ID    —— 上电自整定：静止测 Rs/Ld/Lq（param_id.c，docs/05 §3.2）。
 *   MODE_OPENLOOP_IF —— 开环 I/f：强制角 + 恒定 iq，安全首转。
 *   MODE_SENSORED    —— 有感闭环 FOC：编码器（control/01）。
 *   MODE_SENSORLESS  —— 无感闭环 FOC：I/f 起转 → 反电动势观测器 PLL 交接（control/02）。
 *
 * 默认流程（本次需求「无感 FOC + 自动测量参数」）：
 *   PARAM_ID(自测 Rs/Ld/Lq) → 回填电机参数 → SENSORLESS(I/f 起转→无感闭环)。
 */
#include "bsp_ihm07m1.h"
#include "foc.h"
#include "param_id.h"
#include <math.h>

#define TWO_PI 6.283185307179586f

typedef enum {
    MODE_PARAM_ID = 0, MODE_OPENLOOP_IF, MODE_SENSORED, MODE_SENSORLESS
} run_mode_t;

/* ---- 运行时配置（可由调试器/串口改）---- */
volatile run_mode_t g_mode      = MODE_PARAM_ID;   /* 默认：先自整定 */
volatile float      g_speed_ref = 60.0f;   /* 目标机械速度 (rad/s)，无感需中高速 */
volatile int        g_running   = 0;       /* 1=允许输出，0=封锁（急停）*/

/* ---- I/f 起转参数 ---- */
#define IF_I_CMD        0.8f      /* I/f 注入 q 轴电流 (A)，< i_max */
#define IF_RAMP         40.0f     /* 电角加速度 (rad/s²) */
#define SL_HANDOFF_WE   80.0f     /* I/f→无感闭环交接电角速度 (rad/s)，反电动势够强 */

static foc_t        g_foc;
static param_id_t   g_pid;
static sensorless_t g_sl;

static float  g_if_theta = 0.0f;   /* I/f 开环角积分 */
static float  g_if_omega = 0.0f;   /* I/f 当前电角速度（斜坡）*/
static volatile int   g_sl_closed = 0;   /* 0=I/f 起转, 1=无感闭环 */
static volatile float g_psi_est = 0.0f;  /* 旋转段估计磁链（上报，观测器不需）*/
static volatile int   g_id_done = 0;     /* 自整定完成标志（主循环消费）*/

/* ============================================================================
 * 电流环 ISR：ADC 注入完成（PWM 中点）每 PWM 周期一次。
 * ========================================================================== */
void foc_control_isr(void) {
    float iu, iv, iw;
    bsp_read_phase_currents(&iu, &iv, &iw);
    float duty[3];

    if (!g_running) { duty[0]=duty[1]=duty[2]=0.5f; bsp_pwm_set_duty(duty); return; }

    switch (g_mode) {

    case MODE_PARAM_ID: {
        pid_phase_t ph = param_id_step(&g_pid, iu, iv, iw, duty);
        if (ph == PID_DONE || ph == PID_FAIL) g_id_done = 1;
        bsp_pwm_set_duty(duty);
        return;
    }

    case MODE_OPENLOOP_IF: {
        if (g_if_omega < SL_HANDOFF_WE) g_if_omega += IF_RAMP * BSP_DT;
        g_if_theta += g_if_omega * BSP_DT;
        if (g_if_theta >= TWO_PI) g_if_theta -= TWO_PI;
        foc_current_step(&g_foc, 0.0f, IF_I_CMD, iu, iv, iw, g_if_theta, BSP_DT);
        bsp_pwm_set_duty(g_foc.duty);
        return;
    }

    case MODE_SENSORED: {
        float theta_e = bsp_encoder_theta_e();
        foc_current_step(&g_foc, 0.0f, g_foc.iq_ref, iu, iv, iw, theta_e, BSP_DT);
        bsp_pwm_set_duty(g_foc.duty);
        return;
    }

    case MODE_SENSORLESS: {
        if (!g_sl_closed) {
            /* I/f 开环起转 + 旋转段估 ψ */
            if (g_if_omega < SL_HANDOFF_WE) g_if_omega += IF_RAMP * BSP_DT;
            g_if_theta += g_if_omega * BSP_DT;
            if (g_if_theta >= TWO_PI) g_if_theta -= TWO_PI;
            foc_current_step(&g_foc, 0.0f, IF_I_CMD, iu, iv, iw, g_if_theta, BSP_DT);
            if (g_if_omega >= 0.95f * SL_HANDOFF_WE)
                g_psi_est = pid_calc_psi(g_foc.v_q, g_foc.m.Rs, g_foc.i_q, g_if_omega);
            if (g_if_omega >= SL_HANDOFF_WE) {
                /* 交接：把观测器状态预置到 I/f 角/速，记录上一拍 αβ 电压 */
                bemf_obs_preset(&g_sl.obs, g_if_theta, g_if_omega);
                foc_inv_park(g_foc.v_d, g_foc.v_q, g_if_theta,
                             &g_sl.v_prev_a, &g_sl.v_prev_b);
                g_foc.iw = 0.0f;          /* 速度环积分清零，避免起跳 */
                g_sl_closed = 1;
            }
            bsp_pwm_set_duty(g_foc.duty);
        } else {
            sensorless_step(&g_sl, &g_foc, g_speed_ref, iu, iv, iw, BSP_DT);
            bsp_pwm_set_duty(g_foc.duty);
        }
        return;
    }
    }
}

/* 进入无感模式：用（自整定后的）电机参数装观测器、设无感增益、复位起转状态 */
static void enter_sensorless(void) {
    foc_set_sensorless_gains(&g_foc);
    sensorless_init(&g_sl, &g_foc, 2000.0f);   /* f_lp=2000，对齐仿真 SensorlessFOC */
    foc_reset(&g_foc);
    g_if_theta = 0.0f; g_if_omega = 0.0f; g_sl_closed = 0;
}

/* ============================================================================
 * 主程序
 * ========================================================================== */
int main(void) {
    HAL_Init();
    bsp_clock_init();
    bsp_gpio_init();
    bsp_pwm_init();
    bsp_adc_init();
    bsp_encoder_init();

    motor_params_t m;
    bsp_fill_motor(&m);
    foc_init(&g_foc, &m, BSP_V_DC, BSP_I_MAX);

    bsp_current_offset_calib();     /* 上电零电流校准（EN 关断下）*/

    /* 起动 */
    foc_reset(&g_foc);
    bsp_drive_enable(1);

    if (g_mode == MODE_PARAM_ID) {
        param_id_init(&g_pid, BSP_V_DC, BSP_I_MAX, BSP_DT);
        g_id_done = 0;
    } else if (g_mode == MODE_SENSORLESS) {
        enter_sensorless();
    }
    g_running = 1;

    const float SPEED_DT = 1.0e-3f;
    while (1) {
        HAL_Delay(1);

        /* 自整定完成 → 回填参数 → 转入无感闭环 */
        if (g_mode == MODE_PARAM_ID && g_id_done) {
            g_running = 0;                       /* 切换期间封锁输出 */
            if (g_pid.ok) param_id_apply(&g_pid, &g_foc.m);
            foc_init(&g_foc, &g_foc.m, BSP_V_DC, BSP_I_MAX);  /* 用新参数重算 v_max 等 */
            enter_sensorless();
            g_mode = MODE_SENSORLESS;
            g_running = 1;
        }

        /* 有感模式：1ms 跑速度环（带宽分离）。无感的速度环在 ISR 内（对齐仿真）。*/
        if (g_mode == MODE_SENSORED && g_running) {
            bsp_encoder_tick(SPEED_DT);
            float wm = bsp_encoder_omega_m();
            g_foc.iq_ref = foc_speed_step(&g_foc, g_speed_ref, wm, SPEED_DT);
        }
    }
}

/* HAL 时基（SysTick）*/
void SysTick_Handler(void) { HAL_IncTick(); }
