/* -*- coding: utf-8 -*-
 * main.c —— IHM07M1 + F302R8 基础 FOC 主程序
 *
 * 编排对应仿真 Simulator.run 的单拍循环，但实时版本：控制律在 ADC 注入完成中断
 * （PWM 中点）里执行，主循环只做慢速调度（测速、模式切换、安全监控）。
 *
 * 两种模式（docs/05 §2.3 的上电顺序：先开环确认相序/极对/方向，再闭环）：
 *   MODE_OPENLOOP_IF —— 开环 I/f：强制旋转角 + 恒定 iq，安全首转（推荐第一次上电）。
 *                       对应 control/09_if_opencontrol.py 的思想（此处简化无主动阻尼）。
 *   MODE_SENSORED    —— 有感闭环 FOC：编码器角度 + 速度环，对应 control/01_foc_sensored.py。
 *
 * 切换：默认 I/f 启动并自动跑一段，确认无误后把 g_mode 改 MODE_SENSORED（或加按键/串口）。
 */
#include "bsp_ihm07m1.h"
#include "foc.h"
#include <math.h>

#define TWO_PI 6.283185307179586f

typedef enum { MODE_OPENLOOP_IF = 0, MODE_SENSORED = 1 } run_mode_t;

/* ---- 运行时配置（可由串口/调试器改）---- */
volatile run_mode_t g_mode      = MODE_OPENLOOP_IF;
volatile float      g_speed_ref = 30.0f;   /* 闭环目标机械速度 (rad/s)，对齐 demo 01 */
volatile int        g_running   = 0;       /* 1=允许输出，0=封锁（急停）*/

/* ---- I/f 参数 ---- */
#define IF_I_CMD      0.8f      /* I/f 注入 q 轴电流 (A)，< i_max */
#define IF_OMEGA_E    60.0f     /* I/f 稳态电角速度 (rad/s) = p*ω_m */
#define IF_RAMP       40.0f     /* 电角加速度 (rad/s²) */

static foc_t  g_foc;
static float  g_if_theta = 0.0f;   /* I/f 开环角积分 */
static float  g_if_omega = 0.0f;   /* I/f 当前电角速度（斜坡）*/

/* ============================================================================
 * 电流环 ISR：ADC 注入完成（PWM 中点）回调里被调用，每 PWM 周期一次。
 * 这是仿真 Controller.compute 的实时对应（docs/05 §2.1 移植映射）。
 * ========================================================================== */
void foc_control_isr(void) {
    float iu, iv, iw;
    bsp_read_phase_currents(&iu, &iv, &iw);

    if (!g_running) {                 /* 封锁：写零矢量，不积分 */
        float z[3] = {0.5f, 0.5f, 0.5f};
        bsp_pwm_set_duty(z);
        return;
    }

    float theta_e, id_ref, iq_ref;
    if (g_mode == MODE_OPENLOOP_IF) {
        /* 开环 I/f：角度斜坡，恒定 iq 注入，id=0 */
        if (g_if_omega < IF_OMEGA_E) g_if_omega += IF_RAMP * BSP_DT;
        g_if_theta += g_if_omega * BSP_DT;
        if (g_if_theta >= TWO_PI) g_if_theta -= TWO_PI;
        theta_e = g_if_theta;
        id_ref = 0.0f;
        iq_ref = IF_I_CMD;
    } else {
        /* 有感闭环：编码器电角度 + 速度外环给 iq，基础 FOC id=0 */
        theta_e = bsp_encoder_theta_e();
        id_ref = 0.0f;
        iq_ref = g_foc.iq_ref;        /* 由主循环速度环更新（带宽分离）*/
    }

    foc_current_step(&g_foc, id_ref, iq_ref, iu, iv, iw, theta_e, BSP_DT);
    bsp_pwm_set_duty(g_foc.duty);
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

    /* 上电零电流校准（EN 关断下做）*/
    bsp_current_offset_calib();

    /* 起动：使能功率级，开始输出 */
    foc_reset(&g_foc);
    g_if_theta = 0.0f; g_if_omega = 0.0f;
    bsp_drive_enable(1);
    g_running = 1;

    /* 速度环节拍：电流环 ≫ 速度环（带宽分离，docs 关键经验）。
     * 这里每 ~1ms（每 PWM_F/1000 拍）跑一次速度环与测速。 */
    const float SPEED_DT = 1.0e-3f;
    uint32_t tick = 0;
    while (1) {
        HAL_Delay(1);                 /* 1ms 调度 */
        if (g_mode == MODE_SENSORED && g_running) {
            bsp_encoder_tick(SPEED_DT);
            float wm = bsp_encoder_omega_m();
            g_foc.iq_ref = foc_speed_step(&g_foc, g_speed_ref, wm, SPEED_DT);
        }
        /* I/f 跑满 ~2s 后可在此自动切闭环（默认保守：保持 I/f，由人工确认后改 g_mode）*/
        tick++;
        (void)tick;
    }
}

/* HAL 时基所需（SysTick）*/
void SysTick_Handler(void) { HAL_IncTick(); }
