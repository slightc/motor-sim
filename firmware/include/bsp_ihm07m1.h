/* -*- coding: utf-8 -*-
 * bsp_ihm07m1.h —— X-NUCLEO-IHM07M1 + NUCLEO-F302R8 板级支持
 *
 * 把"一块真实硬件"的全部事实集中到一处（对应仿真 hardware/ihm07m1.py 的
 * HardwareProfile）：引脚映射、功率级电压、电流链增益、ADC 量化。换板子只改本文件。
 *
 * ⚠ 上电前必须核对：引脚映射依据 ST UM1943 + MCSDK 标准分配 + 社区资料，
 *   数字引脚（PWM/EN）已多方确认，模拟引脚（电流/母线 ADC 通道）按 MCSDK 标准
 *   分配填写，**务必对照你手上板子的 UM1943 原理图 / CubeMX 工程确认**后再驱动电机。
 *   见 firmware/README.md 的"安全清单"。
 */
#ifndef BSP_IHM07M1_H
#define BSP_IHM07M1_H

#include "stm32f3xx_hal.h"
#include "foc.h"

/* ============================================================================
 * 1) 功率级 / 电气常数（对齐仿真 hardware/ihm07m1.py 的 PowerStageConfig 等）
 * ========================================================================== */
#define BSP_V_DC            24.0f       /* 母线电压 (V)。InverterLimits(24,..) */
#define BSP_I_MAX           2.5f        /* 控制电流上限 (A)。L6230 2.8A 峰值卡限 */
#define BSP_F_PWM           20000.0f    /* PWM 载波频率 (Hz)。也是电流环频率 */
#define BSP_DEAD_TIME_NS    1000.0f     /* 死区 (ns) = 1µs，对应仿真 dead_time=1e-6 */

/* 电流检测链：R_shunt=0.33Ω, 运放增益 1.53(反相) → 0.505 V/A；
 * STM32 内部 12-bit ADC, ±3.27A 量程 → LSB≈1.6mA（对应 CurrentSensorConfig）。 */
#define BSP_SHUNT_OHM       0.33f
#define BSP_AMP_GAIN        1.53f       /* 反相放大；BSP_CURR_SIGN 处理符号 */
#define BSP_CURR_SIGN       (-1.0f)     /* 运放反相：电流增大→ADC 电压减小 */
#define BSP_ADC_BITS        12
#define BSP_ADC_FULL        4096.0f     /* 2^12 */
#define BSP_ADC_VREF        3.3f        /* ADC 参考电压 (V) */
/* 满量程电流：i_range = (Vref/2) / (R_shunt*gain) = 1.65/0.505 ≈ 3.27A（与仿真一致）*/
#define BSP_I_RANGE         3.27f
/* ADC 计数 → 电流 (A)：i = sign * (count - offset) * (2*i_range / 2^bits) */
#define BSP_ADC_TO_AMP      (BSP_CURR_SIGN * (2.0f * BSP_I_RANGE) / BSP_ADC_FULL)

/* 默认占位电机（对应 hardware/motor 的 small_pmsm，真机标定后回填）*/
#define BSP_MOTOR_RS        0.5f
#define BSP_MOTOR_LD        4.0e-3f
#define BSP_MOTOR_LQ        6.0e-3f
#define BSP_MOTOR_PSI       0.03f
#define BSP_MOTOR_POLEPAIRS 4

/* ============================================================================
 * 2) 引脚映射（NUCLEO-F302R8 ↔ X-NUCLEO-IHM07M1，Arduino/Morpho 连接器）
 * ----------------------------------------------------------------------------
 *  已确认（ST UM1943 + 社区）：
 *    PWM 高边 → L6230 IN1/IN2/IN3 : PA8/PA9/PA10  = TIM1_CH1/CH2/CH3 (AF6)
 *    使能     → L6230 EN1/EN2/EN3 : PC10/PC11/PC12 = GPIO 推挽输出
 *  MCSDK 标准模拟分配（务必对照本板 UM1943 确认）：
 *    相电流 op-amp 输出 → ADC1: PA0(A0)=IN1, PC1(A4)=IN7, PB0(A3)=IN11
 *    母线电压           → ADC1: PA1? / 电位器/温度等见原理图（基础 FOC 不必需）
 * ========================================================================== */

/* --- PWM：TIM1 CH1/CH2/CH3 --- */
#define BSP_PWM_TIM             TIM1
#define BSP_PWM_GPIO_PORT       GPIOA
#define BSP_PWM_U_PIN           GPIO_PIN_8    /* PA8  TIM1_CH1 */
#define BSP_PWM_V_PIN           GPIO_PIN_9    /* PA9  TIM1_CH2 */
#define BSP_PWM_W_PIN           GPIO_PIN_10   /* PA10 TIM1_CH3 */
#define BSP_PWM_GPIO_AF         GPIO_AF6_TIM1

/* --- EN：GPIO 输出（高=使能该半桥）--- */
#define BSP_EN_GPIO_PORT        GPIOC
#define BSP_EN_U_PIN            GPIO_PIN_10   /* PC10 EN1 */
#define BSP_EN_V_PIN            GPIO_PIN_11   /* PC11 EN2 */
#define BSP_EN_W_PIN            GPIO_PIN_12   /* PC12 EN3 */

/* --- 相电流 ADC 通道（ADC1，注入组三通道，TIM1 触发中点采样）--- */
#define BSP_ADC                 ADC1
#define BSP_ADC_CH_IU           ADC_CHANNEL_1   /* PA0 */
#define BSP_ADC_CH_IV           ADC_CHANNEL_7   /* PC1 */
#define BSP_ADC_CH_IW           ADC_CHANNEL_11  /* PB0 */

/* TIM1 定时：中心对齐，ARR = f_clk / (2 * f_pwm)。72MHz/(2*20k)=1800 */
#define BSP_TIM1_CLK_HZ         72000000.0f
#define BSP_TIM1_ARR            ((uint32_t)(BSP_TIM1_CLK_HZ / (2.0f * BSP_F_PWM)))
/* 死区寄存器值（DTG）：dead_time * f_clk，简化线性区取值（DTG<128）。*/
#define BSP_TIM1_DTG            ((uint32_t)(BSP_DEAD_TIME_NS * 1e-9f * BSP_TIM1_CLK_HZ))

/* 控制周期 dt (s) = 1/f_pwm */
#define BSP_DT                  (1.0f / BSP_F_PWM)

/* ============================================================================
 * 3) BSP API
 * ========================================================================== */
void  bsp_clock_init(void);          /* 系统时钟 72MHz（HSE bypass 来自 ST-Link）*/
void  bsp_gpio_init(void);           /* PWM/EN GPIO 复用与输出 */
void  bsp_pwm_init(void);            /* TIM1 中心对齐 PWM + TRGO 触发 ADC */
void  bsp_adc_init(void);            /* ADC1 注入组三相电流，JEOC 中断 */
void  bsp_encoder_init(void);        /* 可选：编码器接口（闭环用）*/

void  bsp_drive_enable(int on);      /* 三相 EN 一起开/关（急停=关）*/
void  bsp_pwm_set_duty(const float duty[3]);   /* 写 TIM1 CCR1/2/3（0..1）*/
void  bsp_read_phase_currents(float *iu, float *iv, float *iw); /* 注入 ADC → 电流 */
void  bsp_current_offset_calib(void);          /* 上电零电流校准 ADC 偏置 */

float bsp_encoder_theta_e(void);     /* 电角度 (rad)，闭环用；开环 I/f 不调用 */
float bsp_encoder_omega_m(void);     /* 机械角速度 (rad/s) */
void  bsp_encoder_tick(float dt);    /* M 法测速更新（速度环节拍调用）*/

void  bsp_adc_store(uint16_t u, uint16_t v, uint16_t w); /* IT 回调缓存原始计数 */
ADC_HandleTypeDef *bsp_adc_handle(void);                 /* 供 IT 取 ADC 句柄 */

/* 把 BSP 电气常数填进 FOC 电机参数结构 */
static inline void bsp_fill_motor(motor_params_t *m) {
    m->Rs = BSP_MOTOR_RS; m->Ld = BSP_MOTOR_LD; m->Lq = BSP_MOTOR_LQ;
    m->psi = BSP_MOTOR_PSI; m->pole_pairs = BSP_MOTOR_POLEPAIRS;
}

#endif /* BSP_IHM07M1_H */
