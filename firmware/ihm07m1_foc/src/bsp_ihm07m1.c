/* -*- coding: utf-8 -*-
 * bsp_ihm07m1.c —— 板级支持实现（STM32F302R8 HAL）
 *
 * 把仿真里的"理想逆变器/传感器"换成真实外设：
 *   仿真 SVPWMInverter   → TIM1 中心对齐 PWM（PA8/9/10）+ L6230 EN（PC10/11/12）
 *   仿真 CurrentSensor   → ADC1 注入组三相电流（中点同步采样，docs/05 §2.2）
 *   仿真 Encoder         → TIM2 编码器接口（闭环可选）
 *
 * 注：本文件依赖 STM32Cube HAL（PlatformIO framework=stm32cube）。无 HAL 环境时
 *     algorithm 层 foc.c 仍可独立编译/回归（见 firmware/test/）。
 */
#include "bsp_ihm07m1.h"
#include <math.h>

static TIM_HandleTypeDef htim1;     /* PWM */
static TIM_HandleTypeDef htim2;     /* 编码器 */
static ADC_HandleTypeDef hadc1;     /* 相电流 */

/* 上电零电流时的 ADC 偏置（计数），由 bsp_current_offset_calib 标定 */
static float s_off_u = BSP_ADC_FULL * 0.5f;
static float s_off_v = BSP_ADC_FULL * 0.5f;
static float s_off_w = BSP_ADC_FULL * 0.5f;

static volatile uint16_t s_adc_u, s_adc_v, s_adc_w;  /* 最近一次注入采样 */

/* ============================================================================
 * 时钟：HSE bypass 8MHz（NUCLEO 由 ST-LINK MCO 提供）→ PLL ×9 = 72MHz
 * ========================================================================== */
void bsp_clock_init(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_BYPASS;            /* ST-LINK 8MHz 方波，bypass 模式 */
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1; /* PLL 输入 = HSE/1 = 8MHz */
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9;            /* 8MHz ×9 = 72MHz */
    HAL_RCC_OscConfig(&osc);

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;      /* HCLK 72MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV2;       /* APB1 36MHz（TIM2 ×2=72MHz）*/
    clk.APB2CLKDivider = RCC_HCLK_DIV1;       /* APB2 72MHz（TIM1）*/
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

/* ============================================================================
 * GPIO：PWM 复用 AF6，EN 推挽输出（初始关断）
 * ========================================================================== */
void bsp_gpio_init(void) {
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PWM：PA8/9/10 复用推挽 */
    g.Pin = BSP_PWM_U_PIN | BSP_PWM_V_PIN | BSP_PWM_W_PIN;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = BSP_PWM_GPIO_AF;
    HAL_GPIO_Init(BSP_PWM_GPIO_PORT, &g);

    /* EN：PC10/11/12 输出，默认拉低（关断功率级，安全）*/
    g.Pin = BSP_EN_U_PIN | BSP_EN_V_PIN | BSP_EN_W_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Alternate = 0;
    HAL_GPIO_Init(BSP_EN_GPIO_PORT, &g);
    HAL_GPIO_WritePin(BSP_EN_GPIO_PORT,
        BSP_EN_U_PIN | BSP_EN_V_PIN | BSP_EN_W_PIN, GPIO_PIN_RESET);
}

/* ============================================================================
 * TIM1：中心对齐 PWM，3 通道；更新事件 TRGO 触发 ADC（PWM 中点采样）
 * ========================================================================== */
void bsp_pwm_init(void) {
    TIM_ClockConfigTypeDef sclk = {0};
    TIM_MasterConfigTypeDef mst = {0};
    TIM_OC_InitTypeDef oc = {0};
    TIM_BreakDeadTimeConfigTypeDef bdt = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = BSP_PWM_TIM;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period = BSP_TIM1_ARR;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);

    sclk.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim1, &sclk);

    /* 更新事件作为 TRGO → 触发 ADC 注入组（中心对齐计数到顶=PWM 中点，电流纹波最小）*/
    mst.MasterOutputTrigger = TIM_TRGO_UPDATE;
    mst.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &mst);

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = BSP_TIM1_ARR / 2;              /* 初始 50% = 零矢量 */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3);

    /* 死区 + 刹车（IHM07M1 用 IN/EN 单边驱动，仅高边 PWM；死区为保护性配置）*/
    bdt.OffStateRunMode = TIM_OSSR_DISABLE;
    bdt.OffStateIDLEMode = TIM_OSSI_DISABLE;
    bdt.LockLevel = TIM_LOCKLEVEL_OFF;
    bdt.DeadTime = BSP_TIM1_DTG;
    bdt.BreakState = TIM_BREAK_DISABLE;
    bdt.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    bdt.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdt);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_MOE_ENABLE(&htim1);
}

/* ============================================================================
 * ADC1：注入组三相电流，外部触发 = TIM1 TRGO，JEOC 中断里跑 FOC
 * ========================================================================== */
void bsp_adc_init(void) {
    ADC_InjectionConfTypeDef inj = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = BSP_ADC;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;  /* 72/4=18MHz ADC 时钟 */
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;  /* 规则组不用 */
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    HAL_ADC_Init(&hadc1);

    /* 注入组：触发源 = TIM1 TRGO（中点采样）；扫描 3 通道 */
    inj.InjectedSamplingTime = ADC_SAMPLETIME_19CYCLES_5;
    inj.InjectedSingleDiff = ADC_SINGLE_ENDED;
    inj.InjectedOffsetNumber = ADC_OFFSET_NONE;
    inj.InjectedOffset = 0;
    inj.InjectedNbrOfConversion = 3;
    inj.InjectedDiscontinuousConvMode = DISABLE;
    inj.AutoInjectedConv = DISABLE;
    inj.ExternalTrigInjecConv = ADC1_EXTERNALTRIGINJEC_T1_TRGO;
    inj.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;

    inj.InjectedChannel = BSP_ADC_CH_IU; inj.InjectedRank = ADC_INJECTED_RANK_1;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &inj);
    inj.InjectedChannel = BSP_ADC_CH_IV; inj.InjectedRank = ADC_INJECTED_RANK_2;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &inj);
    inj.InjectedChannel = BSP_ADC_CH_IW; inj.InjectedRank = ADC_INJECTED_RANK_3;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &inj);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);  /* 电流环最高优先级 */
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/* ============================================================================
 * 编码器（可选，闭环用）：TIM2 编码器模式
 *   ⚠ IHM07M1 编码器/霍尔输入引脚随接线而定，需对照 UM1943 / 你的接线确认 AF。
 *     默认禁用（开环 I/f 不需要）。闭环前请填好引脚与每转线数。
 * ========================================================================== */
#ifndef BSP_ENCODER_PPR
#define BSP_ENCODER_PPR  2500          /* 编码器线数（对齐仿真 Encoder(2500)）*/
#endif
static volatile int32_t s_enc_last = 0;
static volatile float   s_omega_m = 0.0f;

void bsp_encoder_init(void) {
    TIM_Encoder_InitTypeDef enc = {0};
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;            /* 32-bit 计数 */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    enc.EncoderMode = TIM_ENCODERMODE_TI12;    /* 4 倍频 */
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter = 4;
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter = 4;
    HAL_TIM_Encoder_Init(&htim2, &enc);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

/* 机械角 → 电角度：theta_e = (p * theta_m) mod 2π */
float bsp_encoder_theta_e(void) {
    int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    float counts_per_rev = 4.0f * (float)BSP_ENCODER_PPR;
    float theta_m = (2.0f * 3.14159265358979f) * ((float)cnt / counts_per_rev);
    float theta_e = (float)BSP_MOTOR_POLEPAIRS * theta_m;
    theta_e = fmodf(theta_e, 2.0f * 3.14159265358979f);
    if (theta_e < 0) theta_e += 2.0f * 3.14159265358979f;
    return theta_e;
}

/* 速度：M 法（固定窗口位置差分）。需周期性调用 bsp_encoder_tick(dt) 更新。 */
void bsp_encoder_tick(float dt) {
    int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    float counts_per_rev = 4.0f * (float)BSP_ENCODER_PPR;
    float dtheta = (2.0f * 3.14159265358979f) * ((float)(cnt - s_enc_last) / counts_per_rev);
    s_enc_last = cnt;
    s_omega_m = dtheta / dt;
}
float bsp_encoder_omega_m(void) { return s_omega_m; }

/* ============================================================================
 * 功率级开关 / 占空比 / 电流读取
 * ========================================================================== */
void bsp_drive_enable(int on) {
    GPIO_PinState st = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(BSP_EN_GPIO_PORT, BSP_EN_U_PIN, st);
    HAL_GPIO_WritePin(BSP_EN_GPIO_PORT, BSP_EN_V_PIN, st);
    HAL_GPIO_WritePin(BSP_EN_GPIO_PORT, BSP_EN_W_PIN, st);
}

void bsp_pwm_set_duty(const float duty[3]) {
    uint32_t arr = BSP_TIM1_ARR;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(duty[0] * arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(duty[1] * arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(duty[2] * arr));
}

/* 注入组转换完成回调里缓存原始计数（见 stm32f3xx_it.c → HAL_ADCEx_InjectedConvCpltCallback）*/
void bsp_adc_store(uint16_t u, uint16_t v, uint16_t w) {
    s_adc_u = u; s_adc_v = v; s_adc_w = w;
}

void bsp_read_phase_currents(float *iu, float *iv, float *iw) {
    *iu = ((float)s_adc_u - s_off_u) * BSP_ADC_TO_AMP;
    *iv = ((float)s_adc_v - s_off_v) * BSP_ADC_TO_AMP;
    *iw = ((float)s_adc_w - s_off_w) * BSP_ADC_TO_AMP;
}

/* 上电零电流校准：EN 关断（电机不通电）下平均若干次注入采样作为偏置。*/
void bsp_current_offset_calib(void) {
    const int N = 256;
    float su = 0, sv = 0, sw = 0;
    bsp_drive_enable(0);
    for (int i = 0; i < N; ++i) {
        HAL_ADCEx_InjectedStart(&hadc1);
        HAL_ADCEx_InjectedPollForConversion(&hadc1, 5);
        su += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        sv += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        sw += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
    }
    s_off_u = su / N; s_off_v = sv / N; s_off_w = sw / N;
}

/* 供 IT 文件取用的 ADC 句柄 */
ADC_HandleTypeDef *bsp_adc_handle(void) { return &hadc1; }
