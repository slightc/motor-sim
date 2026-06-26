/* -*- coding: utf-8 -*-
 * stm32f3xx_it.c —— 中断服务
 *
 * 电流环走 ADC 注入完成中断：TIM1 TRGO（PWM 中点）触发 ADC 采三相电流 → JEOC →
 * 在回调里把原始计数交给 BSP，并调用 foc_control_isr() 执行一拍 FOC。
 */
#include "bsp_ihm07m1.h"

extern void foc_control_isr(void);

/* ADC1/ADC2 共享中断向量 */
void ADC1_2_IRQHandler(void) {
    HAL_ADC_IRQHandler(bsp_adc_handle());
}

/* 注入组转换完成：读三相原始计数 → 缓存 → 跑电流环 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    uint16_t u = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    uint16_t v = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    uint16_t w = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
    bsp_adc_store(u, v, w);
    foc_control_isr();
}
