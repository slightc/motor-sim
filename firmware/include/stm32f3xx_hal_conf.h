/* -*- coding: utf-8 -*-
 * stm32f3xx_hal_conf.h —— HAL 模块裁剪配置（仅启用基础 FOC 所需模块）
 * 覆盖 framework 默认配置：RCC/GPIO/TIM/ADC/CORTEX/FLASH/PWR/DMA。
 */
#ifndef STM32F3xx_HAL_CONF_H
#define STM32F3xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 启用的 HAL 模块 ---- */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED

/* ---- 振荡器/时钟常数（NUCLEO-F302R8）---- */
#if !defined(HSE_VALUE)
#define HSE_VALUE          8000000U   /* ST-LINK MCO 提供 8MHz（bypass）*/
#endif
#define HSE_STARTUP_TIMEOUT 100U
#if !defined(HSI_VALUE)
#define HSI_VALUE          8000000U
#endif
#define HSI_STARTUP_TIMEOUT 5000U
#if !defined(LSI_VALUE)
#define LSI_VALUE          40000U
#endif
#if !defined(LSE_VALUE)
#define LSE_VALUE          32768U
#endif
#define LSE_STARTUP_TIMEOUT 5000U
#define VDD_VALUE          3300U
#define TICK_INT_PRIORITY  3U
#define USE_RTOS           0U
#define PREFETCH_ENABLE    1U
#define INSTRUCTION_CACHE_ENABLE 0U
#define DATA_CACHE_ENABLE  0U

#define USE_HAL_ADC_REGISTER_CALLBACKS 0U
#define USE_HAL_TIM_REGISTER_CALLBACKS 0U

/* ---- 断言（发布版关闭）---- */
/* #define USE_FULL_ASSERT 1U */

/* ---- 包含启用模块的头 ---- */
#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f3xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32f3xx_hal_gpio.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32f3xx_hal_dma.h"
#endif
#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32f3xx_hal_exti.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32f3xx_hal_cortex.h"
#endif
#ifdef HAL_ADC_MODULE_ENABLED
#include "stm32f3xx_hal_adc.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f3xx_hal_tim.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32f3xx_hal_flash.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32f3xx_hal_pwr.h"
#endif

#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32F3xx_HAL_CONF_H */
