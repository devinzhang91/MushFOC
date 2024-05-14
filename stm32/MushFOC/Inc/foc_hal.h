/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-06-28 13:34:37
 * @LastEditors: Rick
 * @LastEditTime: 2023-06-29 18:26:02
 * @Description:
 */
#ifndef __FOC_HAL_H__
#define __FOC_HAL_H__

#include "foc.h"
#include <stdint.h>
#include "stm32g0xx_hal.h"

void FOC_HAL_InitA(	FOC_T *hfoc,
					TIM_HandleTypeDef *htim_u,
					uint32_t channel_u,
					TIM_HandleTypeDef *htim_v,
					uint32_t channel_v,
					TIM_HandleTypeDef *htim_w,
					uint32_t channel_w,
					I2C_HandleTypeDef *h2ic,
					ADC_HandleTypeDef *hadc);

#endif
