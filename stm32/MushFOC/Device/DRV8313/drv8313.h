/*
 * drv8313.h
 *
 *  Created on: Apr 25, 2024
 *      Author: devin
 */

#ifndef _DRV8313_H_
#define _DRV8313_H_
#include <stdint.h>
#include "stm32g0xx_hal.h"

typedef struct {
  TIM_HandleTypeDef *tim_u_ins;
  uint32_t channel_u;
  TIM_HandleTypeDef *tim_v_ins;
  uint32_t channel_v;
  TIM_HandleTypeDef *tim_w_ins;
  uint32_t channel_w;
} DRV8313_T;

int DRV8313_Init(	DRV8313_T *a,
					TIM_HandleTypeDef *htim_u,
					uint32_t channel_u,
					TIM_HandleTypeDef *htim_v,
					uint32_t channel_v,
					TIM_HandleTypeDef *htim_w,
					uint32_t channel_w);

void DRV8313_SetPWM(DRV8313_T *a,
					uint32_t duty_u,
					uint32_t duty_v,
					uint32_t duty_w);


#endif /* _DRV8313_H_ */
