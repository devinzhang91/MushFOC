/*
 * drv8313.c
 *
 *  Created on: Apr 25, 2024
 *      Author: devin
 */


#include "drv8313.h"

#include "log.h"

int DRV8313_Init(	DRV8313_T *a,
					TIM_HandleTypeDef *htim_u,
					uint32_t channel_u,
					TIM_HandleTypeDef *htim_v,
					uint32_t channel_v,
					TIM_HandleTypeDef *htim_w,
					uint32_t channel_w) {
  a->tim_u_ins = htim_u;
  a->channel_u = channel_u;
  a->tim_v_ins = htim_v;
  a->channel_v = channel_v;
  a->tim_w_ins = htim_w;
  a->channel_w = channel_w;

  if (HAL_TIM_PWM_Start(a->tim_u_ins, a->channel_u) != HAL_OK) return -1;
  if (HAL_TIM_PWM_Start(a->tim_v_ins, a->channel_v) != HAL_OK) return -1;
  if (HAL_TIM_PWM_Start(a->tim_w_ins, a->channel_w) != HAL_OK) return -1;

  return 0;
}

void DRV8313_SetPWM(DRV8313_T *a,
					uint32_t duty_u,
					uint32_t duty_v,
					uint32_t duty_w){

	__HAL_TIM_SetCompare(a->tim_u_ins, a->channel_u, duty_u);    //修改比较值，修改占空比
	__HAL_TIM_SetCompare(a->tim_v_ins, a->channel_v, duty_v);
	__HAL_TIM_SetCompare(a->tim_w_ins, a->channel_w, duty_w);
}


