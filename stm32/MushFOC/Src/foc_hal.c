/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-06-28 13:34:45
 * @LastEditors: Rick
 * @LastEditTime: 2023-06-29 18:25:48
 * @Description:
 */
#include "foc_hal.h"

#include <stm32g0xx_hal.h>

#include "AS5600/as5600.h"
#include "DRV8313/drv8313.h"
#include "CURRENT/current_sensor.h"

AS5600_T G_ANGLE_SENSOR_A;
DRV8313_T G_BLDC_DRIVER_A;
CURRENT_SENSOR_T G_CURRENT_SENSOR_A;

void Sensor_UpdateA() { 
	AS5600_Update(&G_ANGLE_SENSOR_A); 
	CurrentSensor_Update(&G_CURRENT_SENSOR_A);
}

float Sensor_GetOnceAngleA() { 
	return AS5600_GetOnceAngle(&G_ANGLE_SENSOR_A); 
}

float Sensor_GetAngleA() {
	return AS5600_GetAngle(&G_ANGLE_SENSOR_A); 
}

float Sensor_GetVelocityA() { 
	return AS5600_GetVelocity(&G_ANGLE_SENSOR_A);
}

void Driver_SetPWMA( int duty_u, int duty_v, int duty_w) {
	DRV8313_SetPWM(&G_BLDC_DRIVER_A, (uint32_t)duty_u, (uint32_t)duty_v, (uint32_t)duty_w);
}

int Sensor_GetPhaseCurrentsA(float *current_uvw, int num_channel) {
	return CURRENT_SENSOR_GetPhaseCurrents(&G_CURRENT_SENSOR_A, current_uvw, num_channel);
}

void FOC_HAL_InitA(	FOC_T *hfoc,
					TIM_HandleTypeDef *htim_u,
					uint32_t channel_u,
					TIM_HandleTypeDef *htim_v,
					uint32_t channel_v,
					TIM_HandleTypeDef *htim_w,
					uint32_t channel_w,
					I2C_HandleTypeDef *h2ic,
					ADC_HandleTypeDef *hadc) {
  if(AS5600_Init(&G_ANGLE_SENSOR_A, h2ic)) { Error_Handler(); }
  if(DRV8313_Init(&G_BLDC_DRIVER_A, htim_u, channel_u, htim_v, channel_v, htim_w, channel_w)) { Error_Handler(); }
  if(CURRENT_SENSOR_Init(&G_CURRENT_SENSOR_A, hadc)) { Error_Handler(); }

  FOC_Bind_Delay(hfoc, HAL_Delay);
  FOC_Bind_SensorUpdate(hfoc, Sensor_UpdateA);
  FOC_Bind_SensorGetOnceAngle(hfoc, Sensor_GetOnceAngleA);
  FOC_Bind_SensorGetAngle(hfoc, Sensor_GetAngleA);
  FOC_Bind_SensorGetVelocity(hfoc, Sensor_GetVelocityA);
  FOC_Bind_DriverSetPWM(hfoc, Driver_SetPWMA);
  FOC_Bind_SensorGetPhaseCurrents(hfoc, Sensor_GetPhaseCurrentsA);
}
