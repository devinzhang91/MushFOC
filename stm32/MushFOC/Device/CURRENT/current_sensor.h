/*
 * current_sensor.h
 *
 *  Created on: Apr 25, 2024
 *      Author: devin
 */

#ifndef _CURRENT_SENSOR_H_
#define _CURRENT_SENSOR_H_
#include <stdint.h>
#include "stm32g0xx_hal.h"

#define  USE_PINW 0
#define  NUM_CURRENT_SENSORS (2+USE_PINW)
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI          (3300UL)

typedef struct {
    ADC_HandleTypeDef *hadc;
    float current_u;
    float current_v;
    float current_w;
    float offset_iu;
    float offset_iv;
    float offset_iw;
    float gain_u;
    float gain_v;
    float gain_w;

    /* Variables for ADC conversion data */
    uint32_t   adc_data[NUM_CURRENT_SENSORS]; /* ADC group regular conversion data */
    /* Variables for ADC conversion data computation to physical values */
    uint32_t   adc_mVolt[NUM_CURRENT_SENSORS];  /* Value of voltage calculated from ADC conversion data (unit: mV) */

} CURRENT_SENSOR_T;

int CURRENT_SENSOR_Init(CURRENT_SENSOR_T *a,          
                        ADC_HandleTypeDef *hadc);

void CurrentSensor_Update(CURRENT_SENSOR_T *a);
int CURRENT_SENSOR_GetPhaseCurrents(CURRENT_SENSOR_T *a, float *current_uvw, int num_channel);


#endif /* _CURRENT_SENSOR_H_ */
