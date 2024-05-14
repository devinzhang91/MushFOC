#include "current_sensor.h"

void _calADCmVolt(CURRENT_SENSOR_T *a){
    a->adc_mVolt[0] = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, a->adc_data[0], a->hadc->Init.Resolution );
    a->adc_mVolt[1] = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, a->adc_data[1], a->hadc->Init.Resolution );
#if USE_PINW
    // 如果有第三个电流传感器
    a->adc_mVolt[2] = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, a->adc_data[2], a->hadc->Init.Resolution );
#endif /* USE_PINW */
}

// 查找 ADC 零偏移量的函数
void _calibrateOffsets(CURRENT_SENSOR_T *a){
    const int calibration_rounds = 1000;

    // 查找0电流时候的电压
    a->offset_iu = 0;
    a->offset_iv = 0;
    a->offset_iw = 0;
    // 读数1000次
    for (int i = 0; i < calibration_rounds; i++) {
        HAL_Delay(5);
        // 读取电压
    	_calADCmVolt(a);
        a->offset_iu += a->adc_mVolt[0];
        a->offset_iv += a->adc_mVolt[1];
#if USE_PINW        
        // 如果有第三个电流传感器
        offset_iw += a->adc_mVolt[2];
#endif /* USE_PINW */ 
        // 启动ADC DMA读取电压到ADCxConvertedData
        if (HAL_ADC_Start(a->hadc) != HAL_OK) { Error_Handler(); }
    }
    // 求平均，得到误差
    a->offset_iu = a->offset_iu / calibration_rounds;
    a->offset_iv = a->offset_iv / calibration_rounds;
#if USE_PINW
    // 如果有第三个电流传感器
    a->offset_iw = a->offset_iw / calibration_rounds;
#endif /* USE_PINW */
}

int CURRENT_SENSOR_Init(CURRENT_SENSOR_T *a,
                        ADC_HandleTypeDef *hadc){
    a->hadc = hadc;
    // 电流传感器的电阻 0.010欧姆
    float _shunt_resistor = 0.010f;
    int _amp_gain  = 50;
    // volts to amps
    float volts_to_amps_ratio = 1.0f / _shunt_resistor / _amp_gain;
    // gains for each phase
    a->gain_u = volts_to_amps_ratio*-1;
    a->gain_v = volts_to_amps_ratio*-1;
    a->gain_w = volts_to_amps_ratio;

    // 启动ADC
    /* Run the ADC calibration */
    if (HAL_ADCEx_Calibration_Start(a->hadc) != HAL_OK) return -1;
    /* Start ADC group regular conversion with DMA */
    if (HAL_ADC_Start_DMA(a->hadc, (uint32_t *)(a->adc_data), 2) != HAL_OK) return -1;

    // 校准
    _calibrateOffsets(a);
    return 0;
}

void CurrentSensor_Update(CURRENT_SENSOR_T *a){
	_calADCmVolt(a);
    // 电流 = (电压 - 0电流时候的电压) * 电流传感器的增益
    a->current_u = (a->adc_mVolt[0] - a->offset_iu) * a->gain_u;
    a->current_v = (a->adc_mVolt[1] - a->offset_iv) * a->gain_v;
    a->current_w = 0;
#if USE_PINW
    // 如果有第三个电流传感器
    a->current_w = (a->adc_mVolt[2] - a->offset_iw) * a->gain_w;
#endif /* USE_PINW */

    // 启动ADC DMA读取电压到ADCxConvertedData
    if (HAL_ADC_Start(a->hadc) != HAL_OK) {
    	Error_Handler();
    }
}

int CURRENT_SENSOR_GetPhaseCurrents(CURRENT_SENSOR_T *a, float *current_uvw, int num_channel){
	//这里需要将mA转A输出
	current_uvw[0] = a->current_u/1000.0f;
	current_uvw[1] = a->current_v/1000.0f;
#if USE_PINW 
	current_uvw[2] = a->current_w/1000.0f;
#endif /* USE_PINW */
	return NUM_CURRENT_SENSORS;
}
