/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _PI 3.141592653589793f
#define EVENT(x)			(1<<x)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t i2cRxBuff[I2C_SLAVE_RX_LEN];
uint8_t i2cTxBuff[I2C_SLAVE_TX_LEN];
uint8_t uartTxBuff[32] = {0};

FOC_T foc;
PID_T velocityPID;
PID_T anglePID;
PID_T currentPID;
LOWPASS_FILTER_T currentFilter;
LOWPASS_FILTER_T velocityFilter;

FOC_APP_T foc_app;

/* USER CODE END Variables */
/* Definitions for focTask */
osThreadId_t focTaskHandle;
const osThreadAttr_t focTask_attributes = {
  .name = "focTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 192 * 4
};
/* Definitions for msgTask */
osThreadId_t msgTaskHandle;
const osThreadAttr_t msgTask_attributes = {
  .name = "msgTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 64 * 4
};
/* Definitions for watchdogTask */
osThreadId_t watchdogTaskHandle;
const osThreadAttr_t watchdogTask_attributes = {
  .name = "watchdogTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 96 * 4
};
/* Definitions for i2cQueue */
osMessageQueueId_t i2cQueueHandle;
const osMessageQueueAttr_t i2cQueue_attributes = {
  .name = "i2cQueue"
};
/* Definitions for ledTimer */
osTimerId_t ledTimerHandle;
const osTimerAttr_t ledTimer_attributes = {
  .name = "ledTimer"
};
/* Definitions for focEvent */
osEventFlagsId_t focEventHandle;
const osEventFlagsAttr_t focEvent_attributes = {
  .name = "focEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartFOCTask(void *argument);
void StartMsgTask(void *argument);
void StartWatchDogTask(void *argument);
void LedCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of ledTimer */
  ledTimerHandle = osTimerNew(LedCallback, osTimerPeriodic, NULL, &ledTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(ledTimerHandle, 1000);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of i2cQueue */
  i2cQueueHandle = osMessageQueueNew (8, sizeof(uint32_t), &i2cQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of focTask */
  focTaskHandle = osThreadNew(StartFOCTask, NULL, &focTask_attributes);

  /* creation of msgTask */
  msgTaskHandle = osThreadNew(StartMsgTask, NULL, &msgTask_attributes);

  /* creation of watchdogTask */
  watchdogTaskHandle = osThreadNew(StartWatchDogTask, NULL, &watchdogTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of focEvent */
  focEventHandle = osEventFlagsNew(&focEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartFOCTask */
/**
  * @brief  Function implementing the focTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartFOCTask */
void StartFOCTask(void *argument)
{
  /* USER CODE BEGIN StartFOCTask */

  float periodPWM = (1+htim14.Init.Period);
  FOC_APP_Init(&foc_app);
  FOC_Init(&foc, periodPWM, 12.6, -1, 7);
  FOC_SetVoltageLimit(&foc, 12.0f);
  // 初始化 FOC HAL
  FOC_HAL_InitA(&foc,
				&htim14, TIM_CHANNEL_1,
				&htim16, TIM_CHANNEL_1,
				&htim17, TIM_CHANNEL_1,
				&hi2c1,
				&hadc1);

  PID_Init(&velocityPID, 0.01, 0.01, 0, 100000, foc.voltage_power_supply / 2);
  PID_Init(&anglePID, 0.5, 0.01, 0, 100000, 10);
  PID_Init(&currentPID, 0.2, 0.01, 0, 100000, 6);

  LOWPASS_FILTER_Init(&currentFilter, 0.02);
  LOWPASS_FILTER_Init(&velocityFilter, 0.02);

  FOC_AlignmentSensor(&foc);
  //set start angle and disable foc
//  FOC_SetVoltageLimit(&foc, 0.0f);
  // set FOC init done flag
  osEventFlagsSet(focEventHandle, EVENT(FOC_INIT_DONE));
  /* Infinite loop */
  for(;;)
  {
//	Foc_TestAngle(&foc, &anglePID, 0);
//	Foc_TestVelocity(&foc, &velocityFilter, &velocityPID, 10);
//	Foc_TestCurrentVelocity(&foc, &currentFilter, &velocityFilter, &currentPID, &velocityPID, 10);
//	Foc_TestCurrentAngle(&foc, &currentFilter, &currentPID, &anglePID, targetAngleRad);
	Foc_TestCurrentVelocityAngle(&foc, &currentFilter, &velocityFilter, &currentPID, &velocityPID, &anglePID, foc_app.target_angle);
	FOC_SensorUpdate(&foc);
    osDelay(1);
  }
  /* USER CODE END StartFOCTask */
}

/* USER CODE BEGIN Header_StartMsgTask */
/**
* @brief Function implementing the msgTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMsgTask */
void StartMsgTask(void *argument)
{
  /* USER CODE BEGIN StartMsgTask */
  /* 读取队列时, 用这个变量来存放数据 */
  uint32_t revValue;
  osStatus_t xReturn = osOK;

  osEventFlagsWait(focEventHandle,         	/* 事件对象句柄 */
		  	  	   EVENT(FOC_INIT_DONE),	/* 接收任务感兴趣的事件 */
				   osFlagsNoClear,          /* 退出时清除事件位，同时满足感兴趣的所有事件 */
				   osWaitForever);          /* 指定超时事件,一直等 */
  /* Infinite loop */
  for(;;)
  {
	 xReturn = osMessageQueueGet(i2cQueueHandle, 	/* 消息队列的句柄 */
							 	 &revValue,			/* 需要接受的消息内容存放地址 */
								 0,                 /* 接收优先级*/
								 100);  			/*永远等待 */
	 if(osOK == xReturn) {
		//接收3字节的写入数据
		//第一个字节：CMD命令(i2cRxBuff[0])
		//第二第三字：数据(((uint16_t)i2cRxBuff[1]<<8) | i2cRxBuff[2];)
		 uint8_t cmd = (revValue>>FOC_CMD_OFFSET)&0xFF;
		 uint16_t data = (revValue)&0xFFFF;
		 switch(cmd){
			case FOC_SET_ON:
				PID_Clear(&velocityPID);
				PID_Clear(&anglePID);
				PID_Clear(&currentPID);
				LOWPASS_FILTER_Clear(&currentFilter);
				LOWPASS_FILTER_Clear(&velocityFilter);
				FOC_SetVoltageLimit(&foc, 12.0f);
				break;
			case FOC_SET_OFF:
				FOC_SetVoltageLimit(&foc, 0.0f);
				break;

			case FOC_SET_ANGLE:
				float target_angle_rad = (data - 90.0f)/180.0f*_PI;
				foc_app.target_angle = target_angle_rad;
				break;
			case FOC_SET_MAX_ZERO_ANGLE:
				foc_app.max_zero_angle = (float)data;
				break;
			case FOC_SET_MIN_ZERO_ANGLE:
				foc_app.min_zero_angle = (float)data;
				break;
			case FOC_SET_MAX_OUTPUT_VELOCITY:
				foc_app.max_output_velocity = (float)data;
				break;
			case FOC_SET_MAX_OUTPUT_CURRENT:
				foc_app.max_output_current = (float)data;
				break;

			case FOC_RESTART:
				__HAL_RCC_CLEAR_RESET_FLAGS();
				HAL_Delay(500);
				HAL_NVIC_SystemReset();
				break;
		 }
	 }

    osDelay(1);
  }
  /* USER CODE END StartMsgTask */
}

/* USER CODE BEGIN Header_StartWatchDogTask */
/**
* @brief Function implementing the watchdogTask thread.
* @param argument: Not used
* @retval None
*/
#define	NUM_HISTORYs_RECORD				(32)
#define	ACCUMULATED_THRESHOLD_ROATIO	(0.3)
#define	ACCUMULATED_THRESHOLD			(ACCUMULATED_THRESHOLD_ROATIO*NUM_HISTORYs_RECORD*255*255)
/* USER CODE END Header_StartWatchDogTask */
void StartWatchDogTask(void *argument)
{
  /* USER CODE BEGIN StartWatchDogTask */
  uint8_t histroy_pointer = 0;
  uint8_t histroy_current[NUM_HISTORYs_RECORD] = {0};
//  uint8_t histroy_angle[NUM_HISTORYs_RECORD] = {0};
  int accumulated_heat = 0;
  uint32_t abs_current = 0;
  FOC_T* hfoc = &foc;

  osEventFlagsWait(focEventHandle,         	/* 事件对象句柄 */
		  	  	   EVENT(FOC_INIT_DONE),	/* 接收任务感兴趣的事件 */
				   osFlagsNoClear,          /* 退出时清除事件位，同时满足感兴趣的所有事件 */
				   osWaitForever);          /* 指定超时事件,一直等 */
  /* Infinite loop */
  for(;;)
  {
	float sensor_angle = hfoc->Sensor_GetAngle();
	float Iq = FOC_GetCurrent(hfoc);
	abs_current = (uint32_t)(Iq<0? (Iq*-1000) : (Iq*1000)); ///Absolute value of current(mA)
	uint8_t cur_current = abs_current>255? 255:abs_current;
	accumulated_heat += (cur_current*cur_current);	///Q = current^2 * R
	accumulated_heat -= (histroy_current[histroy_pointer]*histroy_current[histroy_pointer]);
	histroy_current[histroy_pointer] = cur_current;
	if(++histroy_pointer == NUM_HISTORYs_RECORD){
		histroy_pointer = 0;
	}

//	uint32_t threshold_current = (uint32_t)(ACCUMULATED_CURRENT_THRESHOLD_RATIO*foc_app.max_output_current*NUM_HISTORYs_RECORD);
	if(accumulated_heat<0) accumulated_heat=0;
	if(accumulated_heat > ACCUMULATED_THRESHOLD){
		FOC_Abort(&foc);
		Error_Handler();
	}
    osDelay(100);
  }
  /* USER CODE END StartWatchDogTask */
}

/* LedCallback function */
void LedCallback(void *argument)
{
  /* USER CODE BEGIN LedCallback */
  HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
  /* USER CODE END LedCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(&hi2c2); // Restart
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	UNUSED(AddrMatchCode);

	if(TransferDirection == I2C_DIRECTION_TRANSMIT) {
		HAL_I2C_Slave_Sequential_Receive_IT(&hi2c2, &i2cRxBuff[0], I2C_SLAVE_RX_LEN, I2C_LAST_FRAME);
	}
	else {
		HAL_I2C_Slave_Sequential_Transmit_IT(&hi2c2, &i2cTxBuff[0], I2C_SLAVE_TX_LEN, I2C_LAST_FRAME);	}

}
/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}
/**
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle.
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
//	strcpy(uartTxBuff, "On SlaveRxCpltCallback \n");
//	if (HAL_UART_Transmit(&huart2, (uint8_t *)uartTxBuff, strlen(uartTxBuff), 1000) != HAL_OK) { Error_Handler(); }
//	memcpy(uartTxBuff, i2cRxBuff, 32);
//	if (HAL_UART_Transmit(&huart2, (uint8_t *)uartTxBuff, strlen(uartTxBuff), 1000) != HAL_OK) { Error_Handler(); }
	//接收3字节的写入数据
	//第一个字节：CMD命令(i2cRxBuff[0])
	//第二第三字：数据(((uint16_t)i2cRxBuff[1]<<8) | i2cRxBuff[2];)
	uint8_t cmd = i2cRxBuff[0];
	switch(cmd){
		case FOC_RESTART:
		case FOC_SET_ON:
		case FOC_SET_OFF:
		case FOC_SET_ANGLE:
		case FOC_SET_MAX_OUTPUT_ANGLE:
		case FOC_SET_MAX_ZERO_ANGLE:
		case FOC_SET_MIN_ZERO_ANGLE:
		case FOC_SET_VELOCITY:
		case FOC_SET_MAX_OUTPUT_VELOCITY:
		case FOC_SET_CURRENT:
		case FOC_SET_MAX_OUTPUT_CURRENT:

			uint32_t revData = 	((uint32_t)cmd			<<FOC_CMD_OFFSET) |
								((uint32_t)i2cRxBuff[1]	<<FOC_DATAH_OFFSET) |
								((uint32_t)i2cRxBuff[2]	<<FOC_DATAL_OFFSET);
		    osStatus_t xReturn = osMessageQueuePut(i2cQueueHandle, 	/* 消息队列的句柄*/
		                   	   	   	   	   	   	   &revData,		/* 发送的消息内容 */
												   0,               /* 发送优先级*/
												   0 );        		/* 等待时间 0 */
		    if(osOK != xReturn) { Error_Handler(); }
			break;

		case FOC_GET_MAX_OUTPUT_ANGLE:
		case FOC_GET_MAX_ZERO_ANGLE:
		case FOC_GET_MIN_ZERO_ANGLE:
		case FOC_GET_VELOCITY:
		case FOC_GET_MAX_OUTPUT_VELOCITY:
		case FOC_GET_CURRENT:
		case FOC_GET_MAX_OUTPUT_CURRENT:
		default:
			break;
	}

}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* In case of ADC error, call main error handler */
  Error_Handler();
}
/* USER CODE END Application */

