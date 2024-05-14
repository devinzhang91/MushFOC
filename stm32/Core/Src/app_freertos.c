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

typedef enum
{
  FOC_ERROR				= 0x10U,
  FOC_INIT_DONE			= 0x11U,
  FOC_SET_ON			= 0x20U,
  FOC_SET_OFF			= 0x21U,
  FOC_SET_ANGLE			= 0x22U,
  FOC_GET_ANGLE			= 0x23U,
  FOC_SET_VELOCITY		= 0x24U,
  FOC_GET_VELOCITY		= 0x25U,

  FOC_RESTART			= 0xA5U,
  FOC_CMD_NUM,
} FOC_CMDTypeDef;

#define FOC_CMD_OFFSET		(16)
#define FOC_DATAH_OFFSET	(8)
#define FOC_DATAL_OFFSET	(0)

#define EVENT_FOC_ERROR				(1<<FOC_ERROR)
#define EVENT_FOC_INIT_DONE			(1<<FOC_INIT_DONE)
#define EVENT_FOC_SET_ON			(1<<FOC_SET_ON)
#define EVENT_FOC_SET_OFF			(1<<FOC_SET_OFF)
#define EVENT_FOC_SET_ANGLE			(1<<FOC_SET_ANGLE)
#define EVENT_FOC_GET_ANGLE			(1<<FOC_GET_ANGLE)
#define EVENT_FOC_SET_VELOCITY		(1<<FOC_SET_VELOCITY)
#define EVENT_FOC_GET_VELOCITY		(1<<FOC_GET_VELOCITY)
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

uint16_t targetAngle = 90;
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
  osEventFlagsSet(focEventHandle, EVENT_FOC_INIT_DONE);
  /* Infinite loop */
  for(;;)
  {
	float targetAngleRad = ((float)targetAngle - 90)/180.0f*_PI;
//	Foc_TestAngle(&foc, &anglePID, 0);
//	Foc_TestVelocity(&foc, &velocityFilter, &velocityPID, 10);
//	Foc_TestCurrentVelocity(&foc, &currentFilter, &velocityFilter, &currentPID, &velocityPID, 10);
//	Foc_TestCurrentAngle(&foc, &currentFilter, &currentPID, &anglePID, targetAngleRad);
	Foc_TestCurrentVelocityAngle(&foc, &currentFilter, &velocityFilter, &currentPID, &velocityPID, &anglePID, targetAngleRad);
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
				   EVENT_FOC_INIT_DONE,		/* 接收任务感兴趣的事件 */
				   osFlagsWaitAll,          /* 退出时清除事件位，同时满足感兴趣的所有事件 */
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
				targetAngle = data;
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
		case FOC_SET_VELOCITY:
			uint32_t revData = 	((uint32_t)i2cRxBuff[0]<<FOC_CMD_OFFSET) |
								((uint32_t)i2cRxBuff[1]<<FOC_DATAH_OFFSET) |
								((uint32_t)i2cRxBuff[2]<<FOC_DATAL_OFFSET);
		    osStatus_t xReturn = osMessageQueuePut(i2cQueueHandle, 	/* 消息队列的句柄*/
		                   	   	   	   	   	   	   &revData,		/* 发送的消息内容 */
												   0,               /* 发送优先级*/
												   0 );        		/* 等待时间 0 */
		    if(osOK != xReturn) { Error_Handler(); }
			break;
		default:
			break;
	}

}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
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

