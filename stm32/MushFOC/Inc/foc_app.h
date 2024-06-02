/*
 * @Author: devinzhang@gmail.com
 * @Date: 2024-05-03 15:14:43
 * @LastEditors: DevinZhang91
 * @LastEditTime: 2024-05-03 15:14:43
 * @Description: Modified according to CAW FOC project https://github.com/GUAIK-ORG/CawFOC
 */

#ifndef __FOC_TEST_H__
#define __FOC_TEST_H__
#include "lowpass_filter.h"
#include "pid.h"
#include "foc.h"

typedef enum
{
  // Status
  FOC_ERROR					= 0x00U,
  FOC_OK					= 0x01U,
  FOC_INIT_DONE,

  // CMD
  FOC_SET_ON				= 0x20U,
  FOC_SET_OFF				= 0x21U,

  FOC_SET_ANGLE				= 0x30U,
  FOC_GET_ANGLE,
  FOC_SET_MAX_OUTPUT_ANGLE,
  FOC_GET_MAX_OUTPUT_ANGLE,
  FOC_SET_MAX_ZERO_ANGLE,
  FOC_GET_MAX_ZERO_ANGLE,
  FOC_SET_MIN_ZERO_ANGLE,
  FOC_GET_MIN_ZERO_ANGLE,

  FOC_SET_VELOCITY			= 0x40U,
  FOC_GET_VELOCITY,
  FOC_SET_MAX_OUTPUT_VELOCITY,
  FOC_GET_MAX_OUTPUT_VELOCITY,

  FOC_SET_CURRENT			= 0x50U,
  FOC_GET_CURRENT,
  FOC_SET_MAX_OUTPUT_CURRENT,
  FOC_GET_MAX_OUTPUT_CURRENT,

  FOC_RESTART				= 0xA5U,
} FOC_CMDTypeDef;

#define FOC_CMD_OFFSET		(16)
#define FOC_DATAH_OFFSET	(8)
#define FOC_DATAL_OFFSET	(0)

#define FOC_DEFAULT_MAX_OUTPUT_ANGLE		(1.0f)
#define FOC_DEFAULT_MAX_ZERO_ANGLE			(3.14f)
#define FOC_DEFAULT_MIN_ZERO_ANGLE			(-3.14f)
#define FOC_DEFAULT_MAX_OUTPUT_VELOCITY		(10.0f)
#define FOC_DEFAULT_MAX_OUTPUT_CURRENT		(3.0f)

typedef struct {
  float 		target_angle;
  float 		max_output_angle;
  float 		max_zero_angle;
  float 		min_zero_angle;

  float 		target_velocity;
  float 		max_output_velocity;

  float 		target_current;
  float 		max_output_current;
} FOC_APP_T;

void FOC_APP_Init(FOC_APP_T *hfoc_app);

void Foc_TestVelocity(FOC_T *hfoc,
					  LOWPASS_FILTER_T *hfilter_velocity,
					  PID_T *hpid_velocity,
					  float target_velocity);

void Foc_TestAngle(FOC_T *hfoc,
				   PID_T *hpid_angle,
				   float target_angle);

void Foc_TestCurrentVelocity(FOC_T *hfoc,
							 LOWPASS_FILTER_T *hfilter_current,
							 LOWPASS_FILTER_T *hfilter_velocity,
							 PID_T *hpid_current,
							 PID_T *hpid_velocity,
							 float target_velocity);

void Foc_TestCurrentAngle(FOC_T *hfoc,
						  LOWPASS_FILTER_T *hfilter_current,
						  PID_T *hpid_current,
						  PID_T *hpid_angle,
						  float target_angle);

void Foc_TestCurrentVelocityAngle(FOC_T *hfoc,
								  LOWPASS_FILTER_T *hfilter_current,
								  LOWPASS_FILTER_T *hfilter_velocity,
								  PID_T *hpid_current,
								  PID_T *hpid_velocity,
								  PID_T *hpid_angle,
								  float target_angle);
#endif
