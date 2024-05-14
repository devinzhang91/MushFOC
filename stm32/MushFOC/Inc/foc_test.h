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
