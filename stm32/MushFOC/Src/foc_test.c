/*
 * @Author: devinzhang@gmail.com
 * @Date: 2024-05-03 15:14:43
 * @LastEditors: DevinZhang91
 * @LastEditTime: 2024-05-03 15:14:43
 * @Description: Modified according to CAW FOC project https://github.com/GUAIK-ORG/CawFOC
 */
#include "foc_test.h"

#define _PI 3.141592653589793f

// 测试闭环速度控制
void Foc_TestVelocity(FOC_T *hfoc,
					  LOWPASS_FILTER_T *hfilter_velocity,
					  PID_T *hpid_velocity,
					  float target_velocity) {
  float sensor_velocity = hfoc->Sensor_GetVelocity();
  sensor_velocity = LOWPASS_FILTER_Calc(hfilter_velocity, hfoc->dir * sensor_velocity);
  PID_Set(hpid_velocity, 0.001, 0.01, 0, 0, 20);
  FOC_SetTorque(hfoc,
		  	  	PID_Calc(hpid_velocity, (target_velocity - sensor_velocity) * 180 / _PI),
				FOC_ElectricalAngle(hfoc));
}

// 测试闭环位置控制和力矩控制
void Foc_TestAngle(FOC_T *hfoc,
				   PID_T *hpid_angle,
				   float target_angle) {
  float sensor_angle = hfoc->Sensor_GetAngle();
  // float Kp = 0.133;
  // 位置控制
  PID_Set(hpid_angle, 0.1, 0, 0, 0, 20);
  FOC_SetTorque(hfoc,
                PID_Calc(hpid_angle, (target_angle - hfoc->dir * sensor_angle) * 180 / _PI),
                FOC_ElectricalAngle(hfoc));
  // 力矩控制
  // FOC_SetTorque(hfoc, angle, FOC_CloseloopElectricalAngle(hfoc));
}

/************************************************电流反馈力矩环***************************************************/
//电流力矩环
void Foc_SetCurrentTorque(FOC_T *hfoc,
						  LOWPASS_FILTER_T *hfilter_current,
						  PID_T *hpid_current,
						  float target_current) {
	float Iq = FOC_GetCurrent(hfoc);
	Iq = LOWPASS_FILTER_Calc(hfilter_current, Iq);
	FOC_SetTorque(hfoc,
				  PID_Calc(hpid_current, (target_current - Iq)),
				  FOC_ElectricalAngle(hfoc));
}

// 测试电流闭环速度控制
void Foc_TestCurrentVelocity(FOC_T *hfoc,
							 LOWPASS_FILTER_T *hfilter_current,
							 LOWPASS_FILTER_T *hfilter_velocity,
							 PID_T *hpid_current,
							 PID_T *hpid_velocity,
							 float target_velocity) {

  // 测试用例的pid
  PID_Set(hpid_current, 0.2, 0.01, 0, 100000, 6);
  PID_Set(hpid_velocity, 0.01, 0.01, 0, 100000, hfoc->voltage_power_supply / 2);

  float sensor_velocity = hfoc->Sensor_GetVelocity();
  sensor_velocity = LOWPASS_FILTER_Calc(hfilter_velocity, hfoc->dir * sensor_velocity);
  float target_current = PID_Calc(hpid_velocity, (target_velocity - sensor_velocity)*180/_PI);
  Foc_SetCurrentTorque(hfoc,
		  	  	  	   hfilter_current,
					   hpid_current,
					   target_current);
}

// 测试电流闭环位置控制
void Foc_TestCurrentAngle(FOC_T *hfoc,
						  LOWPASS_FILTER_T *hfilter_current,
						  PID_T *hpid_current,
						  PID_T *hpid_angle,
						  float target_angle) {
  // 测试用例的pid
  PID_Set(hpid_current, 0.25, 0.01, 0.000, 100000, 6);
  PID_Set(hpid_angle, 0.5, 0.01, 0, 100000, 10);

  float sensor_angle = hfoc->Sensor_GetAngle();
  float target_current = PID_Calc(hpid_angle, (target_angle - hfoc->dir * sensor_angle)*180/_PI);
  Foc_SetCurrentTorque(hfoc, hfilter_current, hpid_current, target_current);
}

// 测试电流闭环速度-位置控制
void Foc_TestCurrentVelocityAngle(FOC_T *hfoc,
								  LOWPASS_FILTER_T *hfilter_current,
								  LOWPASS_FILTER_T *hfilter_velocity,
								  PID_T *hpid_current,
								  PID_T *hpid_velocity,
								  PID_T *hpid_angle,
								  float target_angle) {
  // 测试用例的pid
  PID_Set(hpid_current, 1.2, 2, 0.01, 100000, hfoc->voltage_power_supply / 2);
  PID_Set(hpid_velocity, 0.1, 2, 0, 100000, 1);
  PID_Set(hpid_angle, 0.6, 0.01, 0.001, 100000, 6);

  float sensor_angle = hfoc->Sensor_GetAngle();
  float sensor_velocity = hfoc->Sensor_GetVelocity();
  sensor_velocity = LOWPASS_FILTER_Calc(hfilter_velocity, hfoc->dir * sensor_velocity);
  float target_velocity = PID_Calc(hpid_angle, (target_angle - hfoc->dir * sensor_angle)*180/_PI);
  float target_current = PID_Calc(hpid_velocity, (target_velocity - sensor_velocity));
  Foc_SetCurrentTorque(hfoc, hfilter_current, hpid_current, target_current);

}

