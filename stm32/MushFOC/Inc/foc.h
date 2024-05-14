/*
 * @Author: devinzhang@gmail.com
 * @Date: 2024-05-02 22:32:09
 * @LastEditors: DevinZhang91
 * @LastEditTime: 2024-05-02 22:32:09
 * @Description: Modified according to CAW FOC project https://github.com/GUAIK-ORG/CawFOC
 */
#ifndef __FOC_H__
#define __FOC_H__

// 获取编码器弧度值的函数指针
typedef void (*FUNC_DELAY)(int);
typedef void (*FUNC_SENSOR_UPDATE)();
typedef float (*FUNC_SENSOR_GET_ONCE_ANGLE)();
typedef float (*FUNC_SENSOR_GET_ANGLE)();
typedef float (*FUNC_SENSOR_GET_VELOCITY)();
typedef void (*FUNC_DRIVER_SET_PWM)(int, int, int);
typedef int (*FUNC_SENSOR_GET_PHASECURRENTS)(float*, int);

typedef struct {
  float pwm_period;
  float voltage_power_supply;  // 电源电压
  float voltage_limit;         // 电压限制
  float shaft_angle;           // 轴角度（开环）
  float open_loop_timestamp;   // 开环时间轴
  float zero_electric_angle;   // 零位电角
  float u_alpha;
  float u_beta;
  float u_a, u_b, u_c;
  float dc_a, dc_b, dc_c;

  int dir;  // 方向
  int pp;   // 极对数

  FUNC_DELAY System_Delay;
  FUNC_SENSOR_UPDATE Sensor_Update;
  FUNC_SENSOR_GET_ONCE_ANGLE Sensor_GetOnceAngle;
  FUNC_SENSOR_GET_ANGLE Sensor_GetAngle;
  FUNC_SENSOR_GET_VELOCITY Sensor_GetVelocity;
  FUNC_DRIVER_SET_PWM Driver_SetPWM;
  FUNC_SENSOR_GET_PHASECURRENTS Sensor_GetPhaseCurrents;
} FOC_T;

void FOC_Init(FOC_T *hfoc, float pwm_period, float voltage, int dir, int pp);
void FOC_Abort(FOC_T *hfoc);
void FOC_AlignmentSensor(FOC_T *hfoc);
void FOC_SetVoltageLimit(FOC_T *hfoc, float v);
float FOC_ElectricalAngle(FOC_T *hfoc);
void FOC_SetTorque(FOC_T *hfoc, float Uq, float angle_el);
void FOC_SensorUpdate(FOC_T *hfoc);
float FOC_GetCurrent(FOC_T *hfoc);

void FOC_Bind_Delay(FOC_T *hfoc, FUNC_DELAY t);
void FOC_Bind_SensorUpdate(FOC_T *hfoc, FUNC_SENSOR_UPDATE s);
void FOC_Bind_SensorGetOnceAngle(FOC_T *hfoc, FUNC_SENSOR_GET_ONCE_ANGLE s);
void FOC_Bind_SensorGetAngle(FOC_T *hfoc, FUNC_SENSOR_GET_ANGLE s);
void FOC_Bind_SensorGetVelocity(FOC_T *hfoc, FUNC_SENSOR_GET_VELOCITY s);
void FOC_Bind_DriverSetPWM(FOC_T *hfoc, FUNC_DRIVER_SET_PWM d);
void FOC_Bind_SensorGetPhaseCurrents(FOC_T *hfoc, FUNC_SENSOR_GET_PHASECURRENTS s);
#endif
