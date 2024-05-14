/*
 * @Author: devinzhang@gmail.com
 * @Date: 2024-05-02 22:32:09
 * @LastEditors: DevinZhang91
 * @LastEditTime: 2024-05-02 22:32:09
 * @Description: Modified according to CAW FOC project https://github.com/GUAIK-ORG/CawFOC
 */
#include "foc.h"

#include <math.h>

#define _3PI_2 4.71238898038469f
#define _PI 3.141592653589793f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

#define _constrain(amt, low, high) \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// 归一化角度到 [0, 2PI]
float _normalizeAngle(float angle) {
  float a = fmod(angle, 2 * _PI);
  return a >= 0 ? a : (a + 2 * _PI);
}

// 求解电角度0专用，防止初始化的时候负角度判定为大角度，导致旋转一圈
float _electricalAngle0(FOC_T *hfoc) {
  return _normalizeAngle((float)(hfoc->dir * hfoc->pp) *
                             hfoc->Sensor_GetOnceAngle() -
                         hfoc->zero_electric_angle);
}

void _setPwm(FOC_T *hfoc, float Ua, float Ub, float Uc) {
  Ua = _constrain(Ua, 0.0f, hfoc->voltage_limit);
  Ub = _constrain(Ub, 0.0f, hfoc->voltage_limit);
  Uc = _constrain(Uc, 0.0f, hfoc->voltage_limit);

  float dc_a = _constrain(Ua / hfoc->voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / hfoc->voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / hfoc->voltage_power_supply, 0.0f, 1.0f);

  hfoc->Driver_SetPWM(dc_a * hfoc->pwm_period, dc_b * hfoc->pwm_period, dc_c * hfoc->pwm_period);
}

/**
 * @description: 获取闭环控制电角度数据
 * @param {FOC_T} *hfoc foc句柄
 * @return {float} 电角度值
 */
float FOC_ElectricalAngle(FOC_T *hfoc) {
  return _normalizeAngle((float)(hfoc->dir * hfoc->pp) *
                             hfoc->Sensor_GetOnceAngle() -
                         hfoc->zero_electric_angle);
}

/**
 * @description: 设置力矩
 * @param {FOC_T} *hfoc foc句柄
 * @param {float} Uq 力矩值
 * @param {float} angle_el 电角度
 * @return {*}
 */
void FOC_SetTorque(FOC_T *hfoc, float Uq, float angle_el) {
  Uq = _constrain(Uq, -hfoc->voltage_power_supply / 2,
                  hfoc->voltage_power_supply / 2.0);
  float Ud = 0;
  angle_el = _normalizeAngle(angle_el);

  // Park逆变换
  float Ualpha = -Uq * sin(angle_el);
  float Ubeta = Uq * cos(angle_el);

  // Clark逆变换
  float Ua = Ualpha + hfoc->voltage_power_supply / 2.0;
  float Ub =
      (sqrt(3) * Ubeta - Ualpha) / 2.0 + hfoc->voltage_power_supply / 2.0;
  float Uc =
      (-Ualpha - sqrt(3) * Ubeta) / 2.0 + hfoc->voltage_power_supply / 2.0;
  _setPwm(hfoc, Ua, Ub, Uc);
}

/**
 * @description: FOC控制初始化
 * @param {FOC_T} *hfoc foc句柄
 * @param {float} pwm_period PWM的重装载值
 * @param {float} voltage 电源电压值
 * @param {int} dir 方向
 * @param {int} pp 极对数
 * @return {*}
 */
void FOC_Init(FOC_T *hfoc, float pwm_period, float voltage, int dir, int pp) {
  (void *)memset((void *)hfoc, 0, sizeof(FOC_T));
  hfoc->pwm_period = pwm_period;
  // 配置电源电压和限压
  hfoc->voltage_power_supply = voltage;
  hfoc->voltage_limit = voltage;
  // 配置方向和极对数
  hfoc->dir = dir;
  hfoc->pp = pp;
}

/**
 * @description: FOC紧急停止，释放PWM输出。
 * @param {FOC_T} *hfoc foc句柄
 * @return {*}
 */
void FOC_Abort(FOC_T *hfoc){
  // 配置电源电压和限压
  hfoc->voltage_power_supply = 0.0f;
  hfoc->voltage_limit = 0.0f;
  hfoc->Driver_SetPWM(0, 0, 0);
}

/**
 * @description: 设置电压限制
 * @param {FOC_T} *hfoc foc句柄
 * @param {float} v 电压值
 * @return {*}
 */
void FOC_SetVoltageLimit(FOC_T *hfoc, float v) { hfoc->voltage_limit = v; }

/**
 * @description: 编码器零位较准（需要配置传感器相关函数指针）
 * @param {FOC_T} *hfoc foc句柄
 * @return {*}
 */
void FOC_AlignmentSensor(FOC_T *hfoc) {
  // 较准0位电角度
  if (hfoc->Sensor_GetAngle && hfoc->Sensor_GetOnceAngle) {
	//起劲
    FOC_SetTorque(hfoc, 3, _3PI_2);
	//等待转至对应角度， 更新角度
    hfoc->System_Delay(1000);
    hfoc->zero_electric_angle = _electricalAngle0(hfoc);
	//松劲
    FOC_SetTorque(hfoc, 0, _3PI_2);
    hfoc->Sensor_Update();
  }
}


/**
 * @description: 通过Ia,Ib,Ic计算Iq,Id(目前仅输出Iq)
 * @param {FOC_T} *hfoc foc句柄
 * @param {float} current_u 电流值
 * @param {float} current_v 电流值
 * @param {float} angle_el 电角度
 * @return {float} Iq值
 */
float _calIqId(float current_u, float current_v, float angle_el) {
  float I_alpha=current_u;
  float I_beta = _1_SQRT3 * current_u + _2_SQRT3 * current_v;

  float ct = cos(angle_el);
  float st = sin(angle_el);
  //float I_d = I_alpha * ct + I_beta * st;
  float I_q = I_beta * ct - I_alpha * st;
  return I_q;
}

/**
 * @description: 电流读取
 * @param {FOC_T} *hfoc foc句柄
 * @return {float}  Iq值
 */
float FOC_GetCurrent(FOC_T *hfoc) {
  // 读取电流，这里假设电流传感器的数据是Iu,Iv
  float current_uv[2];
  hfoc->Sensor_GetPhaseCurrents(current_uv, 2);
  float _electricalAngle = FOC_ElectricalAngle(hfoc);
  float Iq = _calIqId(current_uv[0], current_uv[1], _electricalAngle);
  return Iq;  
}

/**
 * @description: 绑定用于延时ms的函数
 * @param {FOC_T} *hfoc foc句柄
 * @param {FUNC_SENSOR_GET_ONCE_ANGLE} s 函数指针
 * @return {*}
 */
void FOC_Bind_Delay(FOC_T *hfoc, FUNC_DELAY t) {
  hfoc->System_Delay = t;
}

/**
 * @description: 绑定用于获取单圈弧度值的函数（0 - 6.28）
 * @param {FOC_T} *hfoc foc句柄
 * @param {FUNC_SENSOR_GET_ONCE_ANGLE} s 函数指针
 * @return {*}
 */
void FOC_Bind_SensorGetOnceAngle(FOC_T *hfoc, FUNC_SENSOR_GET_ONCE_ANGLE s) {
  hfoc->Sensor_GetOnceAngle = s;
}

/**
 * @description: 绑定用于获取累计弧度值的函数
 * @param {FOC_T} *hfoc foc句柄
 * @param {FUNC_SENSOR_GET_ANGLE} s 函数指针
 * @return {*}
 */
void FOC_Bind_SensorGetAngle(FOC_T *hfoc, FUNC_SENSOR_GET_ANGLE s) {
  hfoc->Sensor_GetAngle = s;
}

/**
 * @description: 绑定用于更新计数值相关的函数
 * @param {FOC_T} *hfoc foc句柄
 * @param {FUNC_SENSOR_UPDATE} s 函数指针
 * @return {*}
 */
void FOC_Bind_SensorUpdate(FOC_T *hfoc, FUNC_SENSOR_UPDATE s) {
  hfoc->Sensor_Update = s;
}

/**
 * @description: 绑定用户获取速度值得函数
 * @param {FOC_T} *hfoc foc句柄
 * @param {FUNC_SENSOR_GET_VELOCITY} s 函数指针
 * @return {*}
 */
void FOC_Bind_SensorGetVelocity(FOC_T *hfoc, FUNC_SENSOR_GET_VELOCITY s) {
  hfoc->Sensor_GetVelocity = s;
}

/**
 * @description: 绑定输出PWM函数
 * @param {FOC_T} *hfoc foc句柄
 * @param {FUNC_SENSOR_GET_VELOCITY} d 函数指针
 * @return {*}
 */
void FOC_Bind_DriverSetPWM(FOC_T *hfoc, FUNC_DRIVER_SET_PWM d) {
  hfoc->Driver_SetPWM = d;
}

/**
 * @description: 绑定获取电流函数
 * @param {FOC_T} *hfoc foc句柄
 * @param {FUNC_SENSOR_GET_CURRENT} s 函数指针
 * @return {*}
 */
void FOC_Bind_SensorGetPhaseCurrents(FOC_T *hfoc, FUNC_SENSOR_GET_PHASECURRENTS s) {
  hfoc->Sensor_GetPhaseCurrents = s;
}

/**
 * @description: 更新传感器数据
 * @param {FOC_T} *hfoc foc句柄
 * @return {*}
 */
void FOC_SensorUpdate(FOC_T *hfoc) { hfoc->Sensor_Update(); }
