/**
 * @file foc.h
 * @author 2743823168@qq.com
 * @brief FOC驱动头文件
 * @brief 本模板的实现是基于定时器中断的固定dt调用，enc结构体中也保留了time_prev和time_now以备后续拓展
 * @version 0.1
 * @date 2026-03-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef __FOC_H__ 
#define __FOC_H__
#include "main.h"
#include "tim.h"

#define _COMSTRAIN(AMT,MIN,MAX) ((AMT)<(MIN)?(MIN):((AMT)>(MAX)?(MAX):(AMT)))
#define FOC_POLE_PAIRS 7 // 电机极对数
#define VOLTAGE_LIMIT 6.06f
#define ANGLE_DEADZONE 0.0f // 电角度死区，防止小电压导致的抖动 (弧度)
#define VOLTAGE_DEADZONE 0.5f // 电压死区，防止小电压导致的抖动
#define M_PI 3.14159265358979323846f

#define DEFAULT_KP 0.8f

/*-------------------基础功能--------------------*/
//获取电角度
float GetelectricalAngle(float mechanicalAngle);

//角度归一化
float nomalizeAngle(float angle);

//设置PWM输出
void setPWM(float Ua, float Ub, float Uc);

/**
 * @brief Set the Phase Voltage object
 * 
 * @param Uq 
 * @param Ud 
 * @param el_angle 单位为弧度
 */
void setPhaseVoltage(float Uq, float Ud, float el_angle);

float velocityToVoltage(float velocity);

// 开环速度控制：target_rpm 单位为 RPM，dt 单位为 秒
void openLoopSpeedControl(float target_rpm, float dt);

// 角度控制
float angleControl(float target_angle, float current_angle, float Kp);

//力矩控制
float torqueControl(float Uq,float eleangle);











#endif /* __FOC_H__ */
