/**
 * @file Encoder.h
 * @author 2743823168@qq.com
 * @brief AS5600编码器头文件
 * @version 0.1
 * @date 2026-03-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "i2c.h"

#define AS5600_I2C_ADDR_7BIT 0x36U
#define AS5600_RESOLUTION    4096U
#define ENCODER_SIGN -1
#define ENCODER_I2C_FAIL_RECOVER_THRESHOLD 3U

extern float target_angle;

typedef struct
{
	HAL_StatusTypeDef last_status;
	uint32_t error_count;
	uint16_t consecutive_failures;
	uint16_t last_raw_angle;
	uint8_t valid;
	uint8_t recovery_count;
} EncoderStatus_t;

typedef struct
{
	I2C_HandleTypeDef *hi2c;   // AS5600 所在 I2C 句柄
	uint16_t i2c_addr_7bit;    // AS5600 7位 I2C 地址（默认 0x36）
	uint32_t cpr;              // 每机械圈计数（AS5600原生为4096）
	float pole_pairs;          // 电机极对数

	int32_t last_count;        // 上一次采样计数值
	int64_t total_count;       // 累积机械计数
	uint8_t is_started;        // 驱动启动状态

	float mech_angle_rad;      // 机械角（rad, 0~2pi）
	float elec_angle_rad;      // 电角度（rad, 0~2pi）
	float mech_velocity_rad_s; // 机械角速度（rad/s）
	float mech_velocity_rpm;   // 机械转速（RPM）

	float elec_zero_offset_rad; // 电角度零偏（rad）

	EncoderStatus_t status;    // I2C读取和恢复状态

	//非定时器使用
	uint32_t time_prev;
	uint32_t time_now;
} Encoder_t;


extern Encoder_t encoder_up;
extern Encoder_t encoder_down;

/**
 * @brief 
 * 
 * @param enc 
 * @param hi2c 
 * @param cpr 
 * @param pole_pairs 
 */
void Encoder_Init(Encoder_t *enc, I2C_HandleTypeDef *hi2c, uint32_t cpr, float pole_pairs);
HAL_StatusTypeDef Encoder_Start(Encoder_t *enc);
HAL_StatusTypeDef Encoder_Stop(Encoder_t *enc);

void Encoder_SetMechanicalZero(Encoder_t *enc);
void Encoder_SetElectricalZeroOffset(Encoder_t *enc, float offset_rad);

// dt_s 为采样周期（秒），建议固定周期调用
HAL_StatusTypeDef Encoder_Update(Encoder_t *enc, float dt_s);

//无固定采样周期
HAL_StatusTypeDef Encoder_Updata(Encoder_t *enc);

EncoderStatus_t Encoder_GetStatus(const Encoder_t *enc);
void Encoder_ClearDiagnostics(Encoder_t *enc);
uint8_t Encoder_IsValid(const Encoder_t *enc);

float Encoder_GetMechanicalAngle(const Encoder_t *enc);
float Encoder_GetElectricalAngle(const Encoder_t *enc);
float Encoder_GetMechanicalVelocity(const Encoder_t *enc);
float Encoder_GetMechanicalRPM(const Encoder_t *enc);
int64_t Encoder_GetTotalCount(const Encoder_t *enc);







#endif /* __ENCODER_H__ */
