#include "Encoder.h"

#include <math.h>

#define ENCODER_TWO_PI 6.28318530718f
#define AS5600_REG_RAW_ANGLE_H 0x0CU
#define AS5600_RAW_MASK 0x0FFFU
#define AS5600_I2C_TIMEOUT_MS 100U

float target_angle = 0.0f;

static float encoder_normalize_angle(float angle)
{
	float a = fmodf(angle, ENCODER_TWO_PI);
	return (a < 0.0f) ? (a + ENCODER_TWO_PI) : a;
}

static int32_t encoder_calc_delta(int32_t last_count, int32_t now_count, int32_t modulo)
{
	int32_t delta = now_count - last_count;
	int32_t half_period = modulo / 2;

	if (delta > half_period)
	{
		delta -= modulo;
	}
	else if (delta < -half_period)
	{
		delta += modulo;
	}

	return delta;
}

static HAL_StatusTypeDef encoder_read_raw_angle(const Encoder_t *enc, uint16_t *raw_angle)
{
	if ((enc == NULL) || (enc->hi2c == NULL) || (raw_angle == NULL))
	{
		return HAL_ERROR;
	}

	uint8_t data[2] = {0U};
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(enc->hi2c,
	                                         (uint16_t)(enc->i2c_addr_7bit << 1),
	                                         AS5600_REG_RAW_ANGLE_H,
	                                         I2C_MEMADD_SIZE_8BIT,
	                                         data,
	                                         2U,
	                                         AS5600_I2C_TIMEOUT_MS);
	if (ret != HAL_OK)
	{
		return ret;
	}

	*raw_angle = (uint16_t)((((uint16_t)data[0]) << 8) | (uint16_t)data[1]);
	*raw_angle &= AS5600_RAW_MASK;

	return HAL_OK;
}

static int32_t encoder_raw_to_count(const Encoder_t *enc, uint16_t raw_angle)
{
	return (int32_t)(((uint32_t)raw_angle * enc->cpr) / AS5600_RESOLUTION);
}

void Encoder_Init(Encoder_t *enc, I2C_HandleTypeDef *hi2c, uint32_t cpr, float pole_pairs)
{
	if ((enc == NULL) || (hi2c == NULL))
	{
		return;
	}

	enc->hi2c = hi2c;
	enc->i2c_addr_7bit = AS5600_I2C_ADDR_7BIT;
	enc->cpr = (cpr > 0U) ? cpr : AS5600_RESOLUTION;
	enc->pole_pairs = (pole_pairs > 0.0f) ? pole_pairs : 1.0f;

	uint16_t raw_angle = 0U;
	if (encoder_read_raw_angle(enc, &raw_angle) == HAL_OK)
	{
		enc->last_count = encoder_raw_to_count(enc, raw_angle);
	}
	else
	{
		enc->last_count = 0;
	}

	enc->total_count = 0;
	enc->is_started = 0U;
	enc->mech_angle_rad = 0.0f;
	enc->elec_angle_rad = 0.0f;
	enc->mech_velocity_rad_s = 0.0f;
	enc->mech_velocity_rpm = 0.0f;
	enc->elec_zero_offset_rad = 0.0f;
	enc->time_prev = 0U;
	enc->time_now = 0U;
}

HAL_StatusTypeDef Encoder_Start(Encoder_t *enc)
{
	if ((enc == NULL) || (enc->hi2c == NULL) || (enc->cpr == 0U))
	{
		return HAL_ERROR;
	}

	uint16_t raw_angle = 0U;
	HAL_StatusTypeDef ret = encoder_read_raw_angle(enc, &raw_angle);
	if (ret != HAL_OK)
	{
		return ret;
	}

	enc->last_count = encoder_raw_to_count(enc, raw_angle);
	enc->total_count = 0;
	enc->mech_velocity_rad_s = 0.0f;
	enc->mech_velocity_rpm = 0.0f;
	enc->time_now = HAL_GetTick();
	enc->time_prev = enc->time_now;
	enc->is_started = 1U;

	return HAL_OK;
}

HAL_StatusTypeDef Encoder_Stop(Encoder_t *enc)
{
	if ((enc == NULL) || (enc->hi2c == NULL))
	{
		return HAL_ERROR;
	}

	enc->is_started = 0U;
	enc->mech_velocity_rad_s = 0.0f;
	enc->mech_velocity_rpm = 0.0f;

	return HAL_OK;
}

void Encoder_SetMechanicalZero(Encoder_t *enc)
{
	if ((enc == NULL) || (enc->hi2c == NULL))
	{
		return;
	}

	uint16_t raw_angle = 0U;
	if (encoder_read_raw_angle(enc, &raw_angle) == HAL_OK)
	{
		enc->last_count = encoder_raw_to_count(enc, raw_angle);
	}

	enc->total_count = 0;
	enc->mech_angle_rad = 0.0f;
	enc->elec_angle_rad = encoder_normalize_angle(-enc->elec_zero_offset_rad);
	enc->mech_velocity_rad_s = 0.0f;
	enc->mech_velocity_rpm = 0.0f;
}

void Encoder_SetElectricalZeroOffset(Encoder_t *enc, float offset_rad)
{
	if (enc == NULL)
	{
		return;
	}

	enc->elec_zero_offset_rad = encoder_normalize_angle(offset_rad);
}

void Encoder_Update(Encoder_t *enc, float dt_s)
{
	if ((enc == NULL) || (enc->hi2c == NULL) || (enc->cpr == 0U) || (enc->is_started == 0U) || (dt_s <= 0.0f))
	{
		return;
	}

	uint16_t raw_angle = 0U;
	if (encoder_read_raw_angle(enc, &raw_angle) != HAL_OK)
	{
		return;
	}

	int32_t now_count = encoder_raw_to_count(enc, raw_angle);
	int32_t delta = encoder_calc_delta(enc->last_count, now_count, (int32_t)enc->cpr);

	enc->last_count = now_count;
	enc->total_count += (int64_t)delta;

	int32_t mech_count = (int32_t)(enc->total_count % (int64_t)enc->cpr);
	if (mech_count < 0)
	{
		mech_count += (int32_t)enc->cpr;
	}

	enc->mech_angle_rad = ((float)mech_count / (float)enc->cpr) * ENCODER_TWO_PI;

	float elec = enc->mech_angle_rad * enc->pole_pairs - enc->elec_zero_offset_rad;
	enc->elec_angle_rad = encoder_normalize_angle(elec);

	enc->mech_velocity_rad_s = ((float)delta * ENCODER_TWO_PI) / ((float)enc->cpr * dt_s);
	enc->mech_velocity_rpm = ((float)delta * 60.0f) / ((float)enc->cpr * dt_s);
}

void Encoder_Updata(Encoder_t *enc)
{
	if ((enc == NULL) || (enc->hi2c == NULL) || (enc->cpr == 0U) || (enc->is_started == 0U))
	{
		return;
	}

	enc->time_now = HAL_GetTick();

	uint16_t raw_angle = 0U;
	if (encoder_read_raw_angle(enc, &raw_angle) != HAL_OK)
	{
		return;
	}
	

	int32_t now_count = encoder_raw_to_count(enc, raw_angle);
	int32_t delta = encoder_calc_delta(enc->last_count, now_count, (int32_t)enc->cpr);

	enc->last_count = now_count;
	enc->total_count += (int64_t)delta;

	int32_t mech_count = (int32_t)(enc->total_count % (int64_t)enc->cpr);
	if (mech_count < 0)
	{
		mech_count += (int32_t)enc->cpr;
	}

	enc->mech_angle_rad = ((float)mech_count / (float)enc->cpr) * ENCODER_TWO_PI;

	float elec = enc->mech_angle_rad * enc->pole_pairs - enc->elec_zero_offset_rad;
	enc->elec_angle_rad = encoder_normalize_angle(elec);

	uint32_t dt_ms = enc->time_now - enc->time_prev;
	if (dt_ms > 0U)
	{
		float dt_s = (float)dt_ms * 0.001f;
		enc->mech_velocity_rad_s = ((float)delta * ENCODER_TWO_PI) / ((float)enc->cpr * dt_s);
		enc->mech_velocity_rpm = ((float)delta * 60.0f) / ((float)enc->cpr * dt_s);
	}

	enc->time_prev = enc->time_now;
}

float Encoder_GetMechanicalAngle(const Encoder_t *enc)
{
	return (enc == NULL) ? 0.0f : enc->mech_angle_rad;
}

float Encoder_GetElectricalAngle(const Encoder_t *enc)
{
	return (enc == NULL) ? 0.0f : enc->elec_angle_rad;
}

float Encoder_GetMechanicalVelocity(const Encoder_t *enc)
{
	return (enc == NULL) ? 0.0f : enc->mech_velocity_rad_s;
}

float Encoder_GetMechanicalRPM(const Encoder_t *enc)
{
	return (enc == NULL) ? 0.0f : enc->mech_velocity_rpm;
}

int64_t Encoder_GetTotalCount(const Encoder_t *enc)
{
	return (enc == NULL) ? 0 : enc->total_count;
}
