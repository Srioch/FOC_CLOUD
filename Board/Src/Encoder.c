#include "Encoder.h"

#include <math.h>

#define ENCODER_TWO_PI 6.28318530718f
#define AS5600_REG_RAW_ANGLE_H 0x0CU
#define AS5600_RAW_MASK 0x0FFFU
#define AS5600_I2C_TIMEOUT_MS 2U
#define ENCODER_I2C_RECOVERY_CLOCKS 9U
#define ENCODER_I2C_RECOVERY_DELAY_US 5U

float target_angle = 0.0f;
Encoder_t encoder_up;
Encoder_t encoder_down;

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

static void encoder_delay_us(uint32_t delay_us)
{
	volatile uint32_t cycles = delay_us * 20U;

	while (cycles-- > 0U)
	{
		__NOP();
	}
}

static HAL_StatusTypeDef encoder_get_i2c_pins(const I2C_HandleTypeDef *hi2c,
                                              GPIO_TypeDef **port,
                                              uint16_t *scl_pin,
                                              uint16_t *sda_pin,
                                              uint32_t *alternate)
{
	if ((hi2c == NULL) || (port == NULL) || (scl_pin == NULL) || (sda_pin == NULL) || (alternate == NULL))
	{
		return HAL_ERROR;
	}

	if (hi2c->Instance == I2C1)
	{
		*port = GPIOB;
		*scl_pin = GPIO_PIN_6;
		*sda_pin = GPIO_PIN_7;
		*alternate = GPIO_AF4_I2C1;
		return HAL_OK;
	}

	if (hi2c->Instance == I2C2)
	{
		*port = GPIOB;
		*scl_pin = GPIO_PIN_10;
		*sda_pin = GPIO_PIN_11;
		*alternate = GPIO_AF4_I2C2;
		return HAL_OK;
	}

	return HAL_ERROR;
}

static void encoder_restore_i2c_gpio(I2C_HandleTypeDef *hi2c)
{
	GPIO_TypeDef *port = NULL;
	uint16_t scl_pin = 0U;
	uint16_t sda_pin = 0U;
	uint32_t alternate = 0U;
	GPIO_InitTypeDef gpio = {0};

	if (encoder_get_i2c_pins(hi2c, &port, &scl_pin, &sda_pin, &alternate) != HAL_OK)
	{
		return;
	}

	gpio.Pin = scl_pin | sda_pin;
	gpio.Mode = GPIO_MODE_AF_OD;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Alternate = alternate;
	HAL_GPIO_Init(port, &gpio);
}

static HAL_StatusTypeDef encoder_recover_i2c_bus(Encoder_t *enc)
{
	GPIO_TypeDef *port = NULL;
	uint16_t scl_pin = 0U;
	uint16_t sda_pin = 0U;
	uint32_t alternate = 0U;
	GPIO_InitTypeDef gpio = {0};
	uint8_t i = 0U;

	if ((enc == NULL) || (enc->hi2c == NULL))
	{
		return HAL_ERROR;
	}

	if (encoder_get_i2c_pins(enc->hi2c, &port, &scl_pin, &sda_pin, &alternate) != HAL_OK)
	{
		return HAL_ERROR;
	}

	HAL_I2C_DeInit(enc->hi2c);

	gpio.Pin = scl_pin | sda_pin;
	gpio.Mode = GPIO_MODE_OUTPUT_OD;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(port, &gpio);

	HAL_GPIO_WritePin(port, scl_pin | sda_pin, GPIO_PIN_SET);
	encoder_delay_us(ENCODER_I2C_RECOVERY_DELAY_US);

	for (i = 0U; i < ENCODER_I2C_RECOVERY_CLOCKS; i++)
	{
		if (HAL_GPIO_ReadPin(port, sda_pin) == GPIO_PIN_SET)
		{
			break;
		}
		HAL_GPIO_WritePin(port, scl_pin, GPIO_PIN_RESET);
		encoder_delay_us(ENCODER_I2C_RECOVERY_DELAY_US);
		HAL_GPIO_WritePin(port, scl_pin, GPIO_PIN_SET);
		encoder_delay_us(ENCODER_I2C_RECOVERY_DELAY_US);
	}

	HAL_GPIO_WritePin(port, sda_pin, GPIO_PIN_RESET);
	encoder_delay_us(ENCODER_I2C_RECOVERY_DELAY_US);
	HAL_GPIO_WritePin(port, scl_pin, GPIO_PIN_SET);
	encoder_delay_us(ENCODER_I2C_RECOVERY_DELAY_US);
	HAL_GPIO_WritePin(port, sda_pin, GPIO_PIN_SET);
	encoder_delay_us(ENCODER_I2C_RECOVERY_DELAY_US);

	enc->status.recovery_count++;

	if (HAL_I2C_Init(enc->hi2c) != HAL_OK)
	{
		return HAL_ERROR;
	}

	encoder_restore_i2c_gpio(enc->hi2c);
	return HAL_OK;
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

static void encoder_reset_status(Encoder_t *enc)
{
	enc->status.last_status = HAL_OK;
	enc->status.error_count = 0U;
	enc->status.consecutive_failures = 0U;
	enc->status.last_raw_angle = 0U;
	enc->status.valid = 0U;
	enc->status.recovery_count = 0U;
}

static void encoder_clear_diagnostics_only(Encoder_t *enc)
{
	uint8_t valid = enc->status.valid;
	uint16_t last_raw_angle = enc->status.last_raw_angle;

	enc->status.last_status = HAL_OK;
	enc->status.error_count = 0U;
	enc->status.consecutive_failures = 0U;
	enc->status.last_raw_angle = last_raw_angle;
	enc->status.valid = valid;
	enc->status.recovery_count = 0U;
}

static HAL_StatusTypeDef encoder_note_read_result(Encoder_t *enc, HAL_StatusTypeDef status, uint16_t raw_angle)
{
	if (enc == NULL)
	{
		return HAL_ERROR;
	}

	enc->status.last_status = status;
	if (status == HAL_OK)
	{
		enc->status.consecutive_failures = 0U;
		enc->status.last_raw_angle = raw_angle;
		enc->status.valid = 1U;
		return HAL_OK;
	}

	enc->status.error_count++;
	if (enc->status.consecutive_failures < UINT16_MAX)
	{
		enc->status.consecutive_failures++;
	}
	enc->status.valid = 0U;
	enc->mech_velocity_rad_s = 0.0f;
	enc->mech_velocity_rpm = 0.0f;

	if (enc->status.consecutive_failures >= ENCODER_I2C_FAIL_RECOVER_THRESHOLD)
	{
		(void)encoder_recover_i2c_bus(enc);
	}

	return status;
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
	encoder_reset_status(enc);

	uint16_t raw_angle = 0U;
	HAL_StatusTypeDef ret = encoder_read_raw_angle(enc, &raw_angle);
	if (ret == HAL_OK)
	{
		enc->last_count = encoder_raw_to_count(enc, raw_angle);
		(void)encoder_note_read_result(enc, HAL_OK, raw_angle);
	}
	else
	{
		enc->last_count = 0;
		(void)encoder_note_read_result(enc, ret, 0U);
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
		(void)encoder_note_read_result(enc, ret, 0U);
		return ret;
	}

	enc->last_count = encoder_raw_to_count(enc, raw_angle);
	(void)encoder_note_read_result(enc, HAL_OK, raw_angle);
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
		(void)encoder_note_read_result(enc, HAL_OK, raw_angle);
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

HAL_StatusTypeDef Encoder_Update(Encoder_t *enc, float dt_s)
{
	if ((enc == NULL) || (enc->hi2c == NULL) || (enc->cpr == 0U) || (enc->is_started == 0U) || (dt_s <= 0.0f))
	{
		return HAL_ERROR;
	}

	uint16_t raw_angle = 0U;
	HAL_StatusTypeDef ret = encoder_read_raw_angle(enc, &raw_angle);
	if (ret != HAL_OK)
	{
		return encoder_note_read_result(enc, ret, 0U);
	}
	(void)encoder_note_read_result(enc, HAL_OK, raw_angle);

	int32_t now_count = encoder_raw_to_count(enc, raw_angle);
	int32_t delta = encoder_calc_delta(enc->last_count, now_count, (int32_t)enc->cpr) * ENCODER_SIGN;

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

	return HAL_OK;
}

HAL_StatusTypeDef Encoder_Updata(Encoder_t *enc)
{
	if ((enc == NULL) || (enc->hi2c == NULL) || (enc->cpr == 0U) || (enc->is_started == 0U))
	{
		return HAL_ERROR;
	}

	enc->time_now = HAL_GetTick();

	uint16_t raw_angle = 0U;
	HAL_StatusTypeDef ret = encoder_read_raw_angle(enc, &raw_angle);
	if (ret != HAL_OK)
	{
		return encoder_note_read_result(enc, ret, 0U);
	}
	(void)encoder_note_read_result(enc, HAL_OK, raw_angle);
	

	int32_t now_count = encoder_raw_to_count(enc, raw_angle);
	int32_t delta = encoder_calc_delta(enc->last_count, now_count, (int32_t)enc->cpr) * ENCODER_SIGN;

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

	return HAL_OK;
}

EncoderStatus_t Encoder_GetStatus(const Encoder_t *enc)
{
	EncoderStatus_t status = {0};

	if (enc == NULL)
	{
		status.last_status = HAL_ERROR;
		return status;
	}

	return enc->status;
}

void Encoder_ClearDiagnostics(Encoder_t *enc)
{
	if (enc == NULL)
	{
		return;
	}

	encoder_clear_diagnostics_only(enc);
}

uint8_t Encoder_IsValid(const Encoder_t *enc)
{
	return (enc == NULL) ? 0U : enc->status.valid;
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
