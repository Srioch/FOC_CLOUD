#include "foc.h"

#include <math.h>

#define FOC_ONE_OVER_SQRT3 0.5773502691896258f

float angle_deadzone = 0.0f;
float default_control_x = 160.0f;
float default_control_y = 120.0f;

static float foc_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float foc_clamp_abs(float value, float limit)
{
    float abs_limit = fabsf(limit);

    if (abs_limit <= 0.0f)
    {
        abs_limit = VOLTAGE_LIMIT;
    }

    return foc_clampf(value, -abs_limit, abs_limit);
}

static void foc_limit_voltage_vector(float *ud_v, float *uq_v, float limit_v)
{
    float abs_limit = fabsf(limit_v);
    float mag_sq = 0.0f;
    float limit_sq = 0.0f;
    float scale = 0.0f;

    if ((ud_v == NULL) || (uq_v == NULL))
    {
        return;
    }

    if (abs_limit <= 0.0f)
    {
        abs_limit = VOLTAGE_LIMIT;
    }

    mag_sq = (*ud_v * *ud_v) + (*uq_v * *uq_v);
    limit_sq = abs_limit * abs_limit;
    if ((mag_sq <= limit_sq) || (mag_sq <= 0.0f))
    {
        return;
    }

    scale = abs_limit / sqrtf(mag_sq);
    *ud_v *= scale;
    *uq_v *= scale;
}

static uint32_t foc_voltage_to_compare(float voltage_v, uint32_t timer_top)
{
    float scaled = 0.0f;

    voltage_v = foc_clampf(voltage_v, 0.0f, VOLTAGE_LIMIT);
    scaled = (voltage_v / VOLTAGE_LIMIT) * (float)timer_top;

    if (scaled <= 0.0f)
    {
        return 0U;
    }
    if (scaled >= (float)timer_top)
    {
        return timer_top;
    }

    return (uint32_t)scaled;
}

static void foc_reset_pid(PID_t *pid)
{
    if (pid != NULL)
    {
        PID_Reset(pid);
    }
}

static void foc_update_measurements(FocMotor_t *motor)
{
    motor->state.mechanical_angle_rad = Encoder_GetMechanicalAngle(motor->enc);
    motor->state.electrical_angle_rad = Encoder_GetElectricalAngle(motor->enc);
    motor->state.mechanical_speed_rad_s = Encoder_GetMechanicalVelocity(motor->enc);
}

static void foc_disable_output(FocMotor_t *motor)
{
    disablePWM(motor->motor_id);
    motor->state.output_enabled = 0U;
    motor->state.speed_target_rad_s = 0.0f;
    motor->state.id_target_a = 0.0f;
    motor->state.iq_target_a = 0.0f;
    motor->state.id_measured_a = 0.0f;
    motor->state.iq_measured_a = 0.0f;
    motor->state.ud_command_v = 0.0f;
    motor->state.uq_command_v = 0.0f;
    motor->state.ud_applied_v = 0.0f;
    motor->state.uq_applied_v = 0.0f;
    motor->state.vision_error = 0.0f;
    motor->state.current_valid = 0U;
}

static void foc_apply_voltage_output(FocMotor_t *motor,
                                     float ud_command_v,
                                     float uq_command_v,
                                     uint8_t invert_for_closed_loop)
{
    float ud_applied_v = ud_command_v;
    float uq_applied_v = uq_command_v;

    motor->state.ud_command_v = ud_command_v;
    motor->state.uq_command_v = uq_command_v;
    if (invert_for_closed_loop != 0U)
    {
        ud_applied_v = -ud_applied_v;
        uq_applied_v = -uq_applied_v;
    }

    foc_limit_voltage_vector(&ud_applied_v, &uq_applied_v, motor->uq_limit_v);

    motor->state.ud_applied_v = ud_applied_v;
    motor->state.uq_applied_v = uq_applied_v;
    setPhaseVoltage(uq_applied_v, ud_applied_v, motor->state.electrical_angle_rad, motor->motor_id);
    motor->state.output_enabled = 1U;
}

static void foc_apply_current_output(FocMotor_t *motor,
                                     float id_target_a,
                                     float iq_target_a,
                                     uint8_t invert_for_closed_loop)
{
    float ud_command_v = 0.0f;
    float uq_command_v = 0.0f;

    if (invert_for_closed_loop != 0U)
    {
        iq_target_a = -iq_target_a;
    }

    motor->state.id_target_a = foc_clamp_abs(id_target_a, motor->iq_limit_a);
    motor->state.iq_target_a = foc_clamp_abs(iq_target_a, motor->iq_limit_a);

    if ((motor->pid_current_d == NULL) ||
        (motor->pid_current_q == NULL) ||
        (motor->dq_current.valid == 0U))
    {
        foc_reset_pid(motor->pid_current_d);
        foc_reset_pid(motor->pid_current_q);
        foc_disable_output(motor);
        return;
    }

    ud_command_v = PID_Update(motor->pid_current_d,
                              motor->state.id_target_a,
                              motor->dq_current.id_a);
    uq_command_v = PID_Update(motor->pid_current_q,
                              motor->state.iq_target_a,
                              motor->dq_current.iq_a);
    foc_apply_voltage_output(motor, ud_command_v, uq_command_v, 0U);
}

float Foc_GetElectricalAngle(float mechanical_angle_rad)
{
    return mechanical_angle_rad * FOC_POLE_PAIRS;
}

float Foc_NormalizeAngle(float angle_rad)
{
    float two_pi = 2.0f * M_PI;
    float normalized = fmodf(angle_rad, two_pi);

    return (normalized < 0.0f) ? (normalized + two_pi) : normalized;
}

void Foc_TransformPhaseCurrentToDq(const FocPhaseCurrent_t *phase_current,
                                   float electrical_angle_rad,
                                   FocDqCurrent_t *dq_current)
{
    float ialpha = 0.0f;
    float ibeta = 0.0f;
    float sin_angle = 0.0f;
    float cos_angle = 0.0f;

    if (dq_current == NULL)
    {
        return;
    }

    dq_current->id_a = 0.0f;
    dq_current->iq_a = 0.0f;
    dq_current->valid = 0U;

    if ((phase_current == NULL) || (phase_current->valid == 0U))
    {
        return;
    }

    ialpha = phase_current->ia_a;
    ibeta = (phase_current->ia_a + (2.0f * phase_current->ib_a)) * FOC_ONE_OVER_SQRT3;

    electrical_angle_rad = Foc_NormalizeAngle(electrical_angle_rad);
    sin_angle = sinf(electrical_angle_rad);
    cos_angle = cosf(electrical_angle_rad);

    dq_current->id_a = (ialpha * cos_angle) + (ibeta * sin_angle);
    dq_current->iq_a = (-ialpha * sin_angle) + (ibeta * cos_angle);
    dq_current->valid = 1U;
}

void setPWM(float Ua, float Ub, float Uc, uint8_t motor_id)
{
    uint32_t timer_top = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t pwm1 = foc_voltage_to_compare(Ua, timer_top);
    uint32_t pwm2 = foc_voltage_to_compare(Ub, timer_top);
    uint32_t pwm3 = foc_voltage_to_compare(Uc, timer_top);

    if (motor_id == MOTOR_UP)
    {
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_1, pwm1);
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_2, pwm2);
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_3, pwm3);
    }
    else if (motor_id == MOTOR_DOWN)
    {
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_1, pwm1);
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_2, pwm2);
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_3, pwm3);
    }
}

void disablePWM(uint8_t motor_id)
{
    if (motor_id == MOTOR_UP)
    {
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_3, 0U);
    }
    else if (motor_id == MOTOR_DOWN)
    {
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_3, 0U);
    }
}

void disableAllPWM(void)
{
    disablePWM(MOTOR_UP);
    disablePWM(MOTOR_DOWN);
}

void setPhaseVoltage(float Uq, float Ud, float electrical_angle_rad, uint8_t motor_id)
{
    float ualpha = 0.0f;
    float ubeta = 0.0f;
    float v_center = 0.0f;
    float ua = 0.0f;
    float ub = 0.0f;
    float uc = 0.0f;

    electrical_angle_rad = Foc_NormalizeAngle(electrical_angle_rad + angle_deadzone);

    ualpha = (Ud * cosf(electrical_angle_rad)) - (Uq * sinf(electrical_angle_rad));
    ubeta = (Ud * sinf(electrical_angle_rad)) + (Uq * cosf(electrical_angle_rad));

    v_center = VOLTAGE_LIMIT * 0.5f;
    ua = ualpha + v_center;
    ub = -0.5f * ualpha + (sqrtf(3.0f) * 0.5f) * ubeta + v_center;
    uc = -0.5f * ualpha - (sqrtf(3.0f) * 0.5f) * ubeta + v_center;

    setPWM(ua, ub, uc, motor_id);
}

void FocMotor_Init(FocMotor_t *motor,
                   Encoder_t *enc,
                   uint8_t motor_id,
                   PID_t *pid_position,
                   PID_t *pid_speed,
                   PID_t *pid_vision,
                   PID_t *pid_current_d,
                   PID_t *pid_current_q,
                   float dt_s,
                   float speed_limit_rad_s,
                   float iq_limit_a,
                   float uq_limit_v)
{
    if (motor == NULL)
    {
        return;
    }

    motor->enc = enc;
    motor->motor_id = motor_id;
    motor->pid_position = pid_position;
    motor->pid_speed = pid_speed;
    motor->pid_vision = pid_vision;
    motor->pid_current_d = pid_current_d;
    motor->pid_current_q = pid_current_q;
    motor->dt_s = (dt_s > 0.0f) ? dt_s : PID_DEFAULT_DT_S;
    motor->speed_limit_rad_s = fabsf(speed_limit_rad_s);
    if (motor->speed_limit_rad_s <= 0.0f)
    {
        motor->speed_limit_rad_s = PID_ANGLE_OUT_LIMIT_DEFAULT;
    }
    motor->iq_limit_a = fabsf(iq_limit_a);
    if (motor->iq_limit_a <= 0.0f)
    {
        motor->iq_limit_a = -PID_DEFAULT_OUT_MIN;
    }
    motor->uq_limit_v = fabsf(uq_limit_v);
    if (motor->uq_limit_v <= 0.0f)
    {
        motor->uq_limit_v = VOLTAGE_LIMIT;
    }
    motor->position_iq_direction = -1;
    motor->gravity_gain_a = 0.0f;
    motor->gravity_offset_rad = 0.0f;

    motor->mode = FOC_MODE_DISABLED;
    motor->target_position_rad = 0.0f;
    motor->target_speed_rad_s = 0.0f;
    motor->target_iq_a = 0.0f;
    motor->vision.measure = 0.0f;
    motor->vision.center = 0.0f;
    motor->vision.valid = 0U;
    motor->phase_current.ia_a = 0.0f;
    motor->phase_current.ib_a = 0.0f;
    motor->phase_current.ic_a = 0.0f;
    motor->phase_current.valid = 0U;
    motor->dq_current.id_a = 0.0f;
    motor->dq_current.iq_a = 0.0f;
    motor->dq_current.valid = 0U;
    motor->state.mechanical_angle_rad = 0.0f;
    motor->state.electrical_angle_rad = 0.0f;
    motor->state.mechanical_speed_rad_s = 0.0f;
    motor->state.speed_target_rad_s = 0.0f;
    motor->state.id_target_a = 0.0f;
    motor->state.iq_target_a = 0.0f;
    motor->state.id_measured_a = 0.0f;
    motor->state.iq_measured_a = 0.0f;
    motor->state.ud_command_v = 0.0f;
    motor->state.uq_command_v = 0.0f;
    motor->state.ud_applied_v = 0.0f;
    motor->state.uq_applied_v = 0.0f;
    motor->state.vision_error = 0.0f;
    motor->state.current_valid = 0U;
    motor->state.output_enabled = 0U;

    motor->open_loop.speed_rad_s = 0.0f;
    motor->open_loop.uq_v = 0.0f;
    motor->open_loop.angle_rad = 0.0f;

    if (motor->pid_position != NULL)
    {
        PID_SetDt(motor->pid_position, motor->dt_s);
        PID_SetOutputLimit(motor->pid_position, -motor->speed_limit_rad_s, motor->speed_limit_rad_s);
        PID_Reset(motor->pid_position);
    }
    if (motor->pid_speed != NULL)
    {
        PID_SetDt(motor->pid_speed, motor->dt_s);
        PID_SetOutputLimit(motor->pid_speed, -motor->iq_limit_a, motor->iq_limit_a);
        PID_Reset(motor->pid_speed);
    }
    if (motor->pid_vision != NULL)
    {
        PID_SetDt(motor->pid_vision, motor->dt_s);
        PID_SetOutputLimit(motor->pid_vision, -motor->speed_limit_rad_s, motor->speed_limit_rad_s);
        PID_Reset(motor->pid_vision);
    }
    if (motor->pid_current_d != NULL)
    {
        PID_SetDt(motor->pid_current_d, motor->dt_s);
        PID_SetOutputLimit(motor->pid_current_d, -motor->uq_limit_v, motor->uq_limit_v);
        PID_Reset(motor->pid_current_d);
    }
    if (motor->pid_current_q != NULL)
    {
        PID_SetDt(motor->pid_current_q, motor->dt_s);
        PID_SetOutputLimit(motor->pid_current_q, -motor->uq_limit_v, motor->uq_limit_v);
        PID_Reset(motor->pid_current_q);
    }
}

void FocMotor_Reset(FocMotor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    foc_reset_pid(motor->pid_position);
    foc_reset_pid(motor->pid_speed);
    foc_reset_pid(motor->pid_vision);
    foc_reset_pid(motor->pid_current_d);
    foc_reset_pid(motor->pid_current_q);
    motor->mode = FOC_MODE_DISABLED;
    motor->target_position_rad = 0.0f;
    motor->target_speed_rad_s = 0.0f;
    motor->target_iq_a = 0.0f;
    motor->vision.measure = 0.0f;
    motor->vision.center = 0.0f;
    motor->vision.valid = 0U;
    motor->open_loop.speed_rad_s = 0.0f;
    motor->open_loop.uq_v = 0.0f;
    motor->open_loop.angle_rad = 0.0f;
    foc_disable_output(motor);
}

void FocMotor_SetDisabled(FocMotor_t *motor)
{
    FocMotor_Reset(motor);
}

void FocMotor_SetTorque(FocMotor_t *motor, float iq_a)
{
    if (motor == NULL)
    {
        return;
    }

    motor->mode = FOC_MODE_TORQUE;
    motor->target_iq_a = iq_a;
}

void FocMotor_SetSpeed(FocMotor_t *motor, float speed_rad_s)
{
    if (motor == NULL)
    {
        return;
    }

    motor->mode = FOC_MODE_SPEED;
    motor->target_speed_rad_s = speed_rad_s;
}

void FocMotor_SetPosition(FocMotor_t *motor, float position_rad)
{
    if (motor == NULL)
    {
        return;
    }

    motor->mode = FOC_MODE_POSITION;
    motor->target_position_rad = position_rad;
}

void FocMotor_SetVision(FocMotor_t *motor, const FocVisionCommand_t *command)
{
    if ((motor == NULL) || (command == NULL))
    {
        return;
    }

    motor->mode = FOC_MODE_VISION;
    motor->vision = *command;
}

void FocMotor_SetOpenLoop(FocMotor_t *motor, float speed_rad_s, float uq_v)
{
    if (motor == NULL)
    {
        return;
    }

    motor->mode = FOC_MODE_OPEN_LOOP;
    motor->open_loop.speed_rad_s = speed_rad_s;
    motor->open_loop.uq_v = uq_v;
}

void FocMotor_SetPositionIqDirection(FocMotor_t *motor, int8_t direction)
{
    if (motor == NULL)
    {
        return;
    }

    motor->position_iq_direction = (direction < 0) ? -1 : 1;
}

void FocMotor_SetPhaseCurrent(FocMotor_t *motor, const FocPhaseCurrent_t *phase_current)
{
    if (motor == NULL)
    {
        return;
    }

    if (phase_current == NULL)
    {
        motor->phase_current.ia_a = 0.0f;
        motor->phase_current.ib_a = 0.0f;
        motor->phase_current.ic_a = 0.0f;
        motor->phase_current.valid = 0U;
        return;
    }

    motor->phase_current = *phase_current;
}

void FocMotor_Tick(FocMotor_t *motor)
{
    float iq_command_a = 0.0f;
    HAL_StatusTypeDef encoder_status = HAL_OK;

    if ((motor == NULL) || (motor->enc == NULL) || (motor->enc->is_started == 0U))
    {
        return;
    }

    /* Open-loop mode: synthetic angle, no feedback */
    if (motor->mode == FOC_MODE_OPEN_LOOP)
    {
        motor->open_loop.angle_rad += motor->open_loop.speed_rad_s * FOC_POLE_PAIRS * motor->dt_s;
        motor->open_loop.angle_rad = Foc_NormalizeAngle(motor->open_loop.angle_rad);

        motor->state.speed_target_rad_s = motor->open_loop.speed_rad_s;
        motor->state.id_target_a = 0.0f;
        motor->state.iq_target_a = 0.0f;
        motor->state.id_measured_a = 0.0f;
        motor->state.iq_measured_a = 0.0f;
        motor->state.ud_command_v = 0.0f;
        motor->state.uq_command_v = motor->open_loop.uq_v;
        motor->state.ud_applied_v = 0.0f;
        motor->state.uq_applied_v = motor->open_loop.uq_v;
        motor->state.vision_error = 0.0f;
        motor->state.current_valid = 0U;

        setPhaseVoltage(motor->open_loop.uq_v, 0.0f, motor->open_loop.angle_rad, motor->motor_id);
        motor->state.output_enabled = 1U;
        return;
    }

    encoder_status = Encoder_Update(motor->enc, motor->dt_s);
    foc_update_measurements(motor);

    if (motor->mode == FOC_MODE_DISABLED)
    {
        FocMotor_Reset(motor);
        return;
    }

    if ((encoder_status != HAL_OK) &&
        (Encoder_GetStatus(motor->enc).consecutive_failures >= ENCODER_I2C_FAIL_RECOVER_THRESHOLD))
    {
        FocMotor_Reset(motor);
        return;
    }

    Foc_TransformPhaseCurrentToDq(&motor->phase_current,
                                  motor->state.electrical_angle_rad,
                                  &motor->dq_current);
    motor->state.id_measured_a = motor->dq_current.id_a;
    motor->state.iq_measured_a = motor->dq_current.iq_a;
    motor->state.current_valid = motor->dq_current.valid;
    if (motor->dq_current.valid == 0U)
    {
        foc_reset_pid(motor->pid_speed);
        foc_reset_pid(motor->pid_current_d);
        foc_reset_pid(motor->pid_current_q);
        foc_disable_output(motor);
        return;
    }

    motor->state.speed_target_rad_s = 0.0f;
    motor->state.vision_error = 0.0f;

    switch (motor->mode)
    {
    case FOC_MODE_TORQUE:
        foc_apply_current_output(motor, 0.0f, motor->target_iq_a, 0U);
        break;

    case FOC_MODE_SPEED:
        if (motor->pid_speed == NULL)
        {
            FocMotor_Reset(motor);
            return;
        }
        motor->state.speed_target_rad_s = foc_clamp_abs(motor->target_speed_rad_s, motor->speed_limit_rad_s);
        iq_command_a = PID_Update(motor->pid_speed,
                                  motor->state.speed_target_rad_s,
                                  motor->state.mechanical_speed_rad_s);
        foc_apply_current_output(motor, 0.0f, iq_command_a, 0U);
        break;

    case FOC_MODE_POSITION:
        if ((motor->pid_position == NULL) || (motor->pid_speed == NULL))
        {
            FocMotor_Reset(motor);
            return;
        }
        motor->state.speed_target_rad_s = PID_UpdateAngleWrapped(motor->pid_position,
                                                                 motor->target_position_rad,
                                                                 motor->state.mechanical_angle_rad);
        motor->state.speed_target_rad_s = foc_clamp_abs(motor->state.speed_target_rad_s, motor->speed_limit_rad_s);
        iq_command_a = PID_Update(motor->pid_speed,
                                  motor->state.speed_target_rad_s,
                                  motor->state.mechanical_speed_rad_s);
        iq_command_a += motor->gravity_gain_a * sinf(motor->state.mechanical_angle_rad - motor->gravity_offset_rad);
        iq_command_a *= (float)motor->position_iq_direction;
        foc_apply_current_output(motor, 0.0f, iq_command_a, 0U);
        break;

    case FOC_MODE_VISION:
        if ((motor->pid_vision == NULL) || (motor->pid_speed == NULL) || (motor->vision.valid == 0U))
        {
            FocMotor_Reset(motor);
            return;
        }
        motor->state.vision_error = motor->vision.center - motor->vision.measure;
        if (fabsf(motor->state.vision_error) < 4.0f)
        {
            motor->state.vision_error = 0.0f;
        }
        motor->state.speed_target_rad_s = PID_Update(motor->pid_vision, 0.0f, motor->state.vision_error);
        motor->state.speed_target_rad_s = foc_clamp_abs(motor->state.speed_target_rad_s, motor->speed_limit_rad_s);
        iq_command_a = PID_Update(motor->pid_speed,
                                  motor->state.speed_target_rad_s,
                                  motor->state.mechanical_speed_rad_s);
        iq_command_a += motor->gravity_gain_a * sinf(motor->state.mechanical_angle_rad - motor->gravity_offset_rad);
        foc_apply_current_output(motor, 0.0f, iq_command_a, 0U);
        break;

    default:
        FocMotor_Reset(motor);
        break;
    }
}

const FocState_t *FocMotor_GetState(const FocMotor_t *motor)
{
    if (motor == NULL)
    {
        return NULL;
    }

    return &motor->state;
}

FocMode_t FocMotor_GetMode(const FocMotor_t *motor)
{
    if (motor == NULL)
    {
        return FOC_MODE_DISABLED;
    }

    return motor->mode;
}
