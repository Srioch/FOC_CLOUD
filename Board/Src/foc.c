#include "foc.h"

#include "math.h"

static float normalizeAngleSigned(float angle)
{
    float two_pi = 2.0f * M_PI;
    float a = fmodf(angle, two_pi);

    if (a > M_PI)
    {
        a -= two_pi;
    }
    else if (a < -M_PI)
    {
        a += two_pi;
    }

    return a;
}

static float foc_clamp_abs(float value, float limit)
{
    float abs_limit = fabsf(limit);

    if (abs_limit <= 0.0f)
    {
        abs_limit = VOLTAGE_LIMIT;
    }

    return _COMSTRAIN(value, -abs_limit, abs_limit);
}

static void foc_update_measurements(FocController_t *ctrl)
{
    ctrl->state.mechanical_angle_rad = Encoder_GetMechanicalAngle(ctrl->enc);
    ctrl->state.electrical_angle_rad = Encoder_GetElectricalAngle(ctrl->enc);
    ctrl->state.mechanical_speed_rad_s = Encoder_GetMechanicalVelocity(ctrl->enc);
}

static void foc_disable_output(FocController_t *ctrl)
{
    disablePWM(ctrl->motor_id);
    ctrl->state.output_enabled = 0U;
    ctrl->state.speed_target_rad_s = 0.0f;
    ctrl->state.uq_command_v = 0.0f;
    ctrl->state.uq_applied_v = 0.0f;
    ctrl->state.vision_error = 0.0f;
}

static void foc_reset_pid(PID_t *pid)
{
    if (pid != NULL)
    {
        PID_Reset(pid);
    }
}

static void foc_apply_output(FocController_t *ctrl, float uq_command_v, uint8_t invert_for_closed_loop)
{
    float uq_applied = foc_clamp_abs(uq_command_v, ctrl->uq_limit_v);

    ctrl->state.uq_command_v = uq_command_v;
    if (invert_for_closed_loop != 0U)
    {
        uq_applied = -uq_applied;
    }
    ctrl->state.uq_applied_v = uq_applied;
    torqueControl(uq_applied, ctrl->state.electrical_angle_rad, ctrl->motor_id);
    ctrl->state.output_enabled = 1U;
}

float GetelectricalAngle(float mechanicalAngle)
{
    return mechanicalAngle * FOC_POLE_PAIRS;
}

float nomalizeAngle(float angle)
{
    float two_pi = 2.0f * M_PI;
    float a = fmodf(angle, two_pi);
    return (a < 0.0f) ? (a + two_pi) : a;
}

void setPWM(float Ua, float Ub, float Uc, uint8_t motor_id)
{
    uint32_t top = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint16_t pwm1;
    uint16_t pwm2;
    uint16_t pwm3;

    Ua = _COMSTRAIN(Ua, 0.0f, VOLTAGE_LIMIT);
    Ub = _COMSTRAIN(Ub, 0.0f, VOLTAGE_LIMIT);
    Uc = _COMSTRAIN(Uc, 0.0f, VOLTAGE_LIMIT);

    pwm1 = _COMSTRAIN((uint16_t)((Ua / VOLTAGE_LIMIT) * (float)top), 0U, (uint16_t)top);
    pwm2 = _COMSTRAIN((uint16_t)((Ub / VOLTAGE_LIMIT) * (float)top), 0U, (uint16_t)top);
    pwm3 = _COMSTRAIN((uint16_t)((Uc / VOLTAGE_LIMIT) * (float)top), 0U, (uint16_t)top);

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

void setPhaseVoltage(float Uq, float Ud, float el_angle, uint8_t motor_id)
{
    float Ualpha;
    float Ubeta;
    float v_center;
    float Ua;
    float Ub;
    float Uc;

    (void)Ud;

    el_angle = nomalizeAngle(el_angle + ANGLE_DEADZONE);

    Ualpha = -Uq * sinf(el_angle);
    Ubeta = Uq * cosf(el_angle);

    v_center = VOLTAGE_LIMIT * 0.5f;
    Ua = Ualpha + v_center;
    Ub = -0.5f * Ualpha + (sqrtf(3.0f) / 2.0f) * Ubeta + v_center;
    Uc = -0.5f * Ualpha - (sqrtf(3.0f) / 2.0f) * Ubeta + v_center;

    setPWM(Ua, Ub, Uc, motor_id);
}

float velocityToVoltage(float velocity)
{
    float voltage = (velocity / 100.0f) * VOLTAGE_LIMIT;
    return _COMSTRAIN(voltage, 0.0f, VOLTAGE_LIMIT);
}

void openLoopSpeedControl(float target_rpm, float dt)
{
    static float mech_angle = 0.0f;
    float rev_per_sec = target_rpm / 60.0f;
    float mech_omega = rev_per_sec * 2.0f * M_PI;
    float electrical;
    float Uq;

    mech_angle += mech_omega * dt;
    mech_angle = nomalizeAngle(mech_angle);

    electrical = GetelectricalAngle(mech_angle);
    Uq = velocityToVoltage(target_rpm);
    setPhaseVoltage(Uq, 0.0f, electrical, MOTOR_UP);
}

float angleControl(float target_angle, float current_angle, float Kp)
{
    float error = normalizeAngleSigned(target_angle - current_angle);
    float uq = _COMSTRAIN((error * Kp), -VOLTAGE_LIMIT, VOLTAGE_LIMIT);

    setPhaseVoltage(uq, 0.0f, GetelectricalAngle(current_angle), MOTOR_UP);
    return uq;
}

float torqueControl(float Uq, float eleangle, uint8_t motor_id)
{
    float uq = _COMSTRAIN(Uq, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);

    setPhaseVoltage(uq, 0.0f, eleangle, motor_id);
    return uq;
}

void FocController_Init(FocController_t *ctrl,
                        Encoder_t *enc,
                        uint8_t motor_id,
                        PID_t *pid_position,
                        PID_t *pid_speed,
                        PID_t *pid_vision,
                        float dt_s,
                        float speed_limit_rad_s,
                        float uq_limit_v)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->enc = enc;
    ctrl->motor_id = motor_id;
    ctrl->pid_position = pid_position;
    ctrl->pid_speed = pid_speed;
    ctrl->pid_vision = pid_vision;
    ctrl->dt_s = (dt_s > 0.0f) ? dt_s : PID_DEFAULT_DT_S;
    ctrl->speed_limit_rad_s = fabsf(speed_limit_rad_s);
    if (ctrl->speed_limit_rad_s <= 0.0f)
    {
        ctrl->speed_limit_rad_s = PID_ANGLE_OUT_LIMIT_DEFAULT;
    }
    ctrl->uq_limit_v = fabsf(uq_limit_v);
    if (ctrl->uq_limit_v <= 0.0f)
    {
        ctrl->uq_limit_v = VOLTAGE_LIMIT;
    }
    ctrl->mode = FOC_MODE_DISABLED;
    ctrl->target_position_rad = 0.0f;
    ctrl->target_speed_rad_s = 0.0f;
    ctrl->target_uq_v = 0.0f;
    ctrl->vision_measure = 0.0f;
    ctrl->vision_center = 0.0f;
    ctrl->vision_valid = 0U;
    ctrl->state.mechanical_angle_rad = 0.0f;
    ctrl->state.electrical_angle_rad = 0.0f;
    ctrl->state.mechanical_speed_rad_s = 0.0f;
    ctrl->state.speed_target_rad_s = 0.0f;
    ctrl->state.uq_command_v = 0.0f;
    ctrl->state.uq_applied_v = 0.0f;
    ctrl->state.vision_error = 0.0f;
    ctrl->state.output_enabled = 0U;

    if (ctrl->pid_position != NULL)
    {
        PID_SetDt(ctrl->pid_position, ctrl->dt_s);
        PID_SetOutputLimit(ctrl->pid_position, -ctrl->speed_limit_rad_s, ctrl->speed_limit_rad_s);
        PID_Reset(ctrl->pid_position);
    }
    if (ctrl->pid_speed != NULL)
    {
        PID_SetDt(ctrl->pid_speed, ctrl->dt_s);
        PID_SetOutputLimit(ctrl->pid_speed, -ctrl->uq_limit_v, ctrl->uq_limit_v);
        PID_Reset(ctrl->pid_speed);
    }
    if (ctrl->pid_vision != NULL)
    {
        PID_SetDt(ctrl->pid_vision, ctrl->dt_s);
        PID_SetOutputLimit(ctrl->pid_vision, -ctrl->speed_limit_rad_s, ctrl->speed_limit_rad_s);
        PID_Reset(ctrl->pid_vision);
    }
}

void FocController_Reset(FocController_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    foc_reset_pid(ctrl->pid_position);
    foc_reset_pid(ctrl->pid_speed);
    foc_reset_pid(ctrl->pid_vision);
    ctrl->target_speed_rad_s = 0.0f;
    ctrl->target_uq_v = 0.0f;
    ctrl->vision_measure = 0.0f;
    ctrl->vision_center = 0.0f;
    ctrl->vision_valid = 0U;
    foc_disable_output(ctrl);
}

void runFoc(FocController_t *ctrl)
{
    if ((ctrl == NULL) || (ctrl->enc == NULL) || (ctrl->enc->is_started == 0U))
    {
        return;
    }

    Encoder_Update(ctrl->enc, ctrl->dt_s);
    foc_update_measurements(ctrl);

    if (ctrl->mode == FOC_MODE_DISABLED)
    {
        FocController_Reset(ctrl);
        return;
    }

    ctrl->state.speed_target_rad_s = 0.0f;
    ctrl->state.vision_error = 0.0f;

    switch (ctrl->mode)
    {
    case FOC_MODE_TORQUE:
        foc_apply_output(ctrl, ctrl->target_uq_v, 0U);
        break;

    case FOC_MODE_SPEED:
        if (ctrl->pid_speed == NULL)
        {
            FocController_Reset(ctrl);
            return;
        }
        ctrl->state.speed_target_rad_s = foc_clamp_abs(ctrl->target_speed_rad_s, ctrl->speed_limit_rad_s);
        foc_apply_output(ctrl,
                         PID_Update(ctrl->pid_speed,
                                    ctrl->state.speed_target_rad_s,
                                    ctrl->state.mechanical_speed_rad_s),
                         1U);
        break;

    case FOC_MODE_POSITION:
        if ((ctrl->pid_position == NULL) || (ctrl->pid_speed == NULL))
        {
            FocController_Reset(ctrl);
            return;
        }
        ctrl->state.speed_target_rad_s = PID_UpdateAngleWrapped(ctrl->pid_position,
                                                                ctrl->target_position_rad,
                                                                ctrl->state.mechanical_angle_rad);
        ctrl->state.speed_target_rad_s = foc_clamp_abs(ctrl->state.speed_target_rad_s, ctrl->speed_limit_rad_s);
        foc_apply_output(ctrl,
                         PID_Update(ctrl->pid_speed,
                                    ctrl->state.speed_target_rad_s,
                                    ctrl->state.mechanical_speed_rad_s),
                         1U);
        break;

    case FOC_MODE_VISION:
        if ((ctrl->pid_vision == NULL) || (ctrl->pid_speed == NULL) || (ctrl->vision_valid == 0U))
        {
            FocController_Reset(ctrl);
            return;
        }
        ctrl->state.vision_error = ctrl->vision_center - ctrl->vision_measure;
        if (fabsf(ctrl->state.vision_error) < 4.0f)
        {
            ctrl->state.vision_error = 0.0f;
        }
        ctrl->state.speed_target_rad_s = PID_Update(ctrl->pid_vision, 0.0f, -ctrl->state.vision_error);
        ctrl->state.speed_target_rad_s = foc_clamp_abs(ctrl->state.speed_target_rad_s, ctrl->speed_limit_rad_s);
        foc_apply_output(ctrl,
                         PID_Update(ctrl->pid_speed,
                                    ctrl->state.speed_target_rad_s,
                                    ctrl->state.mechanical_speed_rad_s),
                         1U);
        break;

    default:
        FocController_Reset(ctrl);
        break;
    }
}

void runFocVision(FocController_t *ctrl, const FocVisionCommand_t *cmd)
{
    if ((ctrl == NULL) || (cmd == NULL))
    {
        return;
    }

    ctrl->vision_measure = cmd->measure;
    ctrl->vision_center = cmd->center;
    ctrl->vision_valid = cmd->valid;
    ctrl->mode = FOC_MODE_VISION;
    runFoc(ctrl);
}

void SpeedLoop_Update(Encoder_t *enc, kalman_filter_pos_speed_t *kf, float dt_s, float speed_ref)
{
    static FocController_t speed_ctrl;
    static uint8_t initialized = 0U;

    (void)kf;

    if (initialized == 0U)
    {
        FocController_Init(&speed_ctrl,
                           enc,
                           MOTOR_UP,
                           NULL,
                           &pid_speed,
                           NULL,
                           dt_s,
                           PID_ANGLE_OUT_LIMIT_DEFAULT,
                           VOLTAGE_LIMIT);
        initialized = 1U;
    }

    speed_ctrl.enc = enc;
    speed_ctrl.dt_s = dt_s;
    speed_ctrl.mode = FOC_MODE_SPEED;
    speed_ctrl.target_speed_rad_s = speed_ref;
    runFoc(&speed_ctrl);
}

float Cloud_Control(uint16_t x, uint16_t y, uint8_t find)
{
    float temp = 0.0f;

    if (find != 0U)
    {
        Moto_Control(x, &pid_cloud_x, DEFAULT_CONTORL_X, &encoder_up, MOTOR_UP);
        temp = Moto_Control(y, &pid_cloud_y, DEFAULT_CONTORL_Y, &encoder_down, MOTOR_DOWN);
    }

    return temp;
}

float Moto_Control(uint16_t measure, PID_t *pid, uint16_t target, Encoder_t *enc, uint8_t motor_id)
{
    (void)_COMSTRAIN((uint16_t)((target - measure) * DEFAULT_KP), 0U, 100U);
    return torqueControl(_COMSTRAIN(PID_Update(pid, (float)target, (float)measure), -VOLTAGE_LIMIT, VOLTAGE_LIMIT),
                         Encoder_GetElectricalAngle(enc),
                         motor_id);
}

void VisionControl_Init(VisionControl_t *vc, PID_t *pid_speed, kalman_filter_pos_speed_t *kf_speed, Encoder_t *enc, uint8_t motor_id)
{
    if ((vc == NULL) || (pid_speed == NULL) || (kf_speed == NULL))
    {
        return;
    }

    vc->pid_speed = *pid_speed;
    vc->kf_speed = *kf_speed;
    vc->enc = enc;
    vc->motor_id = motor_id;
    vc->speed_ref = 0.0f;
    vc->uq = 0.0f;
}

void VisionOuterLoop(VisionControl_t *vc, float measure, float center, uint8_t valid, float dt)
{
    float err;

    (void)dt;

    if (vc == NULL)
    {
        return;
    }

    if (valid == 0U)
    {
        vc->speed_ref = 0.0f;
        PID_Reset(&vc->pid_vis);
        return;
    }

    err = center - measure;
    if (fabsf(err) < 4.0f)
    {
        err = 0.0f;
    }

    vc->speed_ref = PID_Update(&vc->pid_vis, 0.0f, -err);
    vc->speed_ref = _COMSTRAIN(vc->speed_ref, -20.0f, 20.0f);
}

float MotorSpeedLoop_Update(VisionControl_t *vc, float dt_s)
{
    if ((vc == NULL) || (vc->enc == NULL))
    {
        return 0.0f;
    }

    Encoder_Update(vc->enc, dt_s);
    vc->uq = PID_Update(&vc->pid_speed, vc->speed_ref, Encoder_GetMechanicalVelocity(vc->enc));
    return torqueControl(-vc->uq, Encoder_GetElectricalAngle(vc->enc), vc->motor_id);
}
