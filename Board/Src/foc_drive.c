/**
 * @file foc_drive.c
 * @author 2743823168@qq.com
 * @brief FOC总接口实现
 * @version 0.1
 * @date 2026-06-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */



#include "foc_drive.h"

#include "MYADC.h"
#include "Encoder.h"
#include "adc.h"
#include "foc.h"
#include "i2c.h"
#include "pid.h"
#include "tim.h"
#include "uart.h"
#include "usart.h"

#include <math.h>
#include <stdio.h>

#define OFFSET_TRACK_ENABLE_SPEED_RAD_S 1.0f
#define ADC_STARTUP_SETTLE_DELAY_MS 20U
#define ADC_STARTUP_CALIB_SAMPLES ADC_FILTER_DEFAULT_CALIB_SAMPLES
#define FOC_LOOP_DT_S 0.001f
#define FOC_SPEED_LIMIT_RAD_S 20.0f
#define FOC_IQ_LIMIT_A 2.0f
#define FOC_UQ_LIMIT_V 6.0f
#define FOC_VISION_FRAME_TIMEOUT_MS 50U
#define FOC_UP_PHASE_A_ADC_CHANNEL ADC_CHANNEL_5
#define FOC_UP_PHASE_B_ADC_CHANNEL ADC_CHANNEL_6
#define FOC_DOWN_PHASE_A_ADC_CHANNEL ADC_CHANNEL_8
#define FOC_DOWN_PHASE_B_ADC_CHANNEL ADC_CHANNEL_9
#define FOC_PHASE_CURRENT_AMP_GAIN 20.0f
#define FOC_PHASE_CURRENT_SHUNT_RESISTANCE_OHM 0.1f
#define FOC_PHASE_CURRENT_ADC_OUTPUT_GAIN 1.0f
#define FOC_PHASE_CURRENT_SENSOR_POLARITY (-1.0f)
#define FOC_PHASE_CURRENT_MAX_MISSES 3U
#define FOC_PHASE_CURRENT_V_TO_A_DIVISOR \
    (FOC_PHASE_CURRENT_AMP_GAIN * FOC_PHASE_CURRENT_SHUNT_RESISTANCE_OHM)

#define FOC_UP_TIM htim2
#define FOC_DOWN_TIM htim3
#define FOC_UP_UPWM_CHANNEL TIM_CHANNEL_1
#define FOC_UP_VPWM_CHANNEL TIM_CHANNEL_2
#define FOC_UP_WPWM_CHANNEL TIM_CHANNEL_3
#define FOC_DOWN_UPWM_CHANNEL TIM_CHANNEL_2
#define FOC_DOWN_VPWM_CHANNEL TIM_CHANNEL_3
#define FOC_DOWN_WPWM_CHANNEL TIM_CHANNEL_4


/**
 * @brief FOC驱动模块实现
 * @param None
 */
typedef struct
{
    ADC_ChannelFilter_t phase_a_filter;
    ADC_ChannelFilter_t phase_b_filter;
    ADC_FilteredSample_t phase_a_sample;
    ADC_FilteredSample_t phase_b_sample;
    PhaseVoltageSample_t phase_sample;
    uint8_t offset_track_enabled;
    uint8_t sample_miss_count;
} FOC_DrivePhaseAdc_t;

static FOC_DrivePhaseAdc_t s_up_phase_adc;
static FOC_DrivePhaseAdc_t s_down_phase_adc;

static PID_t s_up_current_d_pid;
static PID_t s_up_current_q_pid;
static PID_t s_down_speed_pid;
static PID_t s_down_angle_pid;
static PID_t s_down_current_d_pid;
static PID_t s_down_current_q_pid;
static FocMotor_t s_up_motor;
static FocMotor_t s_down_motor;
static FocPhaseCurrent_t s_up_phase_current = {0};
static FocPhaseCurrent_t s_down_phase_current = {0};

static VisionData_t s_vision_frame = {0};
static uint32_t s_last_vision_tick = 0U;
static uint8_t s_initialized = 0U;
static uint16_t s_mode_snapshot = 0U;
static uint8_t s_vision_was_active = 0U;
static float s_up_hold_angle_rad = 0.0f;
static float s_down_hold_angle_rad = 0.0f;
static uint8_t s_open_loop_active = 0U;

float uq = 0.0f;
float speed_raw = 0.0f;
float speed_filt = 0.0f;
volatile float Ia = 0.0f;
volatile float Ib = 0.0f;
volatile float Ic = 0.0f;
volatile float Ia_down = 0.0f;
volatile float Ib_down = 0.0f;
volatile float Ic_down = 0.0f;

static float foc_drive_sense_voltage_to_current(float sense_voltage_v)
{
    return (FOC_PHASE_CURRENT_SENSOR_POLARITY * sense_voltage_v) / FOC_PHASE_CURRENT_V_TO_A_DIVISOR;
}

static void foc_drive_mark_phase_current_miss(FOC_DrivePhaseAdc_t *phase_adc,
                                              FocPhaseCurrent_t *phase_current)
{
    if ((phase_adc == NULL) || (phase_current == NULL))
    {
        return;
    }

    if (phase_adc->sample_miss_count < FOC_PHASE_CURRENT_MAX_MISSES)
    {
        phase_adc->sample_miss_count++;
    }

    if (phase_adc->sample_miss_count >= FOC_PHASE_CURRENT_MAX_MISSES)
    {
        phase_current->valid = 0U;
    }
}

/**
 * @brief 启动PWM输出
 * @param None
 * @warning 相关引脚宏定义在定义区更改
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef foc_drive_start_pwm(void)
{
    if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(&FOC_UP_TIM, FOC_UP_UPWM_CHANNEL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(&FOC_UP_TIM, FOC_UP_VPWM_CHANNEL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(&FOC_UP_TIM, FOC_UP_WPWM_CHANNEL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(&FOC_DOWN_TIM, FOC_DOWN_UPWM_CHANNEL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(&FOC_DOWN_TIM, FOC_DOWN_VPWM_CHANNEL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(&FOC_DOWN_TIM, FOC_DOWN_WPWM_CHANNEL) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}



/**
 * @brief 初始化相位电压ADC滤波器
 * 
 * @param phase_adc 
 * @param phase_a_channel 
 * @param phase_b_channel 
 */
static void foc_drive_init_phase_adc(FOC_DrivePhaseAdc_t *phase_adc,
                                     uint32_t phase_a_channel,
                                     uint32_t phase_b_channel)
{
    ADC_ChannelFilter_Init(&phase_adc->phase_a_filter, &hadc1, phase_a_channel, ADC_SAMPLETIME_144CYCLES,
                           ADC_FILTER_DEFAULT_WINDOW, ADC_FILTER_DEFAULT_ALPHA);
    ADC_ChannelFilter_Init(&phase_adc->phase_b_filter, &hadc1, phase_b_channel, ADC_SAMPLETIME_144CYCLES,
                           ADC_FILTER_DEFAULT_WINDOW, ADC_FILTER_DEFAULT_ALPHA);
    ADC_ChannelFilter_SetTracking(&phase_adc->phase_a_filter,
                                  ADC_FILTER_DEFAULT_OFFSET_ALPHA,
                                  ADC_FILTER_DEFAULT_TRACK_BAND_RAW,
                                  ADC_FILTER_DEFAULT_DRIFT_LIMIT_RAW,
                                  ADC_FILTER_DEFAULT_SATURATION_MARGIN_RAW);
    ADC_ChannelFilter_SetTracking(&phase_adc->phase_b_filter,
                                  ADC_FILTER_DEFAULT_OFFSET_ALPHA,
                                  ADC_FILTER_DEFAULT_TRACK_BAND_RAW,
                                  ADC_FILTER_DEFAULT_DRIFT_LIMIT_RAW,
                                  ADC_FILTER_DEFAULT_SATURATION_MARGIN_RAW);
    ADC_ChannelFilter_SetOutputGain(&phase_adc->phase_a_filter, FOC_PHASE_CURRENT_ADC_OUTPUT_GAIN);
    ADC_ChannelFilter_SetOutputGain(&phase_adc->phase_b_filter, FOC_PHASE_CURRENT_ADC_OUTPUT_GAIN);
    phase_adc->offset_track_enabled = 1U;
    phase_adc->sample_miss_count = FOC_PHASE_CURRENT_MAX_MISSES;
}

// 在FOC驱动初始化过程中，对ADC滤波器进行校准，确保相位电压采样的准确性
static void foc_drive_init_adc_filters(void)
{
    foc_drive_init_phase_adc(&s_up_phase_adc, FOC_UP_PHASE_A_ADC_CHANNEL, FOC_UP_PHASE_B_ADC_CHANNEL);
    foc_drive_init_phase_adc(&s_down_phase_adc, FOC_DOWN_PHASE_A_ADC_CHANNEL, FOC_DOWN_PHASE_B_ADC_CHANNEL);
}




/**
 * @brief 校准相位电压ADC滤波器
 * 
 * @param phase_adc 
 */
static void foc_drive_calibrate_phase_adc(FOC_DrivePhaseAdc_t *phase_adc)
{
    if (ADC_ChannelFilter_Calibrate(&phase_adc->phase_a_filter, ADC_STARTUP_CALIB_SAMPLES) != HAL_OK)
    {
        ADC_ChannelFilter_SeedOffsetVoltage(&phase_adc->phase_a_filter, ADC_PHASE_VOLTAGE_DEFAULT_OFFSET_V);
    }
    if (ADC_ChannelFilter_Calibrate(&phase_adc->phase_b_filter, ADC_STARTUP_CALIB_SAMPLES) != HAL_OK)
    {
        ADC_ChannelFilter_SeedOffsetVoltage(&phase_adc->phase_b_filter, ADC_PHASE_VOLTAGE_DEFAULT_OFFSET_V);
    }
}



/**
 * @brief 校准ADC滤波器
 * 
 */
static void foc_drive_calibrate_adc_filters(void)
{
    HAL_Delay(ADC_STARTUP_SETTLE_DELAY_MS);
    foc_drive_calibrate_phase_adc(&s_up_phase_adc);
    foc_drive_calibrate_phase_adc(&s_down_phase_adc);
}


/**
 * @brief 串口接收处理
 * 
 * @return HAL_StatusTypeDef 
 */
static HAL_StatusTypeDef foc_drive_start_uart(void)
{
    UART_SetHandle(&huart1);
    if (UART_StartReceiveIT(&huart1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (UART_StartReceiveIT(&huart2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}


/**
 * @brief 编码器启动
 * 
 * @return HAL_StatusTypeDef 
 */
static HAL_StatusTypeDef foc_drive_start_encoders(void)
{
    Encoder_Init(&encoder_up, &hi2c1, AS5600_RESOLUTION, FOC_POLE_PAIRS);
    HAL_Delay(300);
    Encoder_Init(&encoder_down, &hi2c2, AS5600_RESOLUTION, FOC_POLE_PAIRS);
    HAL_Delay(300);

    if (Encoder_Start(&encoder_up) != HAL_OK)
    {
        return HAL_ERROR;
    }
    HAL_Delay(300);
    if (Encoder_Start(&encoder_down) != HAL_OK)
    {
        return HAL_ERROR;
    }
    HAL_Delay(1000);

    return HAL_OK;
}


/**
 * @brief 初始化控制回路
 * 
 */
static void foc_drive_init_control_loops(void)
{
    PID_InitAngleLoop(&pid_angle, FOC_LOOP_DT_S, FOC_SPEED_LIMIT_RAD_S);
    PID_InitSpeedLoop(&pid_speed, FOC_LOOP_DT_S, FOC_IQ_LIMIT_A);
    PID_InitCloudLoop(&pid_cloud_y, FOC_LOOP_DT_S, FOC_SPEED_LIMIT_RAD_S);
    PID_InitCurrentLoop(&s_up_current_d_pid, FOC_LOOP_DT_S, FOC_UQ_LIMIT_V);
    PID_InitCurrentLoop(&s_up_current_q_pid, FOC_LOOP_DT_S, FOC_UQ_LIMIT_V);

    PID_InitAngleLoop(&s_down_angle_pid, FOC_LOOP_DT_S, FOC_SPEED_LIMIT_RAD_S);
    PID_InitSpeedLoop(&s_down_speed_pid, FOC_LOOP_DT_S, FOC_IQ_LIMIT_A);
    PID_InitCloudLoop(&pid_cloud_x, FOC_LOOP_DT_S, FOC_SPEED_LIMIT_RAD_S);
    PID_InitCurrentLoop(&s_down_current_d_pid, FOC_LOOP_DT_S, FOC_UQ_LIMIT_V);
    PID_InitCurrentLoop(&s_down_current_q_pid, FOC_LOOP_DT_S, FOC_UQ_LIMIT_V);

    FocMotor_Init(&s_up_motor,
                  &encoder_up,
                  MOTOR_UP,
                  &pid_angle,
                  &pid_speed,
                  &pid_cloud_y,
                  &s_up_current_d_pid,
                  &s_up_current_q_pid,
                  FOC_LOOP_DT_S,
                  FOC_SPEED_LIMIT_RAD_S,
                  FOC_IQ_LIMIT_A,
                  FOC_UQ_LIMIT_V);
    FocMotor_Init(&s_down_motor,
                  &encoder_down,
                  MOTOR_DOWN,
                  &s_down_angle_pid,
                  &s_down_speed_pid,
                  &pid_cloud_x,
                  &s_down_current_d_pid,
                  &s_down_current_q_pid,
                  FOC_LOOP_DT_S,
                  FOC_SPEED_LIMIT_RAD_S,
                  FOC_IQ_LIMIT_A,
                  FOC_UQ_LIMIT_V);
}

/**
 * @brief 根据编码器速度自动启停相位电压ADC的跟踪功能，以实现更稳定的零点漂移补偿
 * 
 * @param phase_adc 
 * @param enable 
 */
static void foc_drive_set_phase_tracking(FOC_DrivePhaseAdc_t *phase_adc, uint8_t enable)
{
    ADC_ChannelFilter_EnableTracking(&phase_adc->phase_a_filter, enable);
    ADC_ChannelFilter_EnableTracking(&phase_adc->phase_b_filter, enable);
    phase_adc->offset_track_enabled = (enable != 0U) ? 1U : 0U;
}

/**
 * @brief 更新相位电压ADC的跟踪状态，根据编码器速度自动启停跟踪功能
 * 
 * @param phase_adc 
 * @param encoder 
 * @param force_disable 强制禁用跟踪功能（如在转动过程中）
 */
static void foc_drive_update_motor_phase_tracking(FOC_DrivePhaseAdc_t *phase_adc,
                                                  const Encoder_t *encoder,
                                                  uint8_t force_disable)
{
    float mech_speed_abs = 0.0f;

    if (force_disable != 0U)
    {
        if (phase_adc->offset_track_enabled != 0U)
        {
            foc_drive_set_phase_tracking(phase_adc, 0U);
        }
        return;
    }

    mech_speed_abs = fabsf(Encoder_GetMechanicalVelocity(encoder));
    if (mech_speed_abs <= OFFSET_TRACK_ENABLE_SPEED_RAD_S)
    {
        if (phase_adc->offset_track_enabled == 0U)
        {
            foc_drive_set_phase_tracking(phase_adc, 1U);
        }
    }
    else if (phase_adc->offset_track_enabled != 0U)
    {
        foc_drive_set_phase_tracking(phase_adc, 0U);
    }
}


/**
 * @brief 更新相位偏置跟踪状态
 * 
 */
static void foc_drive_update_phase_offset_tracking(void)
{
    if (turn_flag != 0U)
    {
        foc_drive_update_motor_phase_tracking(&s_up_phase_adc, &encoder_up, 1U);
        foc_drive_update_motor_phase_tracking(&s_down_phase_adc, &encoder_down, 1U);
    }
    else
    {
        foc_drive_update_motor_phase_tracking(&s_up_phase_adc, &encoder_up, 0U);
        foc_drive_update_motor_phase_tracking(&s_down_phase_adc, &encoder_down, 0U);
    }
}

/**
 * @brief 更新相位电压ADC采样
 * 
 * @param phase_adc 
 * @return HAL_StatusTypeDef 
 */
static HAL_StatusTypeDef foc_drive_update_phase_adc(FOC_DrivePhaseAdc_t *phase_adc)
{
    return PhaseVoltageSampler_Update(&phase_adc->phase_a_filter,
                                      &phase_adc->phase_a_sample,
                                      &phase_adc->phase_b_filter,
                                      &phase_adc->phase_b_sample,
                                      &phase_adc->phase_sample);
}


/**
 * @brief Update phase current samples converted from sense voltages.
 * 
 */
static void foc_drive_update_phase_current_sample(void)
{
    if (foc_drive_update_phase_adc(&s_up_phase_adc) == HAL_OK)
    {
        if (s_up_phase_adc.phase_sample.valid != 0U)
        {
            Ia = foc_drive_sense_voltage_to_current(s_up_phase_adc.phase_sample.ua_v);
            Ib = foc_drive_sense_voltage_to_current(s_up_phase_adc.phase_sample.ub_v);
            Ic = foc_drive_sense_voltage_to_current(s_up_phase_adc.phase_sample.uc_v);
            s_up_phase_current.ia_a = Ia;
            s_up_phase_current.ib_a = Ib;
            s_up_phase_current.ic_a = Ic;
            s_up_phase_current.valid = 1U;
            s_up_phase_adc.sample_miss_count = 0U;
        }
        else
        {
            foc_drive_mark_phase_current_miss(&s_up_phase_adc, &s_up_phase_current);
        }
    }
    else
    {
        foc_drive_mark_phase_current_miss(&s_up_phase_adc, &s_up_phase_current);
    }

    if (foc_drive_update_phase_adc(&s_down_phase_adc) == HAL_OK)
    {
        if (s_down_phase_adc.phase_sample.valid != 0U)
        {
            Ia_down = foc_drive_sense_voltage_to_current(s_down_phase_adc.phase_sample.ua_v);
            Ib_down = foc_drive_sense_voltage_to_current(s_down_phase_adc.phase_sample.ub_v);
            Ic_down = foc_drive_sense_voltage_to_current(s_down_phase_adc.phase_sample.uc_v);
            s_down_phase_current.ia_a = Ia_down;
            s_down_phase_current.ib_a = Ib_down;
            s_down_phase_current.ic_a = Ic_down;
            s_down_phase_current.valid = 1U;
            s_down_phase_adc.sample_miss_count = 0U;
        }
        else
        {
            foc_drive_mark_phase_current_miss(&s_down_phase_adc, &s_down_phase_current);
        }
    }
    else
    {
        foc_drive_mark_phase_current_miss(&s_down_phase_adc, &s_down_phase_current);
    }
}

HAL_StatusTypeDef FOC_Drive_Init(void)
{
    foc_drive_init_adc_filters();

    if (foc_drive_start_pwm() != HAL_OK)
    {
        return HAL_ERROR;
    }
    disableAllPWM();

    foc_drive_calibrate_adc_filters();

    if (foc_drive_start_uart() != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (foc_drive_start_encoders() != HAL_OK)
    {
        return HAL_ERROR;
    }

    foc_drive_init_control_loops();
    s_vision_frame.find = 0U;
    s_last_vision_tick = 0U;
    s_vision_was_active = 0U;
    s_initialized = 1U;

    return HAL_OK;
}

void FOC_Drive_SetOpenLoop(uint8_t motor_id, float speed_rad_s, float uq_v)
{
    FocMotor_t *motor = NULL;

    if (s_initialized == 0U)
    {
        return;
    }

    if (motor_id == MOTOR_UP)
    {
        motor = &s_up_motor;
        FocMotor_SetDisabled(&s_down_motor);
    }
    else if (motor_id == MOTOR_DOWN)
    {
        motor = &s_down_motor;
        FocMotor_SetDisabled(&s_up_motor);
    }
    else
    {
        return;
    }

    PID_Reset(motor->pid_position);
    PID_Reset(motor->pid_speed);
    PID_Reset(motor->pid_vision);
    PID_Reset(motor->pid_current_d);
    PID_Reset(motor->pid_current_q);

    FocMotor_SetOpenLoop(motor, speed_rad_s, uq_v);
    s_open_loop_active = 1U;
}

void FOC_Drive_Service(void)
{
    if (s_initialized == 0U)
    {
        return;
    }

    foc_drive_update_phase_offset_tracking();
}


/**
 * @brief FOC驱动控制周期函数
 * 
 */
void FOC_Drive_ControlTick(void)
{
    const FocState_t *up_state = NULL;

    if (s_initialized == 0U)
    {
        return;
    }

    foc_drive_update_phase_current_sample();
    FocMotor_SetPhaseCurrent(&s_up_motor, &s_up_phase_current);
    FocMotor_SetPhaseCurrent(&s_down_motor, &s_down_phase_current);

    if (s_open_loop_active != 0U)
    {
        FocMotor_Tick(&s_up_motor);
        FocMotor_Tick(&s_down_motor);
    }
    else if (turn_flag != 0U)
    {
        FocMotor_SetPosition(&s_up_motor, target_angle);
        FocMotor_Tick(&s_up_motor);

        FocMotor_SetDisabled(&s_down_motor);
        FocMotor_Tick(&s_down_motor);
    }
    else if ((s_vision_frame.find != 0U) &&
             ((HAL_GetTick() - s_last_vision_tick) <= FOC_VISION_FRAME_TIMEOUT_MS))
    {
        FocVisionCommand_t up_command = {0};
        FocVisionCommand_t down_command = {0};

        up_command.measure = (float)s_vision_frame.y;
        up_command.center = DEFAULT_CONTORL_Y;
        up_command.valid = s_vision_frame.find;
        FocMotor_SetVision(&s_up_motor, &up_command);
        FocMotor_Tick(&s_up_motor);

        down_command.measure = (float)s_vision_frame.x;
        down_command.center = DEFAULT_CONTORL_X;
        down_command.valid = s_vision_frame.find;
        FocMotor_SetVision(&s_down_motor, &down_command);
        FocMotor_Tick(&s_down_motor);
        s_vision_was_active = 1U;
    }
    else
    {
        s_vision_frame.find = 0U;
        if (s_vision_was_active != 0U)
        {
            if (s_up_motor.mode == FOC_MODE_VISION)
            {
                s_up_hold_angle_rad = s_up_motor.state.mechanical_angle_rad;
                s_down_hold_angle_rad = s_down_motor.state.mechanical_angle_rad;
            }
            FocMotor_SetPosition(&s_up_motor, s_up_hold_angle_rad);
            FocMotor_Tick(&s_up_motor);
            FocMotor_SetPosition(&s_down_motor, s_down_hold_angle_rad);
            FocMotor_Tick(&s_down_motor);
        }
        else
        {
            FocMotor_SetDisabled(&s_up_motor);
            FocMotor_Tick(&s_up_motor);
            FocMotor_SetDisabled(&s_down_motor);
            FocMotor_Tick(&s_down_motor);
        }
    }

    up_state = FocMotor_GetState(&s_up_motor);
    if (up_state != NULL)
    {
        speed_raw = up_state->mechanical_speed_rad_s;
        speed_filt = speed_raw;
        uq = up_state->uq_applied_v;
    }

    s_mode_snapshot = (uint16_t)FocMotor_GetMode(&s_up_motor);
}

void FOC_Drive_VisionTask(void)
{
    if (s_initialized == 0U)
    {
        return;
    }

    if (UART_TryGetVisionFrame(&s_vision_frame) != 0U)
    {
        s_last_vision_tick = HAL_GetTick();
    }
}

void FOC_Drive_TelemetryTask(void)
{
    char line[96];
    int len;

    if (s_initialized == 0U)
    {
        return;
    }

   /*  printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
           "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
           s_up_phase_adc.phase_a_sample.filtered_voltage_v,
           s_up_phase_adc.phase_b_sample.filtered_voltage_v,
           Ia,
           Ib,
           Ic,
           s_up_motor.state.mechanical_angle_rad,
           s_up_motor.state.iq_target_a,
           s_down_phase_adc.phase_a_sample.filtered_voltage_v,
           s_down_phase_adc.phase_b_sample.filtered_voltage_v,
           Ia_down,
           Ib_down,
           Ic_down,
           s_down_motor.state.mechanical_angle_rad,
           s_down_motor.state.iq_target_a); */

    len = snprintf(line,
                   sizeof(line),
                   "%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f\r\n",
                   s_up_phase_adc.phase_a_sample.filtered_voltage_v,
                   s_up_phase_adc.phase_b_sample.filtered_voltage_v,
                   Ia,
                   Ib,
                   Ic,
                   s_up_motor.state.mechanical_angle_rad,
                   s_up_motor.state.id_measured_a,
                   s_up_motor.state.iq_measured_a,
                   s_up_motor.state.iq_target_a);
/*     len = snprintf(line,
                   sizeof(line),
                   "%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f\r\n",
                   s_down_phase_adc.phase_a_sample.filtered_voltage_v,
                   s_down_phase_adc.phase_b_sample.filtered_voltage_v,
                   Ia_down,
                   Ib_down,
                   Ic_down,
                   s_down_motor.state.mechanical_angle_rad,
                   s_down_motor.state.id_measured_a,
                   s_down_motor.state.iq_measured_a,
                   s_down_motor.state.iq_target_a); */
    if ((len > 0) && (len < (int)sizeof(line)))
    {
        Uart_Send(&huart1, (uint8_t *)line, (uint16_t)len);
    }
}

void FOC_Drive_Stop(void)
{
    if (s_initialized != 0U)
    {
        FocMotor_SetDisabled(&s_up_motor);
        FocMotor_SetDisabled(&s_down_motor);
    }

    s_vision_frame.find = 0U;
    s_last_vision_tick = 0U;
    s_vision_was_active = 0U;
    s_open_loop_active = 0U;
    s_mode_snapshot = (uint16_t)FOC_MODE_DISABLED;
    disableAllPWM();
}
