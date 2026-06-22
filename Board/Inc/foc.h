/**
 * @file foc.h
 * @brief FOC motor control interface.
 */
#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include "tim.h"
#include "pid.h"
#include "Encoder.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define FOC_POLE_PAIRS 7U
#define VOLTAGE_LIMIT 12.12f
extern float angle_deadzone;

#define CLOUD_UP_TIM htim2
#define CLOUD_DOWN_TIM htim3
#define CLOUD_UP_CHANNEL_1 TIM_CHANNEL_1
#define CLOUD_UP_CHANNEL_2 TIM_CHANNEL_2
#define CLOUD_UP_CHANNEL_3 TIM_CHANNEL_3
#define CLOUD_DOWN_CHANNEL_1 TIM_CHANNEL_2
#define CLOUD_DOWN_CHANNEL_2 TIM_CHANNEL_3
#define CLOUD_DOWN_CHANNEL_3 TIM_CHANNEL_4

extern float default_control_x;
extern float default_control_y;

#define MOTOR_UP 1U
#define MOTOR_DOWN 2U

typedef enum
{
    FOC_MODE_DISABLED = 0U,
    FOC_MODE_TORQUE = 1U,
    FOC_MODE_SPEED = 2U,
    FOC_MODE_POSITION = 3U,
    FOC_MODE_VISION = 4U,
    FOC_MODE_OPEN_LOOP = 5U
} FocMode_t;

typedef struct
{
    float ia_a;
    float ib_a;
    float ic_a;
    uint8_t valid;
} FocPhaseCurrent_t;

typedef struct
{
    float id_a;
    float iq_a;
    uint8_t valid;
} FocDqCurrent_t;

typedef struct
{
    float mechanical_angle_rad;
    float electrical_angle_rad;
    float mechanical_speed_rad_s;
    float speed_target_rad_s;
    float id_target_a;
    float iq_target_a;
    float id_measured_a;
    float iq_measured_a;
    float ud_command_v;
    float uq_command_v;
    float ud_applied_v;
    float uq_applied_v;
    float vision_error;
    uint8_t current_valid;
    uint8_t output_enabled;
} FocState_t;

typedef struct
{
    float speed_rad_s;
    float uq_v;
    float angle_rad;
} FocOpenLoopState_t;

typedef struct
{
    float measure;
    float center;
    uint8_t valid;
} FocVisionCommand_t;

typedef struct
{
    Encoder_t *enc;
    uint8_t motor_id;

    PID_t *pid_position;
    PID_t *pid_speed;
    PID_t *pid_vision;
    PID_t *pid_current_d;
    PID_t *pid_current_q;

    float dt_s;
    float speed_limit_rad_s;
    float iq_limit_a;
    float uq_limit_v;

    FocMode_t mode;
    float target_position_rad;
    float target_speed_rad_s;
    float target_iq_a;
    FocVisionCommand_t vision;
    FocOpenLoopState_t open_loop;
    FocPhaseCurrent_t phase_current;
    FocDqCurrent_t dq_current;

    FocState_t state;
} FocMotor_t;

float Foc_GetElectricalAngle(float mechanical_angle_rad);
float Foc_NormalizeAngle(float angle_rad);
void Foc_TransformPhaseCurrentToDq(const FocPhaseCurrent_t *phase_current,
                                   float electrical_angle_rad,
                                   FocDqCurrent_t *dq_current);

void setPWM(float Ua, float Ub, float Uc, uint8_t motor_id);
void disablePWM(uint8_t motor_id);
void disableAllPWM(void);
void setPhaseVoltage(float Uq, float Ud, float electrical_angle_rad, uint8_t motor_id);

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
                   float uq_limit_v);
void FocMotor_Reset(FocMotor_t *motor);
void FocMotor_SetDisabled(FocMotor_t *motor);
void FocMotor_SetTorque(FocMotor_t *motor, float iq_a);
void FocMotor_SetSpeed(FocMotor_t *motor, float speed_rad_s);
void FocMotor_SetPosition(FocMotor_t *motor, float position_rad);
void FocMotor_SetVision(FocMotor_t *motor, const FocVisionCommand_t *command);
void FocMotor_SetOpenLoop(FocMotor_t *motor, float speed_rad_s, float uq_v);
void FocMotor_SetPhaseCurrent(FocMotor_t *motor, const FocPhaseCurrent_t *phase_current);
void FocMotor_Tick(FocMotor_t *motor);
const FocState_t *FocMotor_GetState(const FocMotor_t *motor);
FocMode_t FocMotor_GetMode(const FocMotor_t *motor);

#endif /* __FOC_H__ */
