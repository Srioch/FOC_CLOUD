/**
 * @file foc.h
 * @brief FOC control interfaces.
 */
#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include "tim.h"
#include "pid.h"
#include "encoder.h"
#include "kalman_filter.h"

#define _COMSTRAIN(AMT,MIN,MAX) ((AMT)<(MIN)?(MIN):((AMT)>(MAX)?(MAX):(AMT)))
#define FOC_POLE_PAIRS 7
#define VOLTAGE_LIMIT 12.12f
#define ANGLE_DEADZONE 0.0f
#define VOLTAGE_DEADZONE 0.5f
#define M_PI 3.14159265358979323846f

#define DEFAULT_KP 0.8f

#define CLOUD_UP_TIM htim2
#define CLOUD_DOWN_TIM htim3
#define CLOUD_UP_CHANNEL_1 TIM_CHANNEL_1
#define CLOUD_UP_CHANNEL_2 TIM_CHANNEL_2
#define CLOUD_UP_CHANNEL_3 TIM_CHANNEL_3
#define CLOUD_DOWN_CHANNEL_1 TIM_CHANNEL_2
#define CLOUD_DOWN_CHANNEL_2 TIM_CHANNEL_3
#define CLOUD_DOWN_CHANNEL_3 TIM_CHANNEL_4

#define DEFAULT_CONTORL_X 160.0f
#define DEFAULT_CONTORL_Y 120.0f

#define MOTOR_UP 1U
#define MOTOR_DOWN 2U

typedef struct
{
    PID_t pid_vis;
    PID_t pid_speed;
    kalman_filter_pos_speed_t kf_speed;
    Encoder_t *enc;
    uint8_t motor_id;
    float speed_ref;
    float uq;
} VisionControl_t;

typedef enum
{
    FOC_MODE_DISABLED = 0U,
    FOC_MODE_TORQUE = 1U,
    FOC_MODE_SPEED = 2U,
    FOC_MODE_POSITION = 3U,
    FOC_MODE_VISION = 4U
} FocMode_t;

typedef struct
{
    float mechanical_angle_rad;
    float electrical_angle_rad;
    float mechanical_speed_rad_s;
    float speed_target_rad_s;
    float uq_command_v;
    float uq_applied_v;
    float vision_error;
    uint8_t output_enabled;
} FocState_t;

typedef struct
{
    Encoder_t *enc;
    uint8_t motor_id;
    PID_t *pid_position;
    PID_t *pid_speed;
    PID_t *pid_vision;
    float dt_s;
    float speed_limit_rad_s;
    float uq_limit_v;
    FocMode_t mode;
    float target_position_rad;
    float target_speed_rad_s;
    float target_uq_v;
    float vision_measure;
    float vision_center;
    uint8_t vision_valid;
    FocState_t state;
} FocController_t;

typedef struct
{
    float measure;
    float center;
    uint8_t valid;
} FocVisionCommand_t;

float GetelectricalAngle(float mechanicalAngle);
float nomalizeAngle(float angle);

void setPWM(float Ua, float Ub, float Uc, uint8_t motor_id);
void disablePWM(uint8_t motor_id);
void disableAllPWM(void);
void setPhaseVoltage(float Uq, float Ud, float el_angle, uint8_t motor_id);

float velocityToVoltage(float velocity);
void openLoopSpeedControl(float target_rpm, float dt);
float angleControl(float target_angle, float current_angle, float Kp);
float torqueControl(float Uq, float eleangle, uint8_t motor_id);
void SpeedLoop_Update(Encoder_t *enc, kalman_filter_pos_speed_t *kf, float dt_s, float speed_ref);

float Cloud_Control(uint16_t x, uint16_t y, uint8_t find);
float Moto_Control(uint16_t measure, PID_t *pid, uint16_t target, Encoder_t *enc, uint8_t motor_id);
void VisionControl_Init(VisionControl_t *vc, PID_t *pid_speed, kalman_filter_pos_speed_t *kf_speed, Encoder_t *enc, uint8_t motor_id);
void VisionOuterLoop(VisionControl_t *vc, float measure, float center, uint8_t valid, float dt);
float MotorSpeedLoop_Update(VisionControl_t *vc, float dt_s);

void FocController_Init(FocController_t *ctrl,
                        Encoder_t *enc,
                        uint8_t motor_id,
                        PID_t *pid_position,
                        PID_t *pid_speed,
                        PID_t *pid_vision,
                        float dt_s,
                        float speed_limit_rad_s,
                        float uq_limit_v);
void FocController_Reset(FocController_t *ctrl);
void runFoc(FocController_t *ctrl);
void runFocVision(FocController_t *ctrl, const FocVisionCommand_t *cmd);

#endif /* __FOC_H__ */
