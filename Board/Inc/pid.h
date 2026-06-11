#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

/*
 * Generic PID controller for this FOC project.
 *
 * Suggested loop arrangement:
 * - Angle loop output: speed command (rad/s)
 * - Speed loop output: Uq command (V)
 */

#define PID_DEFAULT_DT_S            0.001f
#define PID_DEFAULT_DERIV_TAU_S     0.002f
#define PID_DEFAULT_OUT_MIN         (-6.0f)
#define PID_DEFAULT_OUT_MAX         (6.0f)

/* Speed loop defaults: input/output unit = rad/s -> V */
#define PID_SPEED_KP_DEFAULT        0.06f
#define PID_SPEED_KI_DEFAULT        0.05f
#define PID_SPEED_KD_DEFAULT        0.00f

/* Angle loop defaults: input/output unit = rad -> rad/s */
#define PID_ANGLE_KP_DEFAULT        20.00f
#define PID_ANGLE_KI_DEFAULT        0.00f
#define PID_ANGLE_KD_DEFAULT        0.00f
#define PID_ANGLE_OUT_LIMIT_DEFAULT 30.0f

#define PID_CLOUD_KP_DEFAULT 0.2f
#define PID_CLOUD_KI_DEFAULT 0.0f
#define PID_CLOUD_KD_DEFAULT 0.0f




typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float dt_s;
    float out_min;
    float out_max;

    float integrator;
    float prev_error;

    float deriv_filter_tau_s;
    float prev_derivative;
} PID_t;

extern PID_t pid_speed;
extern PID_t pid_angle;

extern PID_t pid_cloud_x;
extern PID_t pid_cloud_y;


void PID_Init(PID_t *pid);
void PID_Reset(PID_t *pid);

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd);
void PID_SetDt(PID_t *pid, float dt_s);
void PID_SetOutputLimit(PID_t *pid, float out_min, float out_max);
void PID_SetDerivativeFilter(PID_t *pid, float tau_s);

float PID_Update(PID_t *pid, float setpoint, float measurement);

/* Wrap angle into [-pi, pi) */
float PID_WrapPmPi(float angle_rad);
/* Angle-aware PID update (shortest path error), output often used as speed command */
float PID_UpdateAngleWrapped(PID_t *pid, float setpoint_rad, float measurement_rad);

/* Project-friendly presets */
void PID_InitSpeedLoop(PID_t *pid, float dt_s, float uq_limit);
void PID_InitAngleLoop(PID_t *pid, float dt_s, float speed_limit_rad_s);

float PID_PositionalControl(PID_t *pid, float target, float current);

void PID_InitCloudLoop(PID_t *pid, float dt_s, float out_limit);

#endif
