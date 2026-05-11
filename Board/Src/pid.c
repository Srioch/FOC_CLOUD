#include "pid.h"

#include <math.h>

#define PID_PI      3.14159265358979323846f
#define PID_TWO_PI  6.28318530717958647692f


PID_t pid_speed;

static float pid_clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

static float pid_update_from_error(PID_t *pid, float error)
{
    float p = 0.0f;
    float i_candidate = 0.0f;
    float d_raw = 0.0f;
    float alpha = 0.0f;
    float d_filtered = 0.0f;
    float d = 0.0f;
    float out_unsat = 0.0f;
    float out_sat = 0.0f;

    if (pid == 0)
    {
        return 0.0f;
    }

    if (pid->dt_s <= 0.0f)
    {
        return 0.0f;
    }

    p = pid->Kp * error;

    i_candidate = pid->integrator + (pid->Ki * pid->dt_s * error);
    i_candidate = pid_clampf(i_candidate, pid->out_min, pid->out_max);

    d_raw = (error - pid->prev_error) / pid->dt_s;
    alpha = pid->dt_s / (pid->deriv_filter_tau_s + pid->dt_s);
    d_filtered = pid->prev_derivative + alpha * (d_raw - pid->prev_derivative);
    pid->prev_derivative = d_filtered;
    d = pid->Kd * d_filtered;

    out_unsat = p + i_candidate + d;
    out_sat = pid_clampf(out_unsat, pid->out_min, pid->out_max);

    /*
     * Conditional integration anti-windup:
     * if saturated and error keeps pushing to saturation, reject integrator growth.
     */
    if (((out_unsat > pid->out_max) && (error > 0.0f)) ||
        ((out_unsat < pid->out_min) && (error < 0.0f)))
    {
        i_candidate = pid->integrator;
        out_unsat = p + i_candidate + d;
        out_sat = pid_clampf(out_unsat, pid->out_min, pid->out_max);
    }

    pid->integrator = i_candidate;
    pid->prev_error = error;

    return out_sat;
}

void PID_Init(PID_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->Kp = PID_SPEED_KP_DEFAULT;
    pid->Ki = PID_SPEED_KI_DEFAULT;
    pid->Kd = PID_SPEED_KD_DEFAULT;

    pid->dt_s = PID_DEFAULT_DT_S;
    pid->out_min = PID_DEFAULT_OUT_MIN;
    pid->out_max = PID_DEFAULT_OUT_MAX;

    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;

    pid->deriv_filter_tau_s = PID_DEFAULT_DERIV_TAU_S;
    pid->prev_derivative = 0.0f;
}

void PID_Reset(PID_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
}

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd)
{
    if (pid == 0)
    {
        return;
    }

    pid->Kp = (kp < 0.0f) ? 0.0f : kp;
    pid->Ki = (ki < 0.0f) ? 0.0f : ki;
    pid->Kd = (kd < 0.0f) ? 0.0f : kd;
}

void PID_SetDt(PID_t *pid, float dt_s)
{
    if (pid == 0)
    {
        return;
    }

    if (dt_s > 0.0f)
    {
        pid->dt_s = dt_s;
    }
}

void PID_SetOutputLimit(PID_t *pid, float out_min, float out_max)
{
    float lo = 0.0f;
    float hi = 0.0f;

    if (pid == 0)
    {
        return;
    }

    lo = (out_min < out_max) ? out_min : out_max;
    hi = (out_min < out_max) ? out_max : out_min;

    pid->out_min = lo;
    pid->out_max = hi;
    pid->integrator = pid_clampf(pid->integrator, pid->out_min, pid->out_max);
}

void PID_SetDerivativeFilter(PID_t *pid, float tau_s)
{
    if (pid == 0)
    {
        return;
    }

    if (tau_s < 0.0f)
    {
        tau_s = 0.0f;
    }

    pid->deriv_filter_tau_s = tau_s;
}

float PID_Update(PID_t *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;
    return pid_update_from_error(pid, error);
}

float PID_WrapPmPi(float angle_rad)
{
    float a = fmodf(angle_rad + PID_PI, PID_TWO_PI);

    if (a < 0.0f)
    {
        a += PID_TWO_PI;
    }

    return a - PID_PI;
}

float PID_UpdateAngleWrapped(PID_t *pid, float setpoint_rad, float measurement_rad)
{
    float error = PID_WrapPmPi(setpoint_rad - measurement_rad);
    return pid_update_from_error(pid, error);
}

void PID_InitSpeedLoop(PID_t *pid, float dt_s, float uq_limit)
{
    float limit = fabsf(uq_limit);

    PID_Init(pid);
    if (pid == 0)
    {
        return;
    }

    pid->Kp = PID_SPEED_KP_DEFAULT;
    pid->Ki = PID_SPEED_KI_DEFAULT;
    pid->Kd = PID_SPEED_KD_DEFAULT;

    PID_SetDt(pid, dt_s);
    if (limit <= 0.0f)
    {
        limit = -PID_DEFAULT_OUT_MIN;
    }
    PID_SetOutputLimit(pid, -limit, limit);
    PID_SetDerivativeFilter(pid, PID_DEFAULT_DERIV_TAU_S);
    PID_Reset(pid);
}

void PID_InitAngleLoop(PID_t *pid, float dt_s, float speed_limit_rad_s)
{
    float limit = fabsf(speed_limit_rad_s);

    PID_Init(pid);
    if (pid == 0)
    {
        return;
    }

    pid->Kp = PID_ANGLE_KP_DEFAULT;
    pid->Ki = PID_ANGLE_KI_DEFAULT;
    pid->Kd = PID_ANGLE_KD_DEFAULT;

    PID_SetDt(pid, dt_s);
    if (limit <= 0.0f)
    {
        limit = PID_ANGLE_OUT_LIMIT_DEFAULT;
    }
    PID_SetOutputLimit(pid, -limit, limit);
    PID_SetDerivativeFilter(pid, PID_DEFAULT_DERIV_TAU_S);
    PID_Reset(pid);
}
