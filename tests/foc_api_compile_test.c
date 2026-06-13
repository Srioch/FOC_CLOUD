#include "foc.h"

#include <math.h>

int main(void)
{
    Encoder_t encoder = {0};
    PID_t pid_position = {0};
    PID_t pid_speed_local = {0};
    PID_t pid_vision = {0};
    PID_t pid_current_d = {0};
    PID_t pid_current_q = {0};
    FocMotor_t motor = {0};
    FocVisionCommand_t vision = {0};
    FocPhaseCurrent_t phase_current = {0};
    FocDqCurrent_t dq_current = {0};
    const FocState_t *state = 0;

    PID_InitAngleLoop(&pid_position, 0.001f, 20.0f);
    PID_InitSpeedLoop(&pid_speed_local, 0.001f, 3.0f);
    PID_InitCloudLoop(&pid_vision, 0.001f, 20.0f);
    PID_InitCurrentLoop(&pid_current_d, 0.001f, 6.0f);
    PID_InitCurrentLoop(&pid_current_q, 0.001f, 6.0f);

    phase_current.ia_a = 1.0f;
    phase_current.ib_a = -0.5f;
    phase_current.ic_a = -0.5f;
    phase_current.valid = 1U;
    Foc_TransformPhaseCurrentToDq(&phase_current, 0.0f, &dq_current);

    FocMotor_Init(&motor,
                  &encoder,
                  MOTOR_UP,
                  &pid_position,
                  &pid_speed_local,
                  &pid_vision,
                  &pid_current_d,
                  &pid_current_q,
                  0.001f,
                  20.0f,
                  3.0f,
                  6.0f);
    FocMotor_SetPhaseCurrent(&motor, &phase_current);

    FocMotor_SetSpeed(&motor, 5.0f);
    FocMotor_Tick(&motor);

    vision.measure = 120.0f;
    vision.center = 120.0f;
    vision.valid = 1U;
    FocMotor_SetVision(&motor, &vision);
    FocMotor_Tick(&motor);

    FocMotor_SetDisabled(&motor);
    FocMotor_Tick(&motor);

    state = FocMotor_GetState(&motor);

    return (FocMotor_GetMode(&motor) == FOC_MODE_DISABLED) &&
           (state != 0) &&
           (motor.motor_id == MOTOR_UP) &&
           (motor.dt_s > 0.0f) &&
           (motor.iq_limit_a > 0.0f) &&
           (motor.uq_limit_v > 0.0f) &&
           (fabsf(dq_current.id_a - 1.0f) < 0.001f) &&
           (fabsf(dq_current.iq_a) < 0.001f) ? 0 : 1;
}
