#include "foc.h"

int main(void)
{
    Encoder_t encoder = {0};
    PID_t pid_position = {0};
    PID_t pid_speed_local = {0};
    PID_t pid_vision = {0};
    FocController_t controller = {0};
    FocVisionCommand_t vision = {0};

    PID_InitAngleLoop(&pid_position, 0.001f, 20.0f);
    PID_InitSpeedLoop(&pid_speed_local, 0.001f, 6.0f);
    PID_InitCloudLoop(&pid_vision, 0.001f, 20.0f);

    FocController_Init(&controller,
                       &encoder,
                       MOTOR_UP,
                       &pid_position,
                       &pid_speed_local,
                       &pid_vision,
                       0.001f,
                       20.0f,
                       6.0f);

    controller.mode = FOC_MODE_SPEED;
    controller.target_speed_rad_s = 5.0f;
    runFoc(&controller);

    vision.measure = 120.0f;
    vision.center = 120.0f;
    vision.valid = 1U;
    runFocVision(&controller, &vision);

    controller.mode = FOC_MODE_DISABLED;
    runFoc(&controller);

    return (controller.motor_id == MOTOR_UP) &&
           (controller.dt_s > 0.0f) &&
           (controller.uq_limit_v > 0.0f) ? 0 : 1;
}
