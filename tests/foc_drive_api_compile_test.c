#include "foc_drive.h"

int main(void)
{
    FOC_Drive_Service();
    FOC_Drive_ControlTick();
    FOC_Drive_VisionTask();
    FOC_Drive_TelemetryTask();
    FOC_Drive_Stop();

    return 0;
}
