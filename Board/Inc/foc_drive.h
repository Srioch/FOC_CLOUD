/**
 * @file foc_drive.h
 * @brief Application-level FOC drive facade.
 */
#ifndef __FOC_DRIVE_H__
#define __FOC_DRIVE_H__

#include "main.h"

HAL_StatusTypeDef FOC_Drive_Init(void);
void FOC_Drive_Service(void);
void FOC_Drive_ControlTick(void);
void FOC_Drive_VisionTask(void);
void FOC_Drive_TelemetryTask(void);
void FOC_Drive_Stop(void);

#endif /* __FOC_DRIVE_H__ */
