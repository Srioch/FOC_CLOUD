/**
 * @file foc_drive.h
 * @brief Application-level FOC drive facade.
 */
#ifndef __FOC_DRIVE_H__
#define __FOC_DRIVE_H__

#include "main.h"
#include "pid.h"

HAL_StatusTypeDef FOC_Drive_Init(void);
void FOC_Drive_Service(void);
void FOC_Drive_ControlTick(void);
void FOC_Drive_VisionTask(void);
void FOC_Drive_TelemetryTask(void);
void FOC_Drive_Stop(void);
void FOC_Drive_SetOpenLoop(uint8_t motor_id, float speed_rad_s, float uq_v);

PID_t *FOC_Drive_GetUpCurrentDPid(void);
PID_t *FOC_Drive_GetUpCurrentQPid(void);
PID_t *FOC_Drive_GetDownSpeedPid(void);
PID_t *FOC_Drive_GetDownAnglePid(void);
PID_t *FOC_Drive_GetDownCurrentDPid(void);
PID_t *FOC_Drive_GetDownCurrentQPid(void);

void  FOC_Drive_SetSpeedLimit(float limit_rad_s);
void  FOC_Drive_SetIqLimit(float limit_a);
void  FOC_Drive_SetUqLimit(float limit_v);
float FOC_Drive_GetSpeedLimit(void);
float FOC_Drive_GetIqLimit(void);
float FOC_Drive_GetUqLimit(void);

void  FOC_Drive_SetGravityGain(uint8_t motor_id, float gain_a);
void  FOC_Drive_SetGravityOffset(uint8_t motor_id, float offset_rad);
float FOC_Drive_GetGravityGain(uint8_t motor_id);
float FOC_Drive_GetGravityOffset(uint8_t motor_id);

#endif /* __FOC_DRIVE_H__ */
