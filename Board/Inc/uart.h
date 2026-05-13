/**
 * @file uart.h
 * @author 2743823168@qq.com
 * @brief 串口模板头文件
 * @version 0.1
 * @date 2026-03-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __UART_H__
#define __UART_H__

#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "pid.h"
#include "Encoder.h"
#include "foc.h"

typedef struct
{
    uint8_t ID;
    uint16_t x;
    uint8_t y;
    uint8_t find;
} VisionData_t;




extern uint8_t turn_flag;
extern volatile VisionData_t vision_data;



void Uart_Send(UART_HandleTypeDef *huart, uint8_t *padata, uint16_t size);

/* 设置 printf 重定向所使用的 UART 句柄。 */
void UART_SetHandle(UART_HandleTypeDef *huart);

/* 启动按字节接收的串口中断，用于命令行解析。 */
HAL_StatusTypeDef UART_StartReceiveIT(UART_HandleTypeDef *huart);

/* 在主循环中轮询并执行已经接收完成的上位机命令。 */
void UART_ProcessPendingCommand(void);

/* 周期性遥测打印任务，在主循环中调用。 */
void UART_TelemetryTask(void);

/* printf 重定向接口：实现 fputc，使标准 printf 输出到串口。 */
int fputc(int ch, FILE *f);

void UART_CommandHandler(const char *command);

void dataGet(uint8_t *command, VisionData_t *vision_data);

uint8_t UART_TryGetVisionFrame(VisionData_t *frame);

#endif
