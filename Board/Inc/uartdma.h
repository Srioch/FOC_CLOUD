/**
 * @file uartdma.h
 * @author 2743823168@qq.com
 * @brief 串口dma
 * @version 0.1
 * @date 2026-04-15
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef __UARTDMA_H__
#define __UARTDMA_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* DMA 发送接口（非阻塞，带环形缓冲） */
HAL_StatusTypeDef UART_SendDMA(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len);

/* 查看因缓冲区满而丢弃的字节数（调试用） */
uint32_t UART_GetTxDropBytes(void);

#ifdef __cplusplus
}
#endif

#endif /* __UARTDMA_H__ */