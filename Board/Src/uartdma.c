#include "uartdma.h"

#define UARTDMA_TX_FIFO_SIZE 512U

static UART_HandleTypeDef *s_uart = NULL;
static uint8_t s_tx_fifo[UARTDMA_TX_FIFO_SIZE];
static volatile uint16_t s_tx_head = 0U;
static volatile uint16_t s_tx_tail = 0U;
static volatile uint16_t s_dma_len = 0U;
static volatile uint8_t s_dma_busy = 0U;
static volatile uint32_t s_drop_bytes = 0U;


/**
 * @brief 获取FIFO剩余空间大小
 * 
 * @return uint16_t 
 */
static uint16_t uartdma_get_free_nolock(void)
{
	if (s_tx_head >= s_tx_tail)
	{
		return (uint16_t)(UARTDMA_TX_FIFO_SIZE - (s_tx_head - s_tx_tail) - 1U);
	}

	return (uint16_t)(s_tx_tail - s_tx_head - 1U);
}

/**
 * @brief 启动DMA传输
 * 
 */
static void uartdma_kick_nolock(void)
{
	uint16_t len;

	if ((s_uart == NULL) || (s_uart->hdmatx == NULL) || (s_dma_busy != 0U))
	{
		return;
	}

	if (s_tx_head == s_tx_tail)
	{
		return;
	}

	if (s_tx_head > s_tx_tail)
	{
		len = (uint16_t)(s_tx_head - s_tx_tail);
	}
	else
	{
		len = (uint16_t)(UARTDMA_TX_FIFO_SIZE - s_tx_tail);
	}

	s_dma_len = len;
	s_dma_busy = 1U;
	if (HAL_UART_Transmit_DMA(s_uart, &s_tx_fifo[s_tx_tail], len) != HAL_OK)
	{
		s_dma_busy = 0U;
		s_dma_len = 0U;
	}
}

HAL_StatusTypeDef UART_SendDMA(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len)
{
	uint16_t i;

	if ((huart == NULL) || (data == NULL))
	{
		return HAL_ERROR;
	}

	if (len == 0U)
	{
		return HAL_OK;
	}

	/* 若 DMA 尚未配置，退化为阻塞发送，保证兼容。 */
	if (huart->hdmatx == NULL)
	{
		return HAL_UART_Transmit(huart, (uint8_t *)data, len, 0xFFFFU);
	}

	__disable_irq();

	if (s_uart == NULL)
	{
		s_uart = huart;
	}
	else if (s_uart != huart)
	{
		__enable_irq();
		return HAL_ERROR;
	}

	if (uartdma_get_free_nolock() < len)// FIFO 空间不足，丢弃数据并统计丢弃字节数
	{
		s_drop_bytes += len;
		__enable_irq();
		return HAL_BUSY;
	}

	for (i = 0U; i < len; i++)
	{
		s_tx_fifo[s_tx_head] = data[i];
		s_tx_head = (uint16_t)((s_tx_head + 1U) % UARTDMA_TX_FIFO_SIZE);
	}

	uartdma_kick_nolock();
	__enable_irq();

	return HAL_OK;
}


/**
 * @brief 获取丢弃的字节数
 * 
 * @return uint32_t 
 */
uint32_t UART_GetTxDropBytes(void)
{
	return s_drop_bytes;
}

/**
 * @brief UART发送完成回调函数
 * 
 * @param huart 
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uint16_t sent;

	if ((s_uart == NULL) || (huart != s_uart))
	{
		return;
	}

	__disable_irq();

	sent = s_dma_len;
	s_dma_len = 0U;
	s_tx_tail = (uint16_t)((s_tx_tail + sent) % UARTDMA_TX_FIFO_SIZE);//
	s_dma_busy = 0U;
	uartdma_kick_nolock();

	__enable_irq();
}
