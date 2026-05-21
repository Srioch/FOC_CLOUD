/**
 * @file uart.c
 * @author 2743823168@qq.com
 * @brief 串口模板实现
 * @version 0.1
 * @date 2026-03-22 更新串口接收处理等功能
 * 
 * @copyright Copyright (c) 2026
 * 
 */



#include "uart.h"
#include "uartdma.h"


#define RX_BUFFER_SIZE 100


static UART_HandleTypeDef *g_huart = NULL;
static uint8_t rx_byte;
static uint8_t rx_byte_u2;

static char rx_buffer[RX_BUFFER_SIZE];
static char rx_buffer_pending[RX_BUFFER_SIZE];
static volatile uint8_t rx_index = 0;
static volatile uint8_t rx_ready = 0;
static volatile uint8_t rx_overflow = 0;
static volatile uint8_t rx_overflow_flag = 0;
uint8_t turn_flag = 0;
static uint8_t openmv_data[7];// 7字节帧：0xa3, 0xb3, ID, xH, xL, y, 0xc3
static uint8_t rx_16t;
volatile VisionData_t vision_data;
static volatile uint8_t vision_frame_ready = 0U;
static volatile uint16_t pwm_value = 0U;



//绑定UART句柄，实现printf输出不同串口
void UART_SetHandle(UART_HandleTypeDef *huart)
{
	g_huart = huart;
}

HAL_StatusTypeDef UART_StartReceiveIT(UART_HandleTypeDef *huart)
{
     if(huart == NULL)
    {
        return HAL_ERROR;
    }

    if(huart->Instance == USART1)
    {
        return HAL_UART_Receive_IT(huart, &rx_byte, 1U);
    }

    if(huart->Instance == USART2)
    {
        return HAL_UART_Receive_IT(huart, &rx_byte_u2, 1U);
    }


    return HAL_ERROR;
}


void Uart_Send(UART_HandleTypeDef *huart,uint8_t *padata,uint16_t size)
{
    (void)UART_SendDMA(huart, padata, size);
}

//重定向printf函数到UART
int fputc(int ch, FILE *f)
{
	uint8_t c = (uint8_t)ch;
    HAL_StatusTypeDef status;
    (void)f;

	if (g_huart != NULL)
	{
        status = UART_SendDMA(g_huart, &c, 1U);
        if (status == HAL_BUSY)
        {
            (void)status;
        }
	}
	else
	{
		extern UART_HandleTypeDef huart1; /* fallback to USART1 if available */
        status = UART_SendDMA(&huart1, &c, 1U);
        if (status == HAL_BUSY)
        {
            (void)status;
        }
	}

	return ch;
}

static void UART_RXHandleLine(uint8_t byte)
{
    if((byte == '\r') || (byte == '\n'))
    {
       if(rx_overflow != 0)
       {
            rx_overflow = 0;rx_overflow_flag = 1;
            rx_index = 0U;
            return;
       } 

       if(rx_index == 0U) return;

       if((rx_index > 0) && (rx_ready == 0U))
       {
            memcpy(rx_buffer_pending, rx_buffer, rx_index);
            rx_buffer_pending[rx_index] = '\0';//添加字符串结束符号
            rx_ready = 1U;//标记接收完成
       }

       rx_index = 0U;
    }

    if(rx_ready != 0U) return;

    if(rx_index < RX_BUFFER_SIZE - 1)
        rx_buffer[rx_index++] = (char)byte;
    else
    {
        rx_overflow = 1U;//标记溢出
        rx_index = 0U;//重置索引
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(g_huart != NULL && huart == &huart1)
    {
    
        // byte 接收字节开始处理
        UART_RXHandleLine(rx_byte);
        HAL_UART_Receive_IT(g_huart, &rx_byte, 1U);
    }

    if(huart == &huart2)
    {
        rx_16t = rx_byte_u2;
        HAL_UART_Receive_IT(&huart2, &rx_byte_u2, 1U);
        static uint8_t openmv_data_index = 0U;
        openmv_data[openmv_data_index++] = rx_16t;
        if(openmv_data_index == 1) vision_data.find = 0U;//每次接收新帧的第一个字节时，重置 find 标志
        if(openmv_data[0] != 0xa3) openmv_data_index = 0U;
        if((openmv_data_index == 2) && (openmv_data[1] != 0xb3)) openmv_data_index = 0U;
        if(openmv_data_index == 7)
        {
            if(openmv_data[6] == 0xc3)
            {
                vision_data.ID = openmv_data[2];
                vision_data.find = 1U;
                vision_data.x = (uint16_t)(openmv_data[3] << 8 |openmv_data[4]);
                vision_data.y = openmv_data[5];
                openmv_data_index = 0U;
                vision_frame_ready = 1U;
            }
            openmv_data_index = 0U;

        }

    }

}

void UART_ProcessPendingCommand(void)
{
    uint8_t command[RX_BUFFER_SIZE];
    uint8_t command_ready = 0U;
    command[0] = '\0';



    if(rx_ready != 0U)
    {
        // 这里可以添加对 rx_buffer_pending 中命令的解析和处理逻辑
        // 例如，解析命令并执行相应的操作
        memcpy(command, rx_buffer_pending, RX_BUFFER_SIZE);
        rx_ready = 0U; // 处理完成后重置标志
        command_ready = 1U;
    }

    if(rx_overflow_flag != 0U)
    {
        // 这里可以添加对接收溢出的处理逻辑

        rx_overflow_flag = 0U;
    }

    if(command_ready != 0U)
      UART_CommandHandler((const char *)command);
}

void UART_CommandHandler(const char *command)
{
    // 这里可以添加对接收到的命令的具体处理逻辑
    // 例如，根据命令内容执行不同的操作
    float temp = 0.0f;
    if ((sscanf(command, "SET ANGLE:%f", &temp) == 1) ||
        (sscanf(command, "SET ANGLE %f", &temp) == 1))
    {
        target_angle = temp / 180.0f * M_PI; // 将角度转换为弧度
        printf("Target angle set to: %.2f deg (%.4f rad)\r\n", temp, target_angle);
    }
    else if(strcmp(command, "TURN") == 0)
    {
        turn_flag = 1U;
        printf("Turn command received\r\n");
    }
    else if(strcmp(command, "STOP") == 0)
    {
        turn_flag = 0U;
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_3, 0);
         printf("Stop command received\r\n");
    }
    else if(sscanf(command, "SET KP:%f", &temp) == 1)
    {
            pid_speed.Kp = temp;
        printf("Angle control KP set to: %.2f\r\n", pid_speed.Kp);
    }
    else if(sscanf(command, "SET KI:%f", &temp) == 1)
    {
            pid_speed.Ki = temp;
        printf("Angle control KI set to: %.2f\r\n", pid_speed.Ki);
    }

    else
    {
        printf("Unknown command: %s\r\n", command);
    }

    printf("Received command: %s\r\n", command);

}

uint8_t UART_TryGetVisionFrame(VisionData_t *frame)
{
      uint8_t ready = 0U;

      if (frame == NULL)
      {
          return 0U;
      }

      __disable_irq();
      if (vision_frame_ready)
      {
          *frame = vision_data;
          vision_frame_ready = 0U;
          ready = 1U;
      }
      __enable_irq();

      return ready;
}


void UART_TelemetryTask(void)
{
    // 这里可以添加周期性发送遥测数据的逻辑
    // 例如，定时发送系统状态、传感器数据等
    
}