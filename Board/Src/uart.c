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
#include "foc_drive.h"
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

static PID_t *cmd_lookup_pid(const char *motor, const char *loop)
{
    if (motor == NULL || loop == NULL) return NULL;

    if (strcmp(motor, "UP") == 0) {
        if (strcmp(loop, "ANG") == 0)  return &pid_angle;
        if (strcmp(loop, "SPD") == 0)  return &pid_speed;
        if (strcmp(loop, "VIS") == 0)  return &pid_cloud_y;
        if (strcmp(loop, "CURD") == 0) return FOC_Drive_GetUpCurrentDPid();
        if (strcmp(loop, "CURQ") == 0) return FOC_Drive_GetUpCurrentQPid();
    } else if (strcmp(motor, "DN") == 0) {
        if (strcmp(loop, "ANG") == 0)  return FOC_Drive_GetDownAnglePid();
        if (strcmp(loop, "SPD") == 0)  return FOC_Drive_GetDownSpeedPid();
        if (strcmp(loop, "VIS") == 0)  return &pid_cloud_x;
        if (strcmp(loop, "CURD") == 0) return FOC_Drive_GetDownCurrentDPid();
        if (strcmp(loop, "CURQ") == 0) return FOC_Drive_GetDownCurrentQPid();
    }
    return NULL;
}

static uint8_t cmd_try_set_pid(const char *command)
{
    char motor[4] = {0};
    char loop[6] = {0};
    char gain[4] = {0};
    float value = 0.0f;
    PID_t *pid = NULL;
    int matched;

    matched = sscanf(command, "SET %3[^.].%5[^.].%3[^ :]:%f", motor, loop, gain, &value);
    if (matched != 4) {
        matched = sscanf(command, "SET %3[^.].%5[^.].%3[^ ] %f", motor, loop, gain, &value);
    }
    if (matched != 4) return 0U;

    pid = cmd_lookup_pid(motor, loop);
    if (pid == NULL) {
        printf("Unknown PID: %s.%s\r\n", motor, loop);
        return 1U;
    }

    if (strcmp(gain, "KP") == 0) {
        if (value < 0.0f) value = 0.0f;
        pid->Kp = value;
    } else if (strcmp(gain, "KI") == 0) {
        if (value < 0.0f) value = 0.0f;
        pid->Ki = value;
    } else if (strcmp(gain, "KD") == 0) {
        if (value < 0.0f) value = 0.0f;
        pid->Kd = value;
    } else {
        printf("Unknown gain: %s (use KP, KI, or KD)\r\n", gain);
        return 1U;
    }

    printf("SET %s.%s.%s = %.4f\r\n", motor, loop, gain, value);
    return 1U;
}

static uint8_t cmd_try_get_pid(const char *command)
{
    char motor[4] = {0};
    char loop[6] = {0};
    PID_t *pid = NULL;

    if (sscanf(command, "GET %3[^.].%5s", motor, loop) != 2) return 0U;

    pid = cmd_lookup_pid(motor, loop);
    if (pid == NULL) {
        printf("Unknown PID: %s.%s\r\n", motor, loop);
        return 1U;
    }

    printf("%s.%s: KP=%.4f  KI=%.4f  KD=%.4f\r\n", motor, loop, pid->Kp, pid->Ki, pid->Kd);
    return 1U;
}

static void cmd_print_status(void)
{
    EncoderStatus_t up_encoder = Encoder_GetStatus(&encoder_up);
    EncoderStatus_t down_encoder = Encoder_GetStatus(&encoder_down);

    printf("===== FOC STATUS =====\r\n");
    printf("-- UP Motor --\r\n");
    printf("UP.ANG:  KP=%.4f KI=%.4f KD=%.4f\r\n", pid_angle.Kp, pid_angle.Ki, pid_angle.Kd);
    printf("UP.SPD:  KP=%.4f KI=%.4f KD=%.4f\r\n", pid_speed.Kp, pid_speed.Ki, pid_speed.Kd);
    printf("UP.VIS:  KP=%.4f KI=%.4f KD=%.4f\r\n", pid_cloud_y.Kp, pid_cloud_y.Ki, pid_cloud_y.Kd);
    printf("UP.CURD: KP=%.4f KI=%.4f KD=%.4f\r\n",
           FOC_Drive_GetUpCurrentDPid()->Kp, FOC_Drive_GetUpCurrentDPid()->Ki, FOC_Drive_GetUpCurrentDPid()->Kd);
    printf("UP.CURQ: KP=%.4f KI=%.4f KD=%.4f\r\n",
           FOC_Drive_GetUpCurrentQPid()->Kp, FOC_Drive_GetUpCurrentQPid()->Ki, FOC_Drive_GetUpCurrentQPid()->Kd);
    printf("UP.ENC: valid=%u last=%d err=%lu fail=%u recover=%u raw=%u\r\n",
           up_encoder.valid,
           (int)up_encoder.last_status,
           (unsigned long)up_encoder.error_count,
           up_encoder.consecutive_failures,
           up_encoder.recovery_count,
           up_encoder.last_raw_angle);
    printf("-- DOWN Motor --\r\n");
    printf("DN.ANG:  KP=%.4f KI=%.4f KD=%.4f\r\n",
           FOC_Drive_GetDownAnglePid()->Kp, FOC_Drive_GetDownAnglePid()->Ki, FOC_Drive_GetDownAnglePid()->Kd);
    printf("DN.SPD:  KP=%.4f KI=%.4f KD=%.4f\r\n",
           FOC_Drive_GetDownSpeedPid()->Kp, FOC_Drive_GetDownSpeedPid()->Ki, FOC_Drive_GetDownSpeedPid()->Kd);
    printf("DN.VIS:  KP=%.4f KI=%.4f KD=%.4f\r\n", pid_cloud_x.Kp, pid_cloud_x.Ki, pid_cloud_x.Kd);
    printf("DN.CURD: KP=%.4f KI=%.4f KD=%.4f\r\n",
           FOC_Drive_GetDownCurrentDPid()->Kp, FOC_Drive_GetDownCurrentDPid()->Ki, FOC_Drive_GetDownCurrentDPid()->Kd);
    printf("DN.CURQ: KP=%.4f KI=%.4f KD=%.4f\r\n",
           FOC_Drive_GetDownCurrentQPid()->Kp, FOC_Drive_GetDownCurrentQPid()->Ki, FOC_Drive_GetDownCurrentQPid()->Kd);
    printf("DN.ENC: valid=%u last=%d err=%lu fail=%u recover=%u raw=%u\r\n",
           down_encoder.valid,
           (int)down_encoder.last_status,
           (unsigned long)down_encoder.error_count,
           down_encoder.consecutive_failures,
           down_encoder.recovery_count,
           down_encoder.last_raw_angle);
    printf("-- Limits --\r\n");
    printf("SPD:%.4f rad/s  IQ:%.4f A  UQ:%.4f V\r\n",
           FOC_Drive_GetSpeedLimit(), FOC_Drive_GetIqLimit(), FOC_Drive_GetUqLimit());
    printf("-- Vision / Misc --\r\n");
    printf("Center X:%.1f px  Y:%.1f px  Deadzone:%.4f rad\r\n",
           default_control_x, default_control_y, angle_deadzone);
    printf("===== END STATUS =====\r\n");
}

static void cmd_print_help(void)
{
    printf("===== COMMAND HELP =====\r\n");
    printf("--- PID write: SET <M>.<L>.<G>:<v> ---\r\n");
    printf("  M=UP|DN  L=ANG|SPD|VIS|CURD|CURQ  G=KP|KI|KD\r\n");
    printf("--- PID read: GET <M>.<L> ---\r\n");
    printf("--- Limits ---\r\n");
    printf("  SET LIM.SPD|LIM.IQ|LIM.UQ:<v>\r\n");
    printf("  GET LIM\r\n");
    printf("--- Vision / Misc ---\r\n");
    printf("  SET VISC.X|VISC.Y:<v>   GET VISC\r\n");
    printf("  SET DZONE:<rad>          GET DZONE\r\n");
    printf("--- Motor ---\r\n");
    printf("  SET ANGLE:<deg>  TURN  STOP  OL:<spd>:<uq>\r\n");
    printf("  SET EOFFSET:<rad>  SET EOFFSET DEG:<deg>\r\n");
    printf("--- Meta ---\r\n");
    printf("  STATUS  HELP\r\n");
    printf("--- Legacy ---\r\n");
    printf("  SET KP|KI|KPX|KPY:<v>\r\n");
    printf("===== END HELP =====\r\n");
}

void UART_CommandHandler(const char *command)
{
    float temp = 0.0f;
    float speed = 0.0f;
    float uq = 0.0f;

    if (strcmp(command, "STATUS") == 0) {
        cmd_print_status();
    }
    else if (strcmp(command, "HELP") == 0) {
        cmd_print_help();
    }
    else if ((sscanf(command, "SET ANGLE:%f", &temp) == 1) ||
             (sscanf(command, "SET ANGLE %f", &temp) == 1)) {
        target_angle = temp / 180.0f * M_PI;
        printf("Target angle set to: %.2f deg (%.4f rad)\r\n", temp, target_angle);
    }
    else if (strcmp(command, "TURN") == 0) {
        turn_flag = 1U;
        printf("Turn command received\r\n");
    }
    else if (strcmp(command, "STOP") == 0) {
        turn_flag = 0U;
        vision_data.find = 0U;
        vision_frame_ready = 0U;
        FOC_Drive_Stop();
        printf("Stop command received\r\n");
    }
    else if ((sscanf(command, "OL:%f:%f", &speed, &uq) == 2) ||
             (sscanf(command, "OL %f %f", &speed, &uq) == 2)) {
        FOC_Drive_SetOpenLoop(MOTOR_UP, speed, uq);
        printf("Open-loop UP: speed=%.2f rad/s, Uq=%.2f V\r\n", speed, uq);
    }
    else if ((sscanf(command, "SET EOFFSET DEG:%f", &temp) == 1) ||
             (sscanf(command, "SET EOFFSET DEG %f", &temp) == 1)) {
        Encoder_SetElectricalZeroOffset(&encoder_up, temp / 180.0f * M_PI);
        printf("Up electrical zero offset set to: %.2f deg (%.4f rad)\r\n",
               temp, encoder_up.elec_zero_offset_rad);
    }
    else if ((sscanf(command, "SET EOFFSET:%f", &temp) == 1) ||
             (sscanf(command, "SET EOFFSET %f", &temp) == 1)) {
        Encoder_SetElectricalZeroOffset(&encoder_up, temp);
        printf("Up electrical zero offset set to: %.4f rad\r\n", encoder_up.elec_zero_offset_rad);
    }
    else if (sscanf(command, "SET KP:%f", &temp) == 1) {
        pid_angle.Kp = temp;
        printf("Position loop KP set to: %.2f\r\n", pid_angle.Kp);
    }
    else if ((sscanf(command, "SET KPX:%f", &temp) == 1) ||
             (sscanf(command, "SET KPX %f", &temp) == 1)) {
        pid_cloud_x.Kp = temp;
        printf("Vision X loop KP set to: %.2f\r\n", pid_cloud_x.Kp);
    }
    else if ((sscanf(command, "SET KPY:%f", &temp) == 1) ||
             (sscanf(command, "SET KPY %f", &temp) == 1)) {
        pid_cloud_y.Kp = temp;
        printf("Vision Y loop KP set to: %.2f\r\n", pid_cloud_y.Kp);
    }
    else if (sscanf(command, "SET KI:%f", &temp) == 1) {
        pid_angle.Ki = temp;
        printf("Position loop KI set to: %.2f\r\n", pid_angle.Ki);
    }
    else if (cmd_try_set_pid(command)) {
        /* handled by cmd_try_set_pid */
    }
    else if (cmd_try_get_pid(command)) {
        /* handled by cmd_try_get_pid */
    }
    else if ((sscanf(command, "SET LIM.SPD:%f", &temp) == 1) ||
             (sscanf(command, "SET LIM.SPD %f", &temp) == 1)) {
        FOC_Drive_SetSpeedLimit(temp);
        printf("Speed limit set to: %.4f rad/s\r\n", FOC_Drive_GetSpeedLimit());
    }
    else if ((sscanf(command, "SET LIM.IQ:%f", &temp) == 1) ||
             (sscanf(command, "SET LIM.IQ %f", &temp) == 1)) {
        FOC_Drive_SetIqLimit(temp);
        printf("Iq limit set to: %.4f A\r\n", FOC_Drive_GetIqLimit());
    }
    else if ((sscanf(command, "SET LIM.UQ:%f", &temp) == 1) ||
             (sscanf(command, "SET LIM.UQ %f", &temp) == 1)) {
        FOC_Drive_SetUqLimit(temp);
        printf("Uq limit set to: %.4f V\r\n", FOC_Drive_GetUqLimit());
    }
    else if (strcmp(command, "GET LIM") == 0) {
        printf("FOC Limits -- SPD:%.4f rad/s  IQ:%.4f A  UQ:%.4f V\r\n",
               FOC_Drive_GetSpeedLimit(), FOC_Drive_GetIqLimit(), FOC_Drive_GetUqLimit());
    }
    else if ((sscanf(command, "SET VISC.X:%f", &temp) == 1) ||
             (sscanf(command, "SET VISC.X %f", &temp) == 1)) {
        default_control_x = temp;
        printf("Vision center X set to: %.1f px\r\n", default_control_x);
    }
    else if ((sscanf(command, "SET VISC.Y:%f", &temp) == 1) ||
             (sscanf(command, "SET VISC.Y %f", &temp) == 1)) {
        default_control_y = temp;
        printf("Vision center Y set to: %.1f px\r\n", default_control_y);
    }
    else if (strcmp(command, "GET VISC") == 0) {
        printf("Vision Center: X=%.1f px  Y=%.1f px\r\n", default_control_x, default_control_y);
    }
    else if ((sscanf(command, "SET DZONE:%f", &temp) == 1) ||
             (sscanf(command, "SET DZONE %f", &temp) == 1)) {
        angle_deadzone = temp;
        printf("Angle deadzone set to: %.4f rad\r\n", angle_deadzone);
    }
    else if (strcmp(command, "GET DZONE") == 0) {
        printf("Angle deadzone: %.4f rad\r\n", angle_deadzone);
    }
    else {
        printf("Unknown command: %s (type HELP for list)\r\n", command);
    }
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
