/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc.h"
#include "pid.h"
#include "encoder.h"
#include "uart.h"
#include "kalman_filter.h"
#include "MYADC.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_DT 5.0f // ms


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t timercount = 0;
volatile uint8_t vision_counter = 0;
volatile uint8_t vision_flag = 0U;
static kalman_filter_pos_speed_t kf_speed;
static Encoder_t encoder;
static volatile uint8_t flag = 0U;
static volatile uint8_t uart_flag = 0U;
float uq;
float speed_raw = 0.0f;
float speed_filt = 0.0f;
static uint8_t speed_loop_div = 0U;
static uint16_t temp = 0U;
static VisionControl_t vc_up;
static VisionControl_t vc_down;
VisionData_t frame;
static uint32_t last_vision_tick = 0U;
volatile float Ua, Ub, Uc;
static ADC_ChannelFilter_t adc_ch4_filter;
static ADC_ChannelFilter_t adc_ch5_filter;
static ADC_FilteredSample_t adc_ch4_sample;
static ADC_FilteredSample_t adc_ch5_sample;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  ADC_ChannelFilter_Init(&adc_ch4_filter, &hadc1, ADC_CHANNEL_4, ADC_SAMPLETIME_144CYCLES,
                         ADC_FILTER_DEFAULT_WINDOW, ADC_FILTER_DEFAULT_ALPHA);
  ADC_ChannelFilter_Init(&adc_ch5_filter, &hadc1, ADC_CHANNEL_5, ADC_SAMPLETIME_144CYCLES,
                         ADC_FILTER_DEFAULT_WINDOW, ADC_FILTER_DEFAULT_ALPHA);

  if (ADC_ChannelFilter_Calibrate(&adc_ch4_filter, ADC_FILTER_DEFAULT_CALIB_SAMPLES) != HAL_OK)
  {
    Error_Handler();
  }

  if (ADC_ChannelFilter_Calibrate(&adc_ch5_filter, ADC_FILTER_DEFAULT_CALIB_SAMPLES) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  UART_SetHandle(&huart1);
  UART_StartReceiveIT(&huart1);
  UART_StartReceiveIT(&huart2);
  Encoder_Init(&encoder_up, &hi2c1, AS5600_RESOLUTION, FOC_POLE_PAIRS);
  HAL_Delay(300);
  Encoder_Init(&encoder_down, &hi2c2, AS5600_RESOLUTION, FOC_POLE_PAIRS);
  HAL_Delay(300);
  if (Encoder_Start(&encoder_up) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_Delay(300);
  if (Encoder_Start(&encoder_down) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_Delay(1000);
  kalman_filter_pos_speed_Init(&kf_speed);
  PID_InitSpeedLoop(&pid_speed, 0.001f, 6.0f);
  PID_InitCloudLoop(&pid_cloud_x, 0.001f, 6.0f);
  PID_InitCloudLoop(&pid_cloud_y, 0.001f, 6.0f);
  VisionControl_Init(&vc_up , &pid_speed, &kf_speed, &encoder_up, MOTOR_UP);
  VisionControl_Init(&vc_down , &pid_speed, &kf_speed, &encoder_down, MOTOR_DOWN);
  PID_InitCloudLoop(&vc_up.pid_vis, 0.005f, 20.0f);
  PID_InitCloudLoop(&vc_down.pid_vis, 0.005f, 20.0f); 
  PID_InitSpeedLoop(&vc_up.pid_speed, 0.001f, 6.0f);
  PID_InitSpeedLoop(&vc_down.pid_speed, 0.001f, 6.0f);
  
  pid_cloud_x.Kp = PID_CLOUD_KP_DEFAULT;
  pid_cloud_y.Kp = PID_CLOUD_KP_DEFAULT;
  kf_speed.R_measure = 0.01f;   /* 编码器位置测量噪声 */
  kf_speed.Q_speed = 0.2f;      /* 速度过程噪声，越大响应越快 */
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

     if(flag)
    {
      if(turn_flag)
        //
        {
          
          SpeedLoop_Update(&encoder_up, &kf_speed, 0.005f, 10.0f);
          
          
          /* if (UART_TryGetVisionFrame(&frame))
          {
              last_vision_tick = HAL_GetTick();

              VisionOuterLoop(&vc_up,   (float)frame.y, DEFAULT_CONTORL_Y,frame.find, 0.005f);
              VisionOuterLoop(&vc_down, (float)frame.x, DEFAULT_CONTORL_X,frame.find, 0.005f);
          }

          if ((HAL_GetTick() - last_vision_tick) > 50U)
          {
              vc_up.speed_ref = 0.0f;
              vc_down.speed_ref = 0.0f;
              PID_Reset(&vc_up.pid_vis);
              PID_Reset(&vc_down.pid_vis);
              PID_Reset(&vc_up.pid_speed);
              PID_Reset(&vc_down.pid_speed);
          }

          temp = MotorSpeedLoop_Update(&vc_up, 0.001f);
          MotorSpeedLoop_Update(&vc_down, 0.001f); */
      }
      else
      {
          vc_up.speed_ref = 0.0f;
          vc_down.speed_ref = 0.0f;
      }

     

      flag = 0;
    }
    

    if(uart_flag)
    {
      uart_flag = 0;
 
      if (ADC_ChannelFilter_Read(&adc_ch4_filter, &adc_ch4_sample) != HAL_OK)
      {
        Error_Handler();
      }

      if (ADC_ChannelFilter_Read(&adc_ch5_filter, &adc_ch5_sample) != HAL_OK)
      {
        Error_Handler();
      }

      /* Ua = (1.67000f-adc_ch4_sample.filtered_voltage_v) * 10;
      Ub = (1.67000f-adc_ch5_sample.filtered_voltage_v) * 10; */
      Ua = adc_ch4_sample.corrected_voltage_v;
      Ub = adc_ch5_sample.corrected_voltage_v; 
      Uc = -(Ua + Ub);

      int64_t count = Encoder_GetTotalCount(&encoder_up);
      float angle = Encoder_GetMechanicalAngle(&encoder_up);
      float elec_angle = Encoder_GetElectricalAngle(&encoder_up);
      //uq, speed_raw, speed_filt(rad/s), kf_pos(rad), mech/elec angle(rad), kp, ki
      printf("%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%d,%d,%d,%d\r\n",
             -uq,
             adc_ch4_sample.filtered_voltage_v,
             adc_ch5_sample.filtered_voltage_v,
             /* speed_raw,
             speed_filt, */
             kf_speed.position,
             Ua,
             Ub,
             Uc,
            vision_data.x,
            vision_data.y,
            temp,
            vision_data.find
             );
    }
      
   
    UART_ProcessPendingCommand();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) // 1ms 定时器
    {
        flag = 1;
        if (++timercount >= 10) // 100ms
        {
            timercount = 0;
            uart_flag = 1U; // 每 100ms 发送一次数据
        }

        if(++vision_counter >= 5) // 10ms
        {
            vision_counter = 0;
            vision_flag = 1U; // 每 10ms 设置一次视觉处理标志
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
