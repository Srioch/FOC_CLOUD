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
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MYADC.h"
#include "encoder.h"
#include "foc.h"
#include "pid.h"
#include "uart.h"

#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_DT 5.0f
#define OFFSET_TRACK_ENABLE_SPEED_RAD_S 1.0f
#define ADC_STARTUP_SETTLE_DELAY_MS 20U
#define ADC_STARTUP_CALIB_SAMPLES ADC_FILTER_DEFAULT_CALIB_SAMPLES
#define FOC_LOOP_DT_S 0.001f
#define FOC_SPEED_LIMIT_RAD_S 20.0f
#define FOC_UQ_LIMIT_V 6.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t timercount = 0U;
volatile uint8_t vision_counter = 0U;
volatile uint8_t vision_flag = 0U;
static volatile uint8_t flag = 0U;
static volatile uint8_t uart_flag = 0U;

float uq = 0.0f;
float speed_raw = 0.0f;
float speed_filt = 0.0f;
static uint16_t temp = 0U;

VisionData_t frame = {0};
static uint32_t last_vision_tick = 0U;

volatile float Ua = 0.0f;
volatile float Ub = 0.0f;
volatile float Uc = 0.0f;

static ADC_ChannelFilter_t adc_ch4_filter;
static ADC_ChannelFilter_t adc_ch5_filter;
static ADC_FilteredSample_t adc_ch4_sample;
static ADC_FilteredSample_t adc_ch5_sample;
static PhaseVoltageSample_t phase_voltage_sample;
static uint8_t phase_offset_track_enabled = 1U;

static PID_t down_speed_pid;
static PID_t down_angle_pid;
static FocController_t foc_up_controller;
static FocController_t foc_down_controller;
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
  HAL_Init();
  SystemClock_Config();

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
  ADC_ChannelFilter_SetTracking(&adc_ch4_filter,
                                ADC_FILTER_DEFAULT_OFFSET_ALPHA,
                                ADC_FILTER_DEFAULT_TRACK_BAND_RAW,
                                ADC_FILTER_DEFAULT_DRIFT_LIMIT_RAW,
                                ADC_FILTER_DEFAULT_SATURATION_MARGIN_RAW);
  ADC_ChannelFilter_SetTracking(&adc_ch5_filter,
                                ADC_FILTER_DEFAULT_OFFSET_ALPHA,
                                ADC_FILTER_DEFAULT_TRACK_BAND_RAW,
                                ADC_FILTER_DEFAULT_DRIFT_LIMIT_RAW,
                                ADC_FILTER_DEFAULT_SATURATION_MARGIN_RAW);
  ADC_ChannelFilter_SetOutputGain(&adc_ch4_filter, ADC_PHASE_VOLTAGE_DEFAULT_SCALE);
  ADC_ChannelFilter_SetOutputGain(&adc_ch5_filter, ADC_PHASE_VOLTAGE_DEFAULT_SCALE);

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  disableAllPWM();

  HAL_Delay(ADC_STARTUP_SETTLE_DELAY_MS);
  if (ADC_ChannelFilter_Calibrate(&adc_ch4_filter, ADC_STARTUP_CALIB_SAMPLES) != HAL_OK)
  {
    ADC_ChannelFilter_SeedOffsetVoltage(&adc_ch4_filter, ADC_PHASE_VOLTAGE_DEFAULT_OFFSET_V);
  }
  if (ADC_ChannelFilter_Calibrate(&adc_ch5_filter, ADC_STARTUP_CALIB_SAMPLES) != HAL_OK)
  {
    ADC_ChannelFilter_SeedOffsetVoltage(&adc_ch5_filter, ADC_PHASE_VOLTAGE_DEFAULT_OFFSET_V);
  }

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

  PID_InitAngleLoop(&pid_angle, FOC_LOOP_DT_S, FOC_SPEED_LIMIT_RAD_S);
  PID_InitSpeedLoop(&pid_speed, FOC_LOOP_DT_S, FOC_UQ_LIMIT_V);
  PID_InitCloudLoop(&pid_cloud_y, FOC_LOOP_DT_S, FOC_SPEED_LIMIT_RAD_S);

  PID_InitAngleLoop(&down_angle_pid, FOC_LOOP_DT_S, FOC_SPEED_LIMIT_RAD_S);
  PID_InitSpeedLoop(&down_speed_pid, FOC_LOOP_DT_S, FOC_UQ_LIMIT_V);
  PID_InitCloudLoop(&pid_cloud_x, FOC_LOOP_DT_S, FOC_SPEED_LIMIT_RAD_S);

  FocController_Init(&foc_up_controller,
                     &encoder_up,
                     MOTOR_UP,
                     &pid_angle,
                     &pid_speed,
                     &pid_cloud_y,
                     FOC_LOOP_DT_S,
                     FOC_SPEED_LIMIT_RAD_S,
                     FOC_UQ_LIMIT_V);
  FocController_Init(&foc_down_controller,
                     &encoder_down,
                     MOTOR_DOWN,
                     &down_angle_pid,
                     &down_speed_pid,
                     &pid_cloud_x,
                     FOC_LOOP_DT_S,
                     FOC_SPEED_LIMIT_RAD_S,
                     FOC_UQ_LIMIT_V);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (turn_flag != 0U)
    {
      if (phase_offset_track_enabled != 0U)
      {
        ADC_ChannelFilter_EnableTracking(&adc_ch4_filter, 0U);
        ADC_ChannelFilter_EnableTracking(&adc_ch5_filter, 0U);
        phase_offset_track_enabled = 0U;
      }
    }
    else
    {
      float mech_speed_abs = fabsf(Encoder_GetMechanicalVelocity(&encoder_up));

      if (mech_speed_abs <= OFFSET_TRACK_ENABLE_SPEED_RAD_S)
      {
        if (phase_offset_track_enabled == 0U)
        {
          ADC_ChannelFilter_EnableTracking(&adc_ch4_filter, 1U);
          ADC_ChannelFilter_EnableTracking(&adc_ch5_filter, 1U);
          phase_offset_track_enabled = 1U;
        }
      }
      else if (phase_offset_track_enabled != 0U)
      {
        ADC_ChannelFilter_EnableTracking(&adc_ch4_filter, 0U);
        ADC_ChannelFilter_EnableTracking(&adc_ch5_filter, 0U);
        phase_offset_track_enabled = 0U;
      }
    }

    if (vision_flag != 0U)
    {
      vision_flag = 0U;
      if (UART_TryGetVisionFrame(&frame) != 0U)
      {
        last_vision_tick = HAL_GetTick();
      }
    }

    if (flag != 0U)
    {
      if (turn_flag != 0U)
      {
        foc_up_controller.mode = FOC_MODE_POSITION;
        foc_up_controller.target_position_rad = target_angle;
        runFoc(&foc_up_controller);

        foc_down_controller.mode = FOC_MODE_DISABLED;
        runFoc(&foc_down_controller);
      }
      else if ((frame.find != 0U) && ((HAL_GetTick() - last_vision_tick) <= 50U))
      {
        foc_up_controller.mode = FOC_MODE_VISION;
        foc_up_controller.vision_measure = (float)frame.y;
        foc_up_controller.vision_center = DEFAULT_CONTORL_Y;
        foc_up_controller.vision_valid = frame.find;
        runFoc(&foc_up_controller);

        foc_down_controller.mode = FOC_MODE_VISION;
        foc_down_controller.vision_measure = (float)frame.x;
        foc_down_controller.vision_center = DEFAULT_CONTORL_X;
        foc_down_controller.vision_valid = frame.find;
        runFoc(&foc_down_controller);
      }
      else
      {
        frame.find = 0U;
        foc_up_controller.mode = FOC_MODE_DISABLED;
        runFoc(&foc_up_controller);
        foc_down_controller.mode = FOC_MODE_DISABLED;
        runFoc(&foc_down_controller);
      }

      speed_raw = foc_up_controller.state.mechanical_speed_rad_s;
      speed_filt = speed_raw;
      uq = foc_up_controller.state.uq_applied_v;
      temp = (uint16_t)foc_up_controller.mode;
      flag = 0U;

      if (PhaseVoltageSampler_Update(&adc_ch4_filter,
                                     &adc_ch4_sample,
                                     &adc_ch5_filter,
                                     &adc_ch5_sample,
                                     &phase_voltage_sample) == HAL_OK)
      {
        if (phase_voltage_sample.valid != 0U)
        {
          Ua = phase_voltage_sample.ua_v / (20.0f * 0.1f);
          Ub = phase_voltage_sample.ub_v / (20.0f * 0.1f);
          Uc = phase_voltage_sample.uc_v / (20.0f * 0.1f);
        }
      }
    }

    if (uart_flag != 0U)
    {
      uart_flag = 0U;

      printf("%.2f,%.6f,%.6f,%.2f,%.6f,%.6f,%.6f,%d,%d,%d,%d\r\n",
             uq,
             adc_ch4_sample.filtered_voltage_v,
             adc_ch5_sample.filtered_voltage_v,
             foc_up_controller.state.mechanical_angle_rad,
             Ua,
             Ub,
             Uc,
             vision_data.x,
             vision_data.y,
             temp,
             vision_data.find);
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  if (htim->Instance == TIM1)
  {
    flag = 1U;
    if (++timercount >= 10U)
    {
      timercount = 0U;
      uart_flag = 1U;
    }

    if (++vision_counter >= 5U)
    {
      vision_counter = 0U;
      vision_flag = 1U;
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
  __disable_irq();
  while (1)
  {
  }
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
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
