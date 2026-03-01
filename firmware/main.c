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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    float Kp, Ki, Kd, Kf;
    float error_prev;
    float integral;
} PID_TypeDef;

typedef enum {
    PARK_IDLE,
    PARK_SEARCHING,
    PARK_POSITIONING,
    PARK_REVERSING,
    PARK_FINISHED
} ParkingState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BNO055_ADDR (0x28 << 1)  //
#define BNO055_OPR_MODE_REG 0x3D
#define BNO055_EULER_H_LSB 0x1A // yaw register
#define NDOF_MODE 0x0C
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
IWDG_HandleTypeDef hiwdg;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t park_wait_tick = 0;
uint8_t gap_counter = 0;
float current_setpoint = 0;
#define ACCEL_STEP 2.5f
uint32_t blinker_tick = 0;
uint8_t rpi_rx_buffer[9];
uint32_t last_rpi_tick = 0;
uint8_t us100_buffer[2];
uint16_t distance_mm = 0;
int32_t encoder_count = 9;
float target_speed = 0;
float target_angle = 0;
PID_TypeDef motorPID = {1.2f, 0.5f, 0.01f, 1500.0f};
ParkingState current_park_state = PARK_IDLE;
float actual_speed = 0;
uint32_t servo_pwm = 1500; // 1500 is center
uint32_t motor_pwm = 1500; // Neutral
float current_yaw = 0.0f;
float start_yaw = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float get_distance_mm() {
    int32_t ticks = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    return (ticks / 330.0f) * 204.2f; //circumference of wheel
}

float ComputePID(float target, float actual, PID_TypeDef *pid) {
    float error = target - actual;
    pid->integral += error * 0.01f;
    
    // Anti-windup: Stop integral from growing too large if car is stuck
    if(pid->integral > 500) pid->integral = 500;
    if(pid->integral < -500) pid->integral = -500;

    float derivative = (error - pid->error_prev) / 0.01f;
    
    // Output = FeedForward + PID terms
    float output = pid->Kf + (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
    pid->error_prev = error;

    // ESC Safety Clamps (Standard RC range is 1000us to 2000us)
    if(output > 2000) output = 2000;
    if(output < 1000) output = 1000;
    
    return output;
}

void BNO055_Init() {
    uint8_t data = NDOF_MODE;
    HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDR, BNO055_OPR_MODE_REG, 1, &data, 1, 100);
    HAL_Delay(50);
}

void Read_BNO055_Yaw() {
    uint8_t buffer[2];
    if (HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO055_EULER_H_LSB, 1, buffer, 2, 100) == HAL_OK) {
        int16_t raw_yaw = (buffer[1] << 8) | buffer[0];
        current_yaw = raw_yaw / 16.0f; // BNO055 returns 1/16th degrees
    }
}

void Send_Telemetry() {
    char debug_msg[64];
    // Format: Speed, Yaw, Distance, Current State
    int len = sprintf(debug_msg, "V:%.1f Y:%.1f D:%d S:%d\r\n",
                      actual_speed, current_yaw, distance_mm, current_park_state);

    // Send via USART1 (The RPi connection)
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, len, 100);
}
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_UART_Receive_DMA(&huart1,rpi_rx_buffer,8);
  BNO055_Init();
  
  HAL_IWDG_Init(&hiwdg);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
        /* 1. SENSOR UPDATES (Orientation) */
        Read_BNO055_Yaw();

        /* 2. ULTRASONIC TRIGGER (Non-Blocking) */
        // We trigger the US-100 every 100ms. 
        // The result is handled automatically by HAL_UART_RxCpltCallback.
        static uint32_t last_us_trigger = 0;
        if(HAL_GetTick() - last_us_trigger > 100) {
            uint8_t trigger = 0x55;
            HAL_UART_Transmit(&huart2, &trigger, 1, 10);
            HAL_UART_Receive_IT(&huart2, us100_buffer, 2); 
            last_us_trigger = HAL_GetTick();
        }

        /* 3. BFMC PARKING STATE MACHINE */
        switch (current_park_state) {
            case PARK_IDLE:
                // Standing by for RPi command to start searching
                break;

            case PARK_SEARCHING:
                target_speed = 100.0f; // Drive slowly at 100mm/s
                target_angle = 0;      // Keep wheels straight
                
                // If the side sensor sees a gap larger than 500mm
                if (distance_mm > 500) {
                      gap_counter++;
                  } else {
                      gap_counter = 0;
                  }

                  if (gap_counter > 5) { // 5 consecutive readings
                      __HAL_TIM_SET_COUNTER(&htim2, 0);
                      gap_counter = 0;
                      current_park_state = PARK_POSITIONING;
                  }
                break;

            case PARK_POSITIONING:
                // Drive forward until we clear the parking spot (approx 300mm)
                if (get_distance_mm() > 300.0f) { 
                    target_speed = 0;
                    start_yaw = current_yaw;
                    park_wait_tick = HAL_GetTick();
                    current_park_state = PARK_REVERSING;
                }
                break;

            case PARK_REVERSING:
                target_angle = -30.0f;  // Hard steer into the spot
                target_speed = -100.0f; // Reverse slowly
                if (HAL_GetTick() - park_wait_tick < 500) {
                  target_speed = 0;
                  break;
                }
                // Calculate relative turn angle (handling 360 wrap-around)
                float turned = current_yaw - start_yaw;
                if (turned > 180)  turned -= 360; 
                if (turned < -180) turned += 360;

                // Stop once the car has rotated 45 degrees
                if (turned >= 45.0f || turned <= -45.0f) { 
                    current_park_state = PARK_FINISHED;
                }
                break;

            case PARK_FINISHED:
                target_speed = 0;
                target_angle = 0; // Straighten wheels
                break;
        }

        /* 4. DATA FEEDBACK */
        // Send Speed, Yaw, and Distance back to Raspberry Pi for processing
        Send_Telemetry();

        /* 5. LOOP STABILITY */
        // A small delay to keep the loop from hogging 100% CPU, 
        // allowing the interrupts (DMA/Timers) to breathe.
        HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 1250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 839;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
      HAL_IWDG_Refresh(&hiwdg);
        if (HAL_GetTick() - last_rpi_tick > 500 && current_park_state == PARK_IDLE) {
          target_speed = 0;
          current_setpoint = 0;
        }
        // A. Speed Ramping (Anti-Jerk)
        if (current_setpoint < target_speed) current_setpoint += ACCEL_STEP;
        else if (current_setpoint > target_speed) current_setpoint -= ACCEL_STEP;

        // B. Odometry (Encoder Calculation)
        int32_t ticks = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        actual_speed = ((float)ticks / 330.0f) * 204.2f * 100.0f;
        if (target_speed < 0) actual_speed = -actual_speed;
        if (actual_speed - target_speed > 50) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // Brake ON
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        }
        // C. BFMC Safety Stop
        if (distance_mm < 150 && target_speed > 0) {
            target_speed = 0; 
            current_setpoint = 0; // Immediate override
        }

        // D. PID Motor Control
        if (target_speed != 0 || actual_speed > 5) {
            motor_pwm = (uint32_t)ComputePID(current_setpoint, actual_speed, &motorPID);
        } else {
            motor_pwm = 1500; // Hard neutral
            motorPID.integral = 0;
        }
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_pwm);

        // E. Servo & BFMC Signaling (Blinkers/Brakes)
        if (target_angle > 30.0f) target_angle = 30.0f;
        if (target_angle < -30.0f) target_angle = -30.0f;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1500 + (target_angle * 16.6f));
        
        blinker_tick++;
        if ((blinker_tick % 25) == 0) { // Every 250ms
            if (target_angle > 15.0f) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1); // Right Signal
            if (target_angle < -15.0f) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2); // Left Signal
        }
    }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    /* ---------- US-100 Ultrasonic ---------- */
    if (huart->Instance == USART2) {
        distance_mm = (us100_buffer[0] << 8) | us100_buffer[1];
        if (distance_mm == 0 || distance_mm > 3000) {
            distance_mm = 3000; // treat as "no obstacle"
        }
        // Restart RX for next trigger
        HAL_UART_Receive_IT(&huart2, us100_buffer, 2);
    }

    /* ---------- Raspberry Pi ---------- */
    if (huart->Instance == USART1) {
        rpi_rx_buffer[8] = '\0';

        if (rpi_rx_buffer[0] == 'A') {
            sscanf((char*)rpi_rx_buffer, "A%f", &target_angle);
        }
        else if (rpi_rx_buffer[0] == 'P') {
            current_park_state = PARK_SEARCHING;
        }
        else if (rpi_rx_buffer[0] == 'E') {
            target_speed = 0;
            current_setpoint = 0;
        }

        last_rpi_tick = HAL_GetTick();
        HAL_UART_Receive_DMA(&huart1, rpi_rx_buffer, 8);
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
