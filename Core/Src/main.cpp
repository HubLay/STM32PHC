/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "520Motor.h"
#include "Kalman.h"

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
MPU6050 PHC;
Motor_520 Left_Motor, Right_Motor;
float Pitch, ACC, Speed;
int16_t Left_Out, Right_Out;
int8_t Cnt = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim3)
  {
    HAL_TIM_Base_Stop_IT(&htim3);
    PHC.get_ID();
    PHC.get_ACCEL();
    PHC.get_GYRO();
    PHC.Updata_TrueData();
    PHC.Updata_ACCAngle();

    ACC = PHC.get_ACC();
    Pitch = Kalman_fifter(PHC.TrueGYRO[0], PHC.ACCAngle[0], ACC);


    Left_Motor.Omega_Updata();
    Right_Motor.Omega_Updata();

    Speed = (Left_Motor.Get_Speed() + (-Right_Motor.Get_Speed()))/2.0f;

    Left_Motor.Set_Speed(Speed);
    Right_Motor.Set_Speed(-Speed);

    Left_Motor.PID_Angle.Set_GYRO(PHC.TrueGYRO[0]);
    Right_Motor.PID_Angle.Set_GYRO(PHC.TrueGYRO[0]);

    Left_Motor.Angle_Updata(Pitch);
    Right_Motor.Angle_Updata(Pitch);

    // Left_Motor.PID_Angle.Set_Target(0.0f);      //记得加个死区，KLM得调，趋近有点慢
    // Right_Motor.PID_Angle.Set_Target(0.0f);

    // sprintf(mes,"%f\n\r",Pitch);
    // HAL_UART_Transmit_DMA(&huart3,(uint8_t *)mes,strlen(mes));
    Left_Motor.Calculate_PID();
    //Right_Motor.Calculate_PID();

    Left_Out = Left_Motor.Get_Out();
    Right_Out = Left_Motor.Get_Out();
    if (Left_Out > 0)
    {
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);

      Left_Out += 37; //21
    }
    else
    {
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);

      Left_Out = -Left_Out + 35;  //25
    }
    if (Right_Out > 0)
    {
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);

      Right_Out += 32;      //26
    }
    else
    {
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);

      Right_Out = -Right_Out + 34;  //30
    }
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,Right_Out);   //R
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,Left_Out);   //L
  }

  HAL_TIM_Base_Start_IT(&htim3);
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
int PWM = 26;

// L 25
// R 30

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  PHC.MPU6050_Init(&hi2c1);
  PHC.MPU6050_ACC_Calibration();
  PHC.MPU6050_GYRO_Calibration();

  Left_Motor.Init(&htim2);
  Right_Motor.Init(&htim4);

  Left_Motor.PID_Omega.Init(-6.2f, -6.2 / 200.0f, 8.0f, 2); // 加个KD，有抖动，目前KP，KI越大，越快稳定，但抖动会加大
  //Left_Motor.PID_Omega.Init(0.0f, 0.0f, 0.0f, 2);
  Left_Motor.PID_Angle.Init(-35.8*0.6f, 0.0f, 4.0*0.6f, 1);
  Right_Motor.PID_Omega.Init(0.0f, 0.0f, 0.0f, 2);
  Right_Motor.PID_Angle.Init(-0.0f, 0.0f, 0.0f, 1);
  //Right_Motor.PID_Angle.Init(-46.8*0.6f, -0.0f, 0.9*0.6f);

  // Left_Motor.PID_Angle.Init(-0.0f, -0.0f, 0.0f);
  // Right_Motor.PID_Angle.Init(-0.0f, -0.0f, 0.0f);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim3);

  // Left_Motor.PID_Omega.Set_Target(0.0f);
  // Right -1
  // 正转
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);

  // 正转 R
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);

  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,36);   //R
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,40);   //L
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
