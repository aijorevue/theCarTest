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
#include "servo.h"
#include "tim.h"
#include "gpio.h"
#include "sensor.h"
#include "pid.h"
#include "motor.h"
#include "OLED.h"
#include"motor.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

int Get_Track_Error(uint8_t status)
{
    // 此时 status 是原始数据：0 代表黑线，1 代表白纸
    switch (status)
    {
        // --- 正中间 ---
        case 0x09: return 0;   // 1001 (中间两路踩黑线)
        case 0x00: return 0;   // 0000 (全黑，可能是十字路口，暂定直走)
        case 0x0F: return 0;   // 1111 (全白/丢线)

        // --- 偏右 (需左转，返回负值) ---
        case 0x0B: return -1;  // 1011 (只有左二踩线，稍微偏右)
        case 0x03: return -2;  // 0011 (左侧两路踩线，偏右)
        case 0x07: return -3;  // 0111 (只有最左踩线，大幅偏右)
        case 0x02: return -4;  // 0010 (左边三路踩线，极端情况)

        // --- 偏左 (需右转，返回正值) ---
        case 0x0D: return 1;   // 1101 (只有右二踩线，稍微偏左)
        case 0x0C: return 2;   // 1100 (右侧两路踩线，偏左)
        case 0x0E: return 3;   // 1110 (只有最右踩线，大幅偏左)
        case 0x04: return 4;   // 0100 (右边三路踩线，极端情况)
        
        default: return 0;
    }
}


void Execute_Avoidance(void)
{
    OLED_ShowString(48, 40, "AVOID", OLED_8X16);
    OLED_Update();

    // 1. 原地右转 (躲开障碍物)
    Motor_SetSpeed(300, -300); 
    HAL_Delay(500); // 调整时间使转角约 45-90度

    // 2. 直行一段距离 (绕过侧面)
    Motor_SetSpeed(300, 300);
    HAL_Delay(1000);

    // 3. 左转回到平行于黑线的方向
    Motor_SetSpeed(-300, 300);
    HAL_Delay(500);

    // 4. 直行超过障碍物长度
    Motor_SetSpeed(300, 300);
    HAL_Delay(1000);

    // 5. 左转切回黑线
    Motor_SetSpeed(-300, 300);
    HAL_Delay(500);
		//6.继续前进
		Motor_SetSpeed(300, 300);
    HAL_Delay(1000);
		//7.右转
		Motor_SetSpeed(300, -300); 
    HAL_Delay(500);

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  Servo_Init();
  /* USER CODE BEGIN 2 */

  Motor_Init();
  OLED_Init(); 
  OLED_ShowString(0, 0, "Status:", OLED_8X16);
  OLED_ShowString(0, 16, "Error:", OLED_8X16);
  OLED_Update(); 


  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  
  PID_t trackPID;
  PID_Init(&trackPID);
  trackPID.Kp = 20.0f;
  trackPID.Ki = 0.0f;
  trackPID.Kd = 5.0f;
  trackPID.OutMax = 400;
  trackPID.OutMin = -400;

  //for test

  /* USER CODE END 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // 启动舵机1 (PB8)
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // 启动舵机2 (PB9)
  OLED_Clear();
  OLED_ShowString(0, 0,  "Track:", OLED_8X16);
  OLED_ShowString(0, 20, "PID:",   OLED_8X16);
  OLED_ShowString(0, 40, "Mode:",  OLED_8X16);
  OLED_Update();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

     if (Sensor_Read_Avoid_L() == 0 && Sensor_Read_Avoid_R() == 0)
      {
          Execute_Avoidance();
          Servo_SetAngle(TIM_CHANNEL_3, 90); // 让 PB8 的舵机转到 45.5 度
Servo_SetAngle(TIM_CHANNEL_4, 135);  // 让 PB9 的舵机转到 135 度
          continue;
      }

      // 2. 读取 4 路循迹状态
      uint8_t sensor_status = Sensor_Read_Tracking() & 0x0F; 

      if (sensor_status == 0x00) 
      {
          Motor_SetSpeed(0, 0);
          OLED_ShowString(48, 40, "STOP   ", OLED_8X16);
      }
      else 
      {
          OLED_ShowString(48, 40, "RUN  ", OLED_8X16);
          // 3. 计算 PID 并设置电机速度
          int error = Get_Track_Error(sensor_status);
          trackPID.Target = 0;
          trackPID.Actual = (float)error;
          PID_Update(&trackPID);
          
          int BaseSpeed = 300; 
          int speed_L = BaseSpeed + (int)trackPID.Out;
          int speed_R = BaseSpeed - (int)trackPID.Out;
          
          Motor_SetSpeed(speed_L, speed_R);
          OLED_ShowBinNum(48, 0, sensor_status, 5, OLED_8X16); 
          // 第二行显示 PID 输出量 (X=48, Y=20)
          OLED_ShowSignedNum(48, 20, (int)trackPID.Out, 4, OLED_8X16);
      }


    HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
