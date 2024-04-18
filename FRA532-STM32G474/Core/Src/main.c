/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "math.h"
#include "arm_math.h"
#include "motors.h"
#include "encoder.h"
#include "string.h"
#include "kalman.h"
#include "config+var.h"

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

uint32_t ms_count = 0;

KalmanFilter filterA;
KalmanFilter filterB;
KalmanFilter filterC;
KalmanFilter filterD;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

uint64_t micros();
void Controller(uint8_t motor_ID[], float target_vel[], int num_motors) ;

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
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  MX_TIM20_Init();
  MX_TIM8_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

	// Initialize Kalman filters
	Kalman_Start(&filterA);
	Kalman_Start(&filterB);
	Kalman_Start(&filterC);
	Kalman_Start(&filterD);

	// Start PWM outputs
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	// Start PWM outputs
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim15, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim20, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim20, TIM_CHANNEL_2);

	// Start timer for other functions
	HAL_TIM_Base_Start_IT(&htim6);  // microsecond timer, 1 kHz
	HAL_TIM_Base_Start_IT(&htim17); // Calculate speed timer, 100 Hz

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Controller(uint8_t motor_ID[], float target_vel[], int num_motors) {
	for (int i = 0; i < num_motors; i++) {
		static double u_pid = 0;
		static double integral[4];

		double e[4];
		double u_ffw = 0.458 * target_vel[i];
		e[i] = target_vel[i] - vel[i];

		double proportional = Kp * e[i];
		integral[i] += Ki * e[i] * 0.001;

		if (fabs(e[i]) < 0.2) {
			integral[i] = 0;
		}

		u_pid = proportional + integral[i];

		double u = u_ffw + u_pid;

		if (u > 12) {
			u = 12;
		} else if (u < -12) {
			u = -12;
		}
		volt[i] = u;
		setMotor(motor_ID[i], u * 100 / 12);

		if (motor_ID[i] == 1) {
			vel[i] = SteadyStateKalmanFilter(&filterA, volt[i], motor_speed[i] * 2 * 3.14);
			float ekalmanvel = fabs(motor_speed[i] - vel[i]);
			if (ekalmanvel > 0.67 * volt[i]) {
				filterA.R[0] = 300 - ekalmanvel * 10;
			} else {
				filterA.R[0] = 300 - ekalmanvel * 1000;
			}

			if (filterA.R[0] < 0) {
				filterA.R[0] = 0.001;
			}
		}

		else if (motor_ID[i] == 2) {
			vel[i] = SteadyStateKalmanFilter(&filterB, volt[i], motor_speed[i] * 2 * 3.14);
			float ekalmanvel = fabs(motor_speed[i] - vel[i]);
			if (ekalmanvel > 0.67 * volt[i]) {
				filterB.R[0] = 300 - ekalmanvel * 10;
			} else {
				filterB.R[0] = 300 - ekalmanvel * 1000;
			}

			if (filterB.R[0] < 0) {
				filterB.R[0] = 0.001;
			}
		}

		else if (motor_ID[i] == 3) {
			vel[i] = SteadyStateKalmanFilter(&filterC, volt[i], motor_speed[i] * 2 * 3.14);
			float ekalmanvel = fabs(motor_speed[i] - vel[i]);
			if (ekalmanvel > 0.67 * volt[i]) {
				filterC.R[0] = 300 - ekalmanvel * 10;
			} else {
				filterC.R[0] = 300 - ekalmanvel * 1000;
			}

			if (filterC.R[0] < 0) {
				filterC.R[0] = 0.001;
			}
		}

		else if (motor_ID[i] == 4) {
			vel[i] = SteadyStateKalmanFilter(&filterD, volt[i], motor_speed[i] * 2 * 3.14);
			float ekalmanvel = fabs(motor_speed[i] - vel[i]);
			if (ekalmanvel > 0.67 * volt[i]) {
				filterD.R[0] = 300 - ekalmanvel * 10;
			} else {
				filterD.R[0] = 300 - ekalmanvel * 1000;
			}

			if (filterD.R[0] < 0) {
				filterD.R[0] = 0.001;
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		static uint32_t last_time = 0;
		uint32_t time = HAL_GetTick();
		if (time - last_time < 200)
			return;
		last_time = time;

		// debounced button
		// insert debug code here
	}
}

uint64_t micros() {
	return ms_count * 1000 + __HAL_TIM_GET_COUNTER(&htim6);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	encoderCallback(htim);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

	if (htim->Instance == TIM6) {
		// 1 kHz loop
		ms_count++;
		Controller(motor_ID, target_vel, 4);
	}

	if (htim->Instance == TIM17) {
		calculateMotorSpeed();
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
