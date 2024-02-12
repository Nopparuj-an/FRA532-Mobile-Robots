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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define M1_duty_offset 31
#define M2_duty_offset 31
#define M3_duty_offset 32
#define M4_duty_offset 31

#define M1_duty_max 1009.0
#define M2_duty_max 1005.0
#define M3_duty_max 1043.0
#define M4_duty_max 1015.0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t duty[4] = { 0 };
uint16_t last_duty[4] = { 0 };

int32_t counter[4] = { 0 };
float compensation[4] = { 1000.0 / M1_duty_max, 1000.0 / M2_duty_max, 1000.0 / M3_duty_max, 1000.0 / M4_duty_max };

uint8_t SPI_rx[12];
uint8_t SPI_tx[12];

uint8_t SPI_status = 0;

static uint32_t count;
HAL_StatusTypeDef status1;
HAL_StatusTypeDef status2;

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
//		status1 = HAL_SPI_Receive(&hspi1, (uint8_t*) &SPI_rx, 1, 10);
//		if (status1 == HAL_OK && SPI_rx[0] == 0b10100101) {
//			count++;
//			SPI_tx[0] = (uint8_t) count;
//			status2 = HAL_SPI_Transmit(&hspi1, (uint8_t*) &SPI_tx, 12, 100);
//		}

		if (SPI_status) {
			status1 = HAL_SPI_Receive(&hspi1, (uint8_t*) &SPI_rx, 1, 10);
			if (status1 == HAL_OK && SPI_rx[0] == 0b10100101) {
				count++;
				SPI_tx[0] = (uint8_t) count;
				for (uint8_t i = 1; i < 12; i++) {
					SPI_tx[i] = i;
				}
				status2 = HAL_SPI_Transmit(&hspi1, (uint8_t*) &SPI_tx, 12, 100);
			}
			SPI_status = 0;
		}

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

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_4) {
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)) {
			SPI_status = 0;
		} else {
			SPI_status = 1;
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	static uint8_t first_run = 0;

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		// Read the IC value
		uint32_t ICValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		if (ICValue != 0) {
			// calculate the Duty Cycle
			uint32_t Duty = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			// Frequency = 72000000 / ICValue;

			if (htim->Instance == TIM1) {
				duty[0] = Duty - M1_duty_offset;
				int32_t delta = duty[0] - last_duty[0];
				if (delta > 500) {
					counter[0] -= delta - M1_duty_max;
				} else if (delta < -500) {
					counter[0] -= delta + M1_duty_max;
				} else {
					counter[0] -= delta;
				}
				last_duty[0] = duty[0];
			} else if (htim->Instance == TIM2) {
				duty[1] = Duty - M2_duty_offset;
				int32_t delta = duty[1] - last_duty[1];
				if (delta > 500) {
					counter[1] += delta - M2_duty_max;
				} else if (delta < -500) {
					counter[1] += delta + M2_duty_max;
				} else {
					counter[1] += delta;
				}
				last_duty[1] = duty[1];
			} else if (htim->Instance == TIM3) {
				duty[2] = Duty - M3_duty_offset;
				int32_t delta = duty[2] - last_duty[2];
				if (delta > 500) {
					counter[2] -= delta - M3_duty_max;
				} else if (delta < -500) {
					counter[2] -= delta + M3_duty_max;
				} else {
					counter[2] -= delta;
				}
				last_duty[2] = duty[2];
			} else if (htim->Instance == TIM4) {
				duty[3] = Duty - M4_duty_offset;
				int32_t delta = duty[3] - last_duty[3];
				if (delta > 500) {
					counter[3] += delta - M4_duty_max;
				} else if (delta < -500) {
					counter[3] += delta + M4_duty_max;
				} else {
					counter[3] += delta;
				}
				last_duty[3] = duty[3];
			}

			if (first_run == 126) {
				first_run = 127;
				counter[0] = 0;
				counter[1] = 0;
				counter[2] = 0;
				counter[3] = 0;
			} else if (first_run != 127) {
				first_run++;
			}
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
