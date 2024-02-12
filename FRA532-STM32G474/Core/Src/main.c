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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CPU_FREQ 170.0 * 1.0e6

#define M1_duty_offset 31
#define M2_duty_offset 31
#define M3_duty_offset 32
#define M4_duty_offset 31

#define M1_duty_max 1014.0
#define M2_duty_max 1009.0
#define M3_duty_max 1048.0
#define M4_duty_max 1019.0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int8_t user_input = 0;
uint32_t ms_count = 0;

uint16_t duty[4] = { 0 };
uint16_t last_duty[4] = { 0 };

int32_t counter[4] = { 0 };
int32_t last_counter[4] = { 0 };

float compensation[4] = { 1000.0 / M1_duty_max, 1000.0 / M2_duty_max, 1000.0 / M3_duty_max, 1000.0 / M4_duty_max };

float motor_speed[4] = { 0.0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

uint8_t PWMWrite(TIM_HandleTypeDef *htimx, uint16_t tim_chx, float freq, float percent_duty);
uint64_t micros();
void setMotor(uint8_t ID, float dutyCycle);
void oneKilohertz();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	/* USER CODE BEGIN 2 */

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
	HAL_TIM_Base_Start_IT(&htim6);  // microsecond timer

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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void oneKilohertz() {
	// calculate the speed of the motors
	for (int i = 0; i < 4; i++) {
		motor_speed[i] = (float) (counter[i] - last_counter[i]) * compensation[i];
	}

	// save last counter values
	memcpy(last_counter, counter, sizeof(last_counter));
}

/**
 * @brief Sets the PWM frequency and duty cycle
 * @param htimx TIM handle
 * @param tim_chx TIM channel
 * @param freq PWM frequency
 * @param percent_duty PWM duty cycle
 * @return 0 if successful, -1 if frequency is too high
 */
uint8_t PWMWrite(TIM_HandleTypeDef *htimx, uint16_t tim_chx, float freq, float percent_duty) {
	if (freq == 0) {
		__HAL_TIM_SET_COMPARE(htimx, tim_chx, 0);
		return 0;
	} else if (freq >= CPU_FREQ / 2.0)
		return -1;
	uint32_t period_cyc = (uint32_t) (CPU_FREQ / freq);
	uint16_t prescaler = (uint16_t) (period_cyc / 65535 + 1);
	uint16_t overflow = (uint16_t) ((period_cyc + (prescaler / 2)) / prescaler);
	__HAL_TIM_SET_PRESCALER(htimx, prescaler);
	__HAL_TIM_SET_AUTORELOAD(htimx, overflow);
	__HAL_TIM_SET_COMPARE(htimx, tim_chx, (uint16_t ) (overflow * fabs(percent_duty) / 100.0));
	return 0;
}

void setMotor(uint8_t ID, float dutyCycle) {
	switch (ID) {
	case 1:
		if (dutyCycle < 0) {
			PWMWrite(&htim4, TIM_CHANNEL_3, 20000, 0.0);
			PWMWrite(&htim4, TIM_CHANNEL_4, 20000, dutyCycle);
		} else {
			PWMWrite(&htim4, TIM_CHANNEL_3, 20000, dutyCycle);
			PWMWrite(&htim4, TIM_CHANNEL_4, 20000, 0.0);
		}
		break;
	case 2:
		if (dutyCycle < 0) {
			PWMWrite(&htim3, TIM_CHANNEL_3, 20000, 0.0);
			PWMWrite(&htim3, TIM_CHANNEL_4, 20000, dutyCycle);
		} else {
			PWMWrite(&htim3, TIM_CHANNEL_3, 20000, dutyCycle);
			PWMWrite(&htim3, TIM_CHANNEL_4, 20000, 0.0);
		}
		break;
	case 3:
		if (dutyCycle < 0) {
			PWMWrite(&htim3, TIM_CHANNEL_1, 20000, 0.0);
			PWMWrite(&htim3, TIM_CHANNEL_2, 20000, dutyCycle);
		} else {
			PWMWrite(&htim3, TIM_CHANNEL_1, 20000, dutyCycle);
			PWMWrite(&htim3, TIM_CHANNEL_2, 20000, 0.0);
		}
		break;
	case 4:
		if (dutyCycle < 0) {
			PWMWrite(&htim4, TIM_CHANNEL_1, 20000, dutyCycle);
			PWMWrite(&htim4, TIM_CHANNEL_2, 20000, 0.0);
		} else {
			PWMWrite(&htim4, TIM_CHANNEL_1, 20000, 0.0);
			PWMWrite(&htim4, TIM_CHANNEL_2, 20000, dutyCycle);
		}
		break;
	default:
		break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		static uint32_t last_time = 0;
		uint32_t time = HAL_GetTick();
		if (time - last_time < 200)
			return;
		last_time = time;

		switch (user_input) {
		default:
		case 0:
			setMotor(1, 25.0);
			setMotor(2, 25.0);
			setMotor(3, -25.0);
			setMotor(4, -25.0);
			user_input = 1;
			break;
		case 1:
			setMotor(1, 100.0);
			setMotor(2, 100.0);
			setMotor(3, -100.0);
			setMotor(4, -100.0);
			user_input = 2;
			break;
		case 2:
			setMotor(1, -25.0);
			setMotor(2, -25.0);
			setMotor(3, 25.0);
			setMotor(4, 25.0);
			user_input = 3;
			break;
		case 3:
			setMotor(1, -100.0);
			setMotor(2, -100.0);
			setMotor(3, 100.0);
			setMotor(4, 100.0);
			user_input = 4;
			break;
		case 4:
			setMotor(1, 0.0);
			setMotor(2, 0.0);
			setMotor(3, 0.0);
			setMotor(4, 0.0);
			user_input = 0;
			break;
		}
	}
}

uint64_t micros() {
	return ms_count * 1000 + __HAL_TIM_GET_COUNTER(&htim6);
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
					counter[0] += delta - M1_duty_max;
				} else if (delta < -500) {
					counter[0] += delta + M1_duty_max;
				} else {
					counter[0] += delta;
				}
				last_duty[0] = duty[0];
			} else if (htim->Instance == TIM8) {
				duty[1] = Duty - M2_duty_offset;
				int32_t delta = duty[1] - last_duty[1];
				if (delta > 500) {
					counter[1] -= delta - M2_duty_max;
				} else if (delta < -500) {
					counter[1] -= delta + M2_duty_max;
				} else {
					counter[1] -= delta;
				}
				last_duty[1] = duty[1];
			} else if (htim->Instance == TIM15) {
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
			} else if (htim->Instance == TIM20) {
				duty[3] = Duty - M4_duty_offset;
				int32_t delta = duty[3] - last_duty[3];
				if (delta > 500) {
					counter[3] -= delta - M4_duty_max;
				} else if (delta < -500) {
					counter[3] -= delta + M4_duty_max;
				} else {
					counter[3] -= delta;
				}
				last_duty[3] = duty[3];
			}

			if (first_run < 127) {
				first_run++;
				counter[0] = 0;
				counter[1] = 0;
				counter[2] = 0;
				counter[3] = 0;
			}
		}
	}
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	if (htim->Instance == TIM6) {
		ms_count++;
		oneKilohertz();
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
void Error_Handler(void) {
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
