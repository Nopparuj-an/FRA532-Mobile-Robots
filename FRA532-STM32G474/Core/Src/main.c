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
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "math.h"
#include "stdio.h"
#include "string.h"
#include "kalman.h"
#include "arm_math.h"
#include "WS2812B.h"

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
float32_t vel1;
float32_t volt;
int8_t user_input = 0;
uint32_t ms_count = 0;

uint16_t duty[4] = { 0 };
uint16_t last_duty[4] = { 0 };



int32_t counter[4] = { 0 };
int32_t Lcounter;
int32_t last_counter[4] = { 0 };

float compensation[4] = { 1000.0 / M1_duty_max, 1000.0 / M2_duty_max, 1000.0 / M3_duty_max, 1000.0 / M4_duty_max };
float target;
float motor_speed[4] = { 0.0 };
float speedP;
char TxBuffer[40];
float Kp = 5;
float Ki = 20;
float Kd = 0;
KalmanFilter filterA;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint8_t PWMWrite(TIM_HandleTypeDef *htimx, uint16_t tim_chx, float freq, float percent_duty);
uint64_t micros();
void setMotor(uint8_t ID, float dutyCycle);
void oneKilohertz();
void Controller(float target_vel);
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

	Kalman_Start(&filterA);

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
	HAL_TIM_Base_Start_IT(&htim17);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		Set_LED(0, 255, 0, 0);
		Set_LED(1, 255, 255, 255);
		Set_LED(2, 0, 0, 255);
		Set_LED(3, 255, 255, 255);
		Set_LED(4, 255, 0, 0);
		Set_Brightness(5);
		WS2812_Send();
		static uint32_t timestamp = 0;
		if(HAL_GetTick() > timestamp){
		  timestamp = HAL_GetTick() + 2;
		  speedP = (float) (counter[0] - Lcounter) * compensation[0] * 0.5;
		  Lcounter = counter[0];
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

void oneKilohertz() {
	Controller(target);

	if (hlpuart1.gState ==  HAL_UART_STATE_READY)
	{
		sprintf(TxBuffer,"%.2f %.2f %.2f\r\n",motor_speed[0]*2*3.14, vel1, speedP*2*3.14);
		HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)TxBuffer, strlen(TxBuffer));
	}

}


void Controller(float target_vel) {
    static double u_pid = 0;
    static double integral = 0;

    double u_ffw = 0.458 * target_vel + 0.1769;
    double e = target_vel - vel1;

    double proportional = Kp * e;
    integral += Ki * e * 0.001;

    if (fabs(e) < 2.0) {
        integral = 0;
    }

    u_pid = proportional + integral;

    double u = u_ffw + u_pid;

    if (u > 12) {
        u = 12;
    } else if (u < -12) {
        u = -12;
    }
    volt = u;
    setMotor(1, u * 100 / 12);
    vel1 = SteadyStateKalmanFilter(&filterA, u, motor_speed[0] * 2 * 3.14);
}

void Velcalc(){
	// calculate the speed of the motors
	for (int i = 0; i < 4; i++) {
		motor_speed[i] = (float) (counter[i] - last_counter[i]) * compensation[i] * 0.5;
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
			PWMWrite(&htim4, TIM_CHANNEL_3, 2000, 0.0);
			PWMWrite(&htim4, TIM_CHANNEL_4, 2000, dutyCycle);
		} else {
			PWMWrite(&htim4, TIM_CHANNEL_3, 2000, dutyCycle);
			PWMWrite(&htim4, TIM_CHANNEL_4, 2000, 0.0);
		}
		break;
	case 2:
		if (dutyCycle < 0) {
			PWMWrite(&htim3, TIM_CHANNEL_3, 2000, 0.0);
			PWMWrite(&htim3, TIM_CHANNEL_4, 2000, dutyCycle);
		} else {
			PWMWrite(&htim3, TIM_CHANNEL_3, 2000, dutyCycle);
			PWMWrite(&htim3, TIM_CHANNEL_4, 2000, 0.0);
		}
		break;
	case 3:
		if (dutyCycle < 0) {
			PWMWrite(&htim3, TIM_CHANNEL_1, 2000, 0.0);
			PWMWrite(&htim3, TIM_CHANNEL_2, 2000, dutyCycle);
		} else {
			PWMWrite(&htim3, TIM_CHANNEL_1, 2000, dutyCycle);
			PWMWrite(&htim3, TIM_CHANNEL_2, 2000, 0.0);
		}
		break;
	case 4:
		if (dutyCycle < 0) {
			PWMWrite(&htim4, TIM_CHANNEL_1, 2000, dutyCycle);
			PWMWrite(&htim4, TIM_CHANNEL_2, 2000, 0.0);
		} else {
			PWMWrite(&htim4, TIM_CHANNEL_1, 2000, 0.0);
			PWMWrite(&htim4, TIM_CHANNEL_2, 2000, dutyCycle);
		}
		break;
	default:
		break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		ms_count++;
		oneKilohertz();
	}

	if (htim->Instance == TIM17) {
		Velcalc();
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
					counter[1] += delta - M2_duty_max;
				} else if (delta < -500) {
					counter[1] += delta + M2_duty_max;
				} else {
					counter[1] += delta;
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
					counter[3] += delta - M4_duty_max;
				} else if (delta < -500) {
					counter[3] += delta + M4_duty_max;
				} else {
					counter[3] += delta;
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
