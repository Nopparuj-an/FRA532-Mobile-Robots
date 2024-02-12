/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "math.h"
#include "arm_math.h"
#include "WS2812B.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

void RGB_Rainbow(uint8_t dobreathing);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint32_t LEDtime = 0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 5000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;) {
		//osDelay(1);
		RGB_Rainbow(0);
		Set_Brightness(45);
		WS2812_Send();
		osThreadYield();
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void RGB_Rainbow(uint8_t dobreathing) {
	static uint32_t startTime = 0;
	static const uint32_t transitionDuration = 5000; // Transition duration in milliseconds

	if (HAL_GetTick() - startTime >= transitionDuration) {
		startTime = HAL_GetTick();
	}

	// Calculate time elapsed in the current transition
	uint32_t elapsed = HAL_GetTick() - startTime;

	// Calculate the normalized progress (0.0 to 1.0) within the transition
	float progress = (float) elapsed / transitionDuration;

	// Calculate the hue angle based on the progress
	float hueAngle = 360.0f * progress;

	// Set LED colors based on the hue angle
	for (int i = 0; i < MAX_LED; i++) {
		// Calculate the hue value for the current LED
		float ledHue = hueAngle + (i * (360.0f / MAX_LED));

		// Convert hue to RGB using HSV color model
		float huePrime = fmodf(ledHue / 60.0f, 6.0f);
		float chroma = 1.0f;
		float x = chroma * (1.0f - fabsf(fmodf(huePrime, 2.0f) - 1.0f));

		float red, green, blue;

		if (huePrime >= 0.0f && huePrime < 1.0f) {
			red = chroma;
			green = x;
			blue = 0.0f;
		} else if (huePrime >= 1.0f && huePrime < 2.0f) {
			red = x;
			green = chroma;
			blue = 0.0f;
		} else if (huePrime >= 2.0f && huePrime < 3.0f) {
			red = 0.0f;
			green = chroma;
			blue = x;
		} else if (huePrime >= 3.0f && huePrime < 4.0f) {
			red = 0.0f;
			green = x;
			blue = chroma;
		} else if (huePrime >= 4.0f && huePrime < 5.0f) {
			red = x;
			green = 0.0f;
			blue = chroma;
		} else {
			red = chroma;
			green = 0.0f;
			blue = x;
		}

		// slow fade in
		if (LEDtime == 0) {
			LEDtime = HAL_GetTick();
		}

		float intensity;
		if (HAL_GetTick() - LEDtime < 4000) {
			intensity = (HAL_GetTick() - LEDtime) / 4000.0;
		} else {
			intensity = 1;
		}

		intensity = sqrt(intensity);

		// breathing pattern
		float intensity2;
		if (dobreathing) {
			intensity2 = 0.1 + 0.9 * (0.5 * (1.0 + sinf((2.0 * M_PI * elapsed) / 2000)));
		} else {
			intensity2 = 1.0;
		}

		// Scale RGB values to 0-255 range
		uint8_t r = (uint8_t) (red * 255.0 * intensity * intensity2);
		uint8_t g = (uint8_t) (green * 255.0 * intensity * intensity2);
		uint8_t b = (uint8_t) (blue * 255.0 * intensity * intensity2);

		// Set LED color
		Set_LED(i, r, g, b);
	}
}

/* USER CODE END Application */

