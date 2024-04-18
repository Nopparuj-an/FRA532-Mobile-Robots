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
#include "usart.h"
#include <stdio.h>
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <geometry_msgs/msg/twist.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint32_t LEDtime = 0;

float cmd_x = 0;
float cmd_y = 0;
float cmd_w = 0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 4000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for secondTask */
osThreadId_t secondTaskHandle;
uint32_t secondTaskBuffer[ 1000 ];
osStaticThreadDef_t secondTaskControlBlock;
const osThreadAttr_t secondTask_attributes = {
  .name = "secondTask",
  .stack_mem = &secondTaskBuffer[0],
  .stack_size = sizeof(secondTaskBuffer),
  .cb_mem = &secondTaskControlBlock,
  .cb_size = sizeof(secondTaskControlBlock),
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void RGB_Rainbow(uint8_t dobreathing);

void twist_callback(const void *msgin);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartSecondTask(void *argument);

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

  /* creation of secondTask */
  secondTaskHandle = osThreadNew(StartSecondTask, NULL, &secondTask_attributes);

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

	rmw_uros_set_custom_transport(true, (void*) &hlpuart1, cubemx_transport_open, cubemx_transport_close,
			cubemx_transport_write, cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate = microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		printf("Error on default allocators (line %d)\n", __LINE__);
	}

	//create init_options
	rclc_executor_t executor;
	rclc_support_t support;
	rcl_allocator_t allocator;
	allocator = rcl_get_default_allocator();
	rclc_support_init(&support, 0, NULL, &allocator);

	// create node
	rcl_node_t node;
	rclc_node_init_default(&node, "STM32_node", "", &support);

	// create timer
	// rcl_timer_t defaultTimer;
	// rclc_timer_init_default(&defaultTimer, &support, RCL_MS_TO_NS(1000), &timerCallback);

	// create messages
	std_msgs__msg__Int64 msg;
	geometry_msgs__msg__Twist twist_msg;

	// sync time
	rmw_uros_sync_session(1000);

	// create publisher/subscriber
	rcl_publisher_t publisher;
	rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), "STM32_status");
	rcl_subscription_t subscriber;
	rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
			"cmd_vel");

	// create executor
	rclc_executor_init(&executor, &support.context, 1, &allocator); // DON'T FOTGET: Increase the number of handles
	// rclc_executor_add_timer(&executor, &defaultTimer);
	rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &twist_callback, ON_NEW_DATA);

	msg.data = 0;

	for (;;) {

		// spin the executor (must do to receive data)
		rclc_executor_spin_some(&executor, 0);

		// msg.data++;
		msg.data = rmw_uros_epoch_millis();
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

		osThreadYield();

	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartSecondTask */
/**
* @brief Function implementing the secondTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSecondTask */
void StartSecondTask(void *argument)
{
  /* USER CODE BEGIN StartSecondTask */
	/* Infinite loop */
	for (;;) {
		RGB_Rainbow(0);
		Set_Brightness(45);
		WS2812_Send();
		osThreadYield();
	}
  /* USER CODE END StartSecondTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void twist_callback(const void *msgin) {
	const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist*) msgin;
	cmd_x = msg->linear.x;
	cmd_w = msg->angular.z;
	cmd_y = msg->linear.y;
}

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

