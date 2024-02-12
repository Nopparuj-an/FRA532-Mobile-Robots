// https://controllerstech.com/interface-ws2812-with-stm32/

#ifndef INC_WS2812B_H_
#define INC_WS2812B_H_

#include "stm32g4xx_hal.h"

#define TIMx htim16 // Update the timer instance here

#define MAX_LED 5
#define USE_BRIGHTNESS 1

extern TIM_HandleTypeDef TIMx;
volatile uint8_t datasentflag = 1;

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness

uint16_t pwmData[(24 * MAX_LED) + 50];

void Set_LED(int LEDnum, int Red, int Green, int Blue);
void Set_Brightness(int brightness);
void WS2812_Send(void);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

void Set_LED(int LEDnum, int Red, int Green, int Blue) {
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

void Set_Brightness(int brightness)  // 0-45
{
#if USE_BRIGHTNESS
	if (brightness > 45)
		brightness = 45;
	for (int i = 0; i < MAX_LED; i++) {
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j = 1; j < 4; j++) {
			float angle = 90 - brightness;  // in degrees
			angle = angle * 3.14159265 / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j]) / tan(angle);
		}
	}
#endif
}

void WS2812_Send(void) {
	if (!datasentflag) {
		return;
	}
	uint32_t indx = 0;
	uint32_t color;

	for (int i = 0; i < MAX_LED; i++) {
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1] << 16) | (LED_Mod[i][2] << 8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1] << 16) | (LED_Data[i][2] << 8) | (LED_Data[i][3]));
#endif

		for (int i = 23; i >= 0; i--) {
			if (color & (1 << i)) {
				pwmData[indx] = 83;  // 2/3 of 125
			} else {
				pwmData[indx] = 42;  // 1/3 of 125
			}

			indx++;
		}
	}

	for (int i = 0; i < 50; i++) {
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&TIMx, TIM_CHANNEL_1, (uint32_t*) pwmData, indx);
	datasentflag = 0;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &TIMx) {
		HAL_TIM_PWM_Stop_DMA(&TIMx, TIM_CHANNEL_1);
		datasentflag = 1;
	}
}

#endif /* INC_WS2812B_H_ */
