/*
 * motors.c
 *
 *  Created on: Apr 18, 2024
 *      Author: nopparuj
 */

#include "motors.h"

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
			PWMWrite(&htim4, TIM_CHANNEL_3, motor_freq, 0.0);
			PWMWrite(&htim4, TIM_CHANNEL_4, motor_freq, dutyCycle);
		} else {
			PWMWrite(&htim4, TIM_CHANNEL_3, motor_freq, dutyCycle);
			PWMWrite(&htim4, TIM_CHANNEL_4, motor_freq, 0.0);
		}
		break;
	case 2:
		if (dutyCycle < 0) {
			PWMWrite(&htim3, TIM_CHANNEL_3, motor_freq, 0.0);
			PWMWrite(&htim3, TIM_CHANNEL_4, motor_freq, dutyCycle);
		} else {
			PWMWrite(&htim3, TIM_CHANNEL_3, motor_freq, dutyCycle);
			PWMWrite(&htim3, TIM_CHANNEL_4, motor_freq, 0.0);
		}
		break;
	case 3:
		if (dutyCycle < 0) {
			PWMWrite(&htim3, TIM_CHANNEL_1, motor_freq, dutyCycle);
			PWMWrite(&htim3, TIM_CHANNEL_2, motor_freq, 0.0);
		} else {
			PWMWrite(&htim3, TIM_CHANNEL_1, motor_freq, 0.0);
			PWMWrite(&htim3, TIM_CHANNEL_2, motor_freq, dutyCycle);
		}
		break;
	case 4:
		if (dutyCycle < 0) {
			PWMWrite(&htim4, TIM_CHANNEL_1, motor_freq, 0.0);
			PWMWrite(&htim4, TIM_CHANNEL_2, motor_freq, dutyCycle);
		} else {
			PWMWrite(&htim4, TIM_CHANNEL_1, motor_freq, dutyCycle);
			PWMWrite(&htim4, TIM_CHANNEL_2, motor_freq, 0.0);
		}
		break;
	default:
		break;
	}
}
