/*
 * encoder.c
 *
 *  Created on: Apr 18, 2024
 *      Author: nopparuj
 */

#include "encoder.h"

void calculateMotorSpeed() {
	// calculate the speed of the motors
	for (int i = 0; i < 4; i++) {
		motor_speed[i] = (float) (counter[i] - last_counter[i]) * compensation[i] * 0.1;
	}

	// save last counter values
	memcpy(last_counter, counter, sizeof(last_counter));
}

void encoderCallback(TIM_HandleTypeDef *htim) {
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
