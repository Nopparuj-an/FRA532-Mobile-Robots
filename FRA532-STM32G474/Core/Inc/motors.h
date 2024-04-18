/*
 * motors.h
 *
 *  Created on: Apr 18, 2024
 *      Author: nopparuj
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

// Configurations
#define CPU_FREQ 170.0 * 1.0e6
#define motor_freq 5000

// Includes
#include "math.h"
#include "stm32g4xx_hal.h"
#include "tim.h"

// Function Declarations
uint8_t PWMWrite(TIM_HandleTypeDef *htimx, uint16_t tim_chx, float freq, float percent_duty);
void setMotor(uint8_t ID, float dutyCycle);

#endif /* INC_MOTORS_H_ */
