/*
 * motors.h
 *
 *  Created on: Apr 18, 2024
 *      Author: nopparuj
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

// Includes
#include <stm32g4xx_hal.h>
#include "config.h"
#include "math.h"
#include "tim.h"

// Function Declarations
uint8_t PWMWrite(TIM_HandleTypeDef *htimx, uint16_t tim_chx, float freq, float percent_duty);
void setMotor(uint8_t ID, float dutyCycle);

#endif /* INC_MOTORS_H_ */
