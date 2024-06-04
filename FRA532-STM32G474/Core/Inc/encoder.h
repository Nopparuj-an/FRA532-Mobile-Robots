/*
 * encoder.h
 *
 *  Created on: Apr 18, 2024
 *      Author: nopparuj
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

// Includes
#include <config+var.h>
#include <stm32g4xx_hal.h>
#include "tim.h"
#include "string.h"

// Function prototypes
void calculateMotorSpeed();
void encoderCallback(TIM_HandleTypeDef *htim);

#endif /* INC_ENCODER_H_ */
