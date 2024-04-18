/*
 * configs.h
 *
 *  Created on: Apr 18, 2024
 *      Author: nopparuj
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include <main.h>

/* ========== BEGIN motors.h ========== */

#define CPU_FREQ 170.0 * 1.0e6
#define motor_freq 2000

/* =========== END motors.h =========== */

/* ========== BEGIN encoder.h ========== */

#define M1_duty_offset 31
#define M2_duty_offset 31
#define M3_duty_offset 32
#define M4_duty_offset 31

#define M1_duty_max 1014.0
#define M2_duty_max 1009.0
#define M3_duty_max 1048.0
#define M4_duty_max 1019.0

extern float compensation[4];

extern uint16_t duty[4];
extern uint16_t last_duty[4];

extern int32_t counter[4];
extern int32_t last_counter[4];

extern float motor_speed[4];

/* =========== END encoder.h =========== */

/* ========== BEGIN controller.h ========== */

extern float Kp;
extern float Ki;
extern float Kd;

/* =========== END controller.h =========== */

#endif /* INC_CONFIG_H_ */
