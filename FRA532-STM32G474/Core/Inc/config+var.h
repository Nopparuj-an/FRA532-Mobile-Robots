/*
 * configs.h
 *
 *  Created on: Apr 18, 2024
 *      Author: nopparuj
 */

#ifndef INC_CONFIG_VAR_H_
#define INC_CONFIG_VAR_H_

#include <main.h>
#include <arm_math.h>
#include "kalman.h"

/* ========== BEGIN Robot ========== */

#define WHEEL_RADIUS		0.049338
#define WHEEL_R_INV			1.0/WHEEL_RADIUS
#define WHEEL_DISTANCE_X	0.33
#define WHEEL_DISTANCE_Y	0.336
#define WHEEL_DIST_SUM		WHEEL_DISTANCE_X + WHEEL_DISTANCE_Y

/* ========== END Robot ========== */

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

/* ========== BEGIN Kalman & Controller ========== */

// Speed ramp (rad/s^2)
#define RAMP_ACCEL 50.0

extern float32_t volt[4];
extern float vel[4];
extern float target_vel_ramped[4];
extern float target_vel_unramped[4];

extern uint8_t motor_ID[];

extern float Kp;
extern float Ki;
extern float Kd;

extern KalmanFilter filterA;
extern KalmanFilter filterB;
extern KalmanFilter filterC;
extern KalmanFilter filterD;

/* =========== END Kalman & Controller =========== */

#endif /* INC_CONFIG_VAR_H_ */
