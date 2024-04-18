/*
 * config.c
 *
 *  Created on: Apr 18, 2024
 *      Author: nopparuj
 */

#include <config+var.h>

/* ========== BEGIN motors.h ========== */

// none

/* =========== END motors.h =========== */

/* ========== BEGIN encoder.h ========== */

float compensation[4] = { 1000.0 / M1_duty_max, 1000.0 / M2_duty_max, 1000.0 / M3_duty_max, 1000.0 / M4_duty_max };

uint16_t duty[4] = { 0 };
uint16_t last_duty[4] = { 0 };

int32_t counter[4] = { 0 };
int32_t last_counter[4] = { 0 };

float motor_speed[4] = { 0.0 };

/* =========== END encoder.h =========== */

/* ========== BEGIN Kalman & Controller ========== */

float32_t volt[4];
float vel[4] = {0.0};
float target_vel[4];

uint8_t motor_ID[] = {1, 2, 3 ,4};

float Kp = 0.7;
float Ki = 3;
float Kd = 0;

/* =========== END Kalman & Controller =========== */
