/*
 * kalman.h
 *
 *  Created on: May 16, 2023
 *      Author: pawee
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "main.h"
#include "arm_math.h"

typedef struct KalmanParams{
	float32_t X_k[4];
	float32_t P_k[16];
	float32_t A[16];
	float32_t B[4];
	float32_t C[4];
	float32_t G[4];
	float32_t Q;
	float32_t R[1];
	float32_t A_transpose[16];
	float32_t C_transpose[4];
	float32_t G_transpose[4];
	float32_t GGT[16];
	float32_t GQGT[16];
	float32_t Ax_data[16];
	float32_t Bu_data[4];
	float32_t Ax_datap[4];
	float32_t CP[4];
	float32_t CPCT[1];
	float32_t CPCTR[1];
	float32_t K[4];
	float32_t PCT[4];
	float32_t CPCTRinv[1];
	float32_t Cx[1];
	float32_t yCx[1];
	float32_t KyCx[4];
	float32_t Es_velocity[1];
	float32_t eye[16];
	float32_t Z[1];
} KalmanFilter;

float SteadyStateKalmanFilter(KalmanFilter* filter,float32_t Vin, float32_t Velocity);
void Kalman_Start(KalmanFilter* filter);

#endif /* INC_KALMAN_H_ */
