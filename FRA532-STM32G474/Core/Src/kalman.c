/*
 * kalman.c
 *
 *  Created on: 17 May 2023
 *      Author: Paweekorn
 */

#include "kalman.h"
#include "arm_math.h"
float Kalman_Speed = 0;

// Define initial state of X_pk and P_pk
arm_matrix_instance_f32 X_k_matrix;
arm_matrix_instance_f32 P_k_matrix;
// System matrices
arm_matrix_instance_f32 A_matrix;
arm_matrix_instance_f32 A_transpose_matrix;
arm_matrix_instance_f32 eye_matrix;
arm_matrix_instance_f32 B_matrix;
arm_matrix_instance_f32 C_matrix;
arm_matrix_instance_f32 C_transpose_matrix;
arm_matrix_instance_f32 G_matrix;
arm_matrix_instance_f32 G_transpose_matrix;
arm_matrix_instance_f32 Output_matrix;
arm_matrix_instance_f32 GGT_matrix;
arm_matrix_instance_f32 GQGT_matrix;
//------------------------------------------- for equa ------------------------------------

// Compute Xk = Ax + Bu
arm_matrix_instance_f32 Bu_matrix;
arm_matrix_instance_f32 Ax_matrix;

// Compute (C * P_k * C^T + R)
arm_matrix_instance_f32 CP_matrix;
arm_matrix_instance_f32 CPCT_matrix;
arm_matrix_instance_f32 CPCTR_matrix;

// Compute Kalman Gain: K = P_k * C^T * inv(C * P_k * C^T + R)
arm_matrix_instance_f32 K_matrix;
arm_matrix_instance_f32 PCT_matrix;

// Compute inverse of (C * P_k * C^T + R)
arm_matrix_instance_f32 CPCTRinv_matrix;

// Computation of the estimated state
arm_matrix_instance_f32 Cx_matrix;
arm_matrix_instance_f32 yCx_matrix;
arm_matrix_instance_f32 KyCx_matrix;


arm_matrix_instance_f32 R_matrix;
arm_matrix_instance_f32 Z_matrix;
arm_matrix_instance_f32 Velocity_matrix;

volatile arm_status Calst;

float checkVal;

float SteadyStateKalmanFilter(KalmanFilter* filter, float32_t Vin,float32_t Velocity){
	  arm_mat_init_f32(&Velocity_matrix, 1, 1,(float32_t*) &Velocity);
	  arm_mat_trans_f32(&A_matrix, &A_transpose_matrix);
	  arm_mat_trans_f32(&C_matrix, &C_transpose_matrix);
	  arm_mat_trans_f32(&G_matrix, &G_transpose_matrix);
	  // Compute Xk = Ax + Bu
	  arm_mat_scale_f32(&B_matrix, Vin, &Bu_matrix); 		   				// Bu
	  arm_mat_mult_f32(&A_matrix, &X_k_matrix, &Ax_matrix);  		   		// Ax
	  arm_mat_add_f32(&Ax_matrix, &Bu_matrix, &X_k_matrix); 		   		// Xk = Ax + Bu

	  // Compute (A * P_pk * A^T + G * Q * G^T)
	  arm_mat_mult_f32(&A_matrix, &P_k_matrix, &P_k_matrix);  		   		// Pk = A * P_pk
	  arm_mat_mult_f32(&P_k_matrix, &A_transpose_matrix, &P_k_matrix); 		// Pk = A * P_pk * A^T
	  arm_mat_mult_f32(&G_matrix, &G_transpose_matrix, &GGT_matrix);        // G * G^T
	  arm_mat_scale_f32(&GGT_matrix, filter->Q, &GQGT_matrix); 				   	   	// G * Q
	  arm_mat_add_f32(&P_k_matrix, &GQGT_matrix, &P_k_matrix); 	       		// A * P_pk * A^T + G * Q * G^T

	  // Compute (C * P_k * C^T + R)
	  arm_mat_mult_f32(&C_matrix, &P_k_matrix, &CP_matrix);			     // C * Pk
	  arm_mat_mult_f32(&CP_matrix, &C_transpose_matrix, &CPCT_matrix);   // C * Pk * C^T
	  arm_mat_add_f32(&CPCT_matrix, &R_matrix, &CPCTR_matrix);			 // C * P_k * C^T + R

	  // Compute inverse of (C * P_k * C^T + R)
	  arm_mat_inverse_f32(&CPCTR_matrix, &CPCTRinv_matrix);					 // inverse of (C * P_k * C^T + R)

	  // Compute Kalman Gain: K = P_k * C^T * inv(C * P_k * C^T + R)
	  arm_mat_mult_f32(&P_k_matrix, &C_transpose_matrix, &PCT_matrix); 		 // P_k * C^T
	  arm_mat_mult_f32(&PCT_matrix, &CPCTRinv_matrix, &K_matrix);  			 // P_k * C^T * inv(C * P_k * C^T + R)

	  // Computation of the estimated state
	  arm_mat_mult_f32(&C_matrix, &X_k_matrix, &Cx_matrix);				 // C * X_k
	  arm_mat_sub_f32(&Velocity_matrix,  &Cx_matrix, &yCx_matrix);			  // y - ( C * X_k )
	  arm_mat_mult_f32(&K_matrix, &yCx_matrix, &KyCx_matrix);		     // K( y - ( C * X_k ) )
	  arm_mat_add_f32(&X_k_matrix, &KyCx_matrix, &X_k_matrix);		 	 // X_k + K( y - ( C * X_k ) )

	  // Computation of the estimated output
	  arm_mat_mult_f32(&C_matrix, &X_k_matrix, &Output_matrix);

	  // Computation of the state covariance error
	  arm_matrix_instance_f32 temp_matrix4;
	  float32_t temp_data4[16];
	  arm_mat_init_f32(&temp_matrix4, 4, 4,(float32_t*) &temp_data4);

	  arm_mat_mult_f32(&K_matrix, &C_matrix, &temp_matrix4);				// K * C
	  arm_mat_sub_f32(&eye_matrix, &temp_matrix4, &temp_matrix4);			// (I - (K * C))
	  arm_mat_mult_f32(&temp_matrix4, &P_k_matrix, &P_k_matrix);			// (I - (K * C)) * P_k
	  Kalman_Speed = filter->X_k[1];
	  return  Kalman_Speed;
}

void Kalman_Start(KalmanFilter* filter){
	filter->Q = 1.0f;
	filter->R[0] = 1.0f;

	float32_t a[16] = {1.0f, 0.000996946187806927f,-0.000781560031349208f, 8.37430695770173e-05f,
	                   0.0f, 0.992523116469047f,   -1.56115964889452f, 0.160829336670584f,
	                   0.0f, 0.0f   ,   1.0f  , 0.0f,
	                   0.0f,-0.0454381199874783f,  0.0370492618434313f, 0.782610780493230f};

	float32_t b[4] = {6.11011237621270e-06f,
				      0.0179660664417631f,
					  0.0f,
					  0.190433717271840f};

	float32_t c[4] = {1.0f, 0.0f, 0.0f, 0.0f};

	float32_t g[4] = {0.0f,
					  1.0f,
					  0.0f,
					  0.0f};

	float32_t iden[16] = {1.0f, 0.0f, 0.0f, 0.0f,
			  	  	 0.0f, 1.0f, 0.0f, 0.0f,
					 0.0f, 0.0f, 1.0f, 0.0f,
					 0.0f, 0.0f, 0.0f, 1.0f,};

	float32_t x_k[4] = {0.0f, 0.0f, 0.0f, 0.0f};

	filter->Es_velocity[1] = 0.0f;

	int i;
	for(i=0;i<16;i++)
	{
		filter->A[i] = a[i];
		filter->eye[i] = iden[i];
		filter->P_k[i] = 0.0f;
	}

	for(i=0;i<4;i++)
	{
		filter->X_k[i] = x_k[i];
		filter->B[i] = b[i];
		filter->C[i] = c[i];
		filter->G[i] = g[i];

	}

	arm_mat_init_f32(&X_k_matrix, 4, 1,filter->X_k);
	arm_mat_init_f32(&P_k_matrix, 4, 4,filter->P_k);

	arm_mat_init_f32(&A_matrix, 4, 4,filter->A);
	arm_mat_init_f32(&B_matrix, 4, 1,filter->B);
	arm_mat_init_f32(&C_matrix, 1, 4,filter->C);
	arm_mat_init_f32(&G_matrix, 4, 1,filter->G);

	arm_mat_init_f32(&A_transpose_matrix, 4, 4, filter->A_transpose);
	arm_mat_init_f32(&C_transpose_matrix, 4, 1, filter->C_transpose);
	arm_mat_init_f32(&G_transpose_matrix, 1, 4, filter->G_transpose);

	arm_mat_init_f32(&GGT_matrix, 4, 4, filter->GGT);
	arm_mat_init_f32(&GQGT_matrix, 4, 4, filter->GQGT);

	// Compute Xk = Ax + Bu
	arm_mat_init_f32(&Bu_matrix, 4, 1, filter->Bu_data);
	arm_mat_init_f32(&Ax_matrix, 4, 1, filter->Ax_data);

	// Compute (C * P_k * C^T + R)
	arm_mat_init_f32(&CP_matrix, 1, 4, filter->CP);
	arm_mat_init_f32(&CPCT_matrix, 1, 1, filter->CPCT);
	arm_mat_init_f32(&CPCTR_matrix, 1, 1, filter->CPCTR);

	// Compute Kalman Gain: K = P_k * C^T * inv(C * P_k * C^T + R)
	arm_mat_init_f32(&K_matrix, 4, 1, filter->K);
	arm_mat_init_f32(&PCT_matrix, 4, 1,filter->PCT);

	// Compute inverse of (C * P_k * C^T + R)
	arm_mat_init_f32(&CPCTRinv_matrix, 1, 1,filter->CPCTRinv);

	// Computation of the estimated state
	arm_mat_init_f32(&Cx_matrix, 1, 1, filter->Cx);
	arm_mat_init_f32(&yCx_matrix, 1, 1, filter->yCx);
	arm_mat_init_f32(&KyCx_matrix, 4, 1, filter->KyCx);

	arm_mat_init_f32(&Output_matrix, 1, 1, filter->Es_velocity);
	arm_mat_init_f32(&eye_matrix, 4, 4, filter->eye);

	arm_mat_init_f32(&R_matrix, 1, 1, filter->R);
	arm_mat_init_f32(&Z_matrix, 1, 1, filter->Z);
}


