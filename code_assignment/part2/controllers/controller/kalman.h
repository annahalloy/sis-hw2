#ifndef KALMAN_H
#define KALMAN_H 

#include "matrix.h"
#include "odometry.h"

#define DIM 3 // state dimension

/* Initializes/resets the kalman filter. */
void kal_reset(int timestep);

/**
 * Check if the state has a NaN value. 
 * Exit if NaN detected. 
 */
void kal_check_nan();

void kal_compute_input_u(double Aleft_enc, double Aright_enc);

/* Updates the estimation of the current pose based on wheel encoders
 * and distance sensor measures combined with prior knowledge of the map.
 * Updates the given struct with the current pose estimation
 */
void kal_predict(pose_t* pose);

void kal_update(double z, double C[1][DIM], double Q);
void kal_update_x(pose_t* pose, double z);
void kal_update_y(pose_t* pose, double z);
void kal_update_heading(pose_t* pose, double z);

#endif //KALMAN_H