
#include "kalman.h"
#include <math.h>
#include <memory.h>

static double _T = 1.0;

// kalman variables:

// State mu (x,y,heading)
static double mu[DIM][1] = {{0},
					        {0},
						    {0}};

// input u (vb, wb)
static double u[2][1] = {{0},
						 {0}};

// TODO: Define matrix A 
static const double A[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

// TODO: Define the state covariance sigma (assume perfect knowledge of inital pose)
static double sigma[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

// Identity matrix 			 
static const double I[DIM][DIM] = {{1, 0, 0},
                                   {0, 1, 0},
                                   {0, 0, 1}};
						 

void kal_reset(int timestep){
	
	_T = (double)timestep/1000.;
	
	for (int i=0;i<DIM;i++){
		mu[i][0] = 0;
        if(sigma == NULL) continue;
        for(int j=0; j<DIM; j++){
            sigma[i][j] = 0;
        }
	}
}

void kal_check_nan(){
    for (int i=0;i<DIM;i++){
		if(isnan(mu[i][0])){
            printf("FATAL: kalman state is NaN, exiting...\n");
            exit(EXIT_FAILURE);
        }
	}
}

/**
 * @brief   Compute the body speeds, hence the input of the system. 
 *          Store the values in the variable u.  
 */
void kal_compute_input_u(double Aleft_enc, double Aright_enc)
{
	// TODO: update input u based on wheel encoders increments 
	
    u[0][0] = WHEEL_RADIUS * (Aright_enc + Aleft_enc) / (2.0 * _T);    // body speed 
    
    u[1][0] = WHEEL_RADIUS * (Aright_enc - Aleft_enc) / (WHEEL_AXIS * _T);    // body rate 
}

void kal_predict(pose_t* pose){
    
	// TODO: Define the input matrix B
	double B[3][2] = {{_T*cos(pose->heading), 0},{_T*sin(pose->heading),0},{0,_T}};

	// TODO: Define the process covariance matrix R
	double R[3][3] = {{0.06,0,0},{0,0.06,0},{0,0,0.07}};

	///********* state vector update ******************//
	// TODO: compute the state vector prediction: mu = A*mu + B*u
	double Bu[3][1] = {{0},{0},{0}};
	multiply(3,2,B,2,1,u,Bu);
	add(3,1,mu,Bu,mu);

	///********* noise vector update ******************//
	// TODO: compute the state covariance prediction sigma = A*sigma*A^T + R

	add(3,3,sigma,R,sigma);

	// write result back to struct
	//printf("u = \n");
	//print(3,1,u);
	pose->x = mu[0][0];
	pose->y = mu[1][0];
	pose->heading = mu[2][0];

	kal_check_nan();
}

void kal_update(double z, double C[1][DIM], double Q){

    ///********* Kalman gain ******************//
	// TODO: compute the Kalman gain K = S*C^T*(C*S*C^T + Q)⁻¹
    

    ///********* Update state mu ******************//
    // TODO: compute the state update mu = mu + K*(z - C*mu)
    

    ///********* Update state covariance sigma ******************/
    // TODO: compute the state covariance update S = (I - K*C)*S
    

    kal_check_nan();
}

void kal_update_x(pose_t* pose, double z){
    
    // TODO: Define the C matrix 
    double** C;

    // TODO: Define the covariance Q
    double Q = 0.0; 

    // TODO: Call kal_update(z,C,Q);
    

    // write result back to struct
	pose->x = mu[0][0];
	pose->y = mu[1][0];
	pose->heading = mu[2][0];
}

void kal_update_y(pose_t* pose, double z){
    
    // TODO: Define the C matrix 
    double** C;

    // TODO: Define the covariance Q
    double Q = 0.0; 

    // TODO: Call kal_update(z,C,Q);
    

    // write result back to struct
	pose->x = mu[0][0];
	pose->y = mu[1][0];
	pose->heading = mu[2][0];
}

void kal_update_heading(pose_t* pose, double z){
    
    // TODO: Define the C matrix 
    double** C;

    // TODO: Define the covariance Q
    double Q = 0.0; 

    // TODO: Call kal_update(z,C,Q);


    // write result back to struct
	pose->x = mu[0][0];
	pose->y = mu[1][0];
	pose->heading = mu[2][0];
}
