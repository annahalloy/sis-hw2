
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
//static const double A[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
							// since it's the identitx matrix here, we wil neglect it

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
	double R[3][3] = {{0.06*0.06,0,0},{0,0.06*0.06,0},{0,0,0.07*0.07}};

	///********* state vector update ******************//
	// TODO: compute the state vector prediction: mu = A*mu + B*u
	double Bu[3][1] = {{0},{0},{0}};
	multiply(3,2,B,2,1,u,Bu);
	add(3,1,mu,Bu,mu);

	///********* noise vector update ******************//
	// TODO: compute the state covariance prediction sigma = A*sigma*A^T + R

	add(3,3,sigma,R,sigma);

	//printf("sigma = \n");
	//print(3,3,sigma);

	// write result back to struct
	pose->x = mu[0][0];
	pose->y = mu[1][0];
	pose->heading = mu[2][0];

	kal_check_nan();
}

void kal_update(double z, double C[1][DIM], double Q){

    ///********* Kalman gain ******************//
	// TODO: compute the Kalman gain K = S*C^T*(C*S*C^T + Q)⁻¹

		double CT[3][1]; //inverse of C
		transpose(1,3,C,CT);

		//printf("CT = \n"); //until here ok !
		//print(3,1,CT);

		double SCT[3][1]; // to store S * CT
		multiply(3,3,sigma,3,1,CT,SCT);

		double CSCT[1][1]; // to store C * S * CT
		multiply(1,3,C,3,1,SCT,CSCT);
		CSCT[0][0] += Q; // C*S*CT + Q
		//double CI[1][1];
		inverse(1,CSCT); //inverse of a scalar

		//printf("SCT = \n");
		//print(3,1,SCT);
		//printf("CSCT = \n");
		//print(1,1,CSCT);

		double K[3][1];
		multiply(3,1,SCT,1,1,CSCT,K);

		//printf("K = \n");
		//print(3,1,K);

    ///********* Update state mu ******************//
    // TODO: compute the state update mu = mu + K*(z - C*mu)

		double CMu[1][1];
		multiply(1,3,C,3,1,mu,CMu);

		z -= CMu[0][0];

		double KZ[3][1];
		double zi[1][1] = {{z}};
		multiply(3,1,K,1,1,zi,KZ);
		add(3,1,mu,KZ,mu);


    ///********* Update state covariance sigma ******************/
    // TODO: compute the state covariance update S = (I - K*C)*S

		double IKC[3][3];
		multiply(3,1,K,1,3,C,IKC);// K*C
		substract(3,3,I,IKC,IKC); // I-K*C

		//print(3,3,IKC); //là ca a l'air correct (tjr 1,1,1)

		multiply(3,3,sigma,3,3,IKC,sigma); //result S

    kal_check_nan();
}

void kal_update_x(pose_t* pose, double z){

    // TODO: Define the C matrix
    double C[1][3] = {{1,0,0}};

    // TODO: Define the covariance Q
    double Q = 0.001*0.001;

    // TODO: Call kal_update(z,C,Q);

		kal_update(z,C,Q);

    // write result back to struct
	pose->x = mu[0][0];
	pose->y = mu[1][0];
	pose->heading = mu[2][0];
}

void kal_update_y(pose_t* pose, double z){

    // TODO: Define the C matrix
    double C[1][3] = {{0,1,0}};

    // TODO: Define the covariance Q
    double Q = 0.001*0.001; //distance sensor variance

    // TODO: Call kal_update(z,C,Q);

		kal_update(z,C,Q);

    // write result back to struct
	pose->x = mu[0][0];
	pose->y = mu[1][0];
	pose->heading = mu[2][0];
}

void kal_update_heading(pose_t* pose, double z){

    // TODO: Define the C matrix
    double C[1][3] = {{0,0,1}};

    // TODO: Define the covariance Q
    double Q = 0.03*0.03; //compass variance

    // TODO: Call kal_update(z,C,Q);

		kal_update(z,C,Q);


    // write result back to struct
	pose->x = mu[0][0];
	pose->y = mu[1][0];
	pose->heading = mu[2][0];
}
