#include <stdio.h>
#include <stdlib.h>

//#include <math.h>

#include "matrix.h"


int DIM = 3;
double PI = 3.14;
static double _T = 1.0;

// kalman variables:

// State mu (x,y,heading)
static double mu[3][1] = {{0},
					        {0},
						    {0}};

// input u (vb, wb)
static double u[2][1] = {{0},
						 {0}};

// TODO: Define matrix A
static const double A[3][3] = {{1.0,0,0},{0,1.0,0},{0,0,1.0}};

// TODO: Define the state covariance sigma (assume perfect knowledge of inital pose)
static double sigma[3][3] = {{0,0,0},{0,0,0},{0,0,0}};


int main(int argc, char const *argv[]) {

  // TODO: Define the input matrix B
double B[3][3] = {{_T * 0.5, 0, 0},{_T * 0.5,0,0},{0,0,_T}};

  // TODO: Define the process covariance matrix R
double R[3][3] = {{0.06,0,0},{0,0.06,0},{0,0,0.07}};

///********* state vector update ******************//
// TODO: compute the state vector prediction: mu = A*mu + B*u

double Bu[3][1] = {{0},{0},{0}};
multiply(3,3,B,3,1,mu,Bu);
add(3,1,mu,Bu,mu);

///********* noise vector update ******************//
// TODO: compute the state covariance prediction sigma = A*sigma*A^T + R

add(3,3,sigma,R,sigma);

  return 0;
}
