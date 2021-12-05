#include "matrix.h"

///////////////// MATRIX UTILITIES ///////////////////

void print(int N, int M, double mat[N][M]){
	for (int i=0;i<N;i++){
		for (int j=0;j<M;j++){
			printf("%f ",mat[i][j]);
		}
		printf("\n");
	}
}

void copy(int m1, int m2, const double mat[m1][m2], double res[m1][m2]){
    for (int i = 0; i < m1; ++i){
        for (int j = 0; j < m2; ++j) {
            res[i][j] = mat[i][j];
        }
    }
}

void transpose(int m1, int m2, const double mat[m1][m2], double res[m2][m1]){
    for (int i = 0; i < m1; ++i){
        for (int j = 0; j < m2; ++j) {
            res[j][i] = mat[i][j];
        }
    }
}


// Multiplies two matrices mat1[][] and mat2[][]
// and returns result.
// (m1) x (m2) and (n1) x (n2) are dimensions
// of given matrices.
void multiply(int m1, int m2, const double mat1[][m2], int n1,
              int n2, const double mat2[][n2], double res[m1][n2])
{
    int x, i, j;
    for (i = 0; i < m1; i++) 
    {
        for (j = 0; j < n2; j++) 
        {
            res[i][j] = 0;
            for (x = 0; x < m2; x++) 
            {
                *(*(res + i) + j) += *(*(mat1 + i) + x)
                                     * *(*(mat2 + x) + j);
            }
        }
    }
}


void inverse(int matsize,double I[matsize][matsize]){
	
	double A[matsize][matsize];
	int i,j,k;
	for(i=0;i<matsize;i++){
		for(j=0;j<matsize;j++){
			A[i][j] = I[i][j];
			I[i][j] = (i==j ? 1 : 0);
		}
	}
	
    for(k=0;k<matsize;k++)                                  
    {      
        double temp=A[k][k];                   //'temp'  
        // stores the A[k][k] value so that A[k][k]  will not change
        for(j=0;j<matsize;j++)      //during the operation //A[i] //[j]/=A[k][k]  when i=j=k
        {
            A[k][j]/=temp;                                  //it performs // the following row operations to make A to unit matrix
            I[k][j]/=temp;                                  //R0=R0/A[0][0],similarly for I also R0=R0/A[0][0]
        }                                                   //R1=R1-R0*A[1][0] similarly for I
        for(i=0;i<matsize;i++)                              //R2=R2-R0*A[2][0]      ,,
        {
            temp=A[i][k];                       //R1=R1/A[1][1]
            for(j=0;j<matsize;j++)             //R0=R0-R1*A[0][1]
            {                                   //R2=R2-R1*A[2][1]
                if(i==k)
                    break;                      //R2=R2/A[2][2]
                A[i][j]-=A[k][j]*temp;          //R0=R0-R2*A[0][2]
                I[i][j]-=I[k][j]*temp;          //R1=R1-R2*A[1][2]
            }
        }
    }
}

// Adds two matrices mat1[][] and mat2[][] and returns result.
// (m1) x (m2) are dimensions of given matrices.
void add(int m1, int m2, const double mat1[m1][m2], const double mat2[m1][m2], double res[m1][m2])
{
    int i, j;
    for (i = 0; i < m1; i++) 
    {
        for (j = 0; j < m2; j++) 
        {
            res[i][j] = mat1[i][j] + mat2[i][j];
        }
    }
}

// Substracts matrice mat2[][] from mat1[][] and returns result.
// (m1) x (m2) are dimensions of given matrices.
void substract(int m1, int m2, const double mat1[m1][m2], const double mat2[m1][m2], double res[m1][m2])
{
    int i, j;
    for (i = 0; i < m1; i++) 
    {
        for (j = 0; j < m2; j++) 
        {
            res[i][j] = mat1[i][j] - mat2[i][j];
        }
    }
}