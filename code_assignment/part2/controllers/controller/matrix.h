#ifndef MATRIX_H
#define MATRIX_H 

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
////// MATRIX UTILITIES /////

/**
 * Copy content of mat into res. 
 */
void copy(int m1, int m2, const double mat[m1][m2], double res[m1][m2]);

/**
 * Transpose the matrix mat into res.  
 */
void transpose(int m1, int m2, const double mat[m1][m2], double res[m2][m1]);

/* Multiplies two matrices mat1[][] and mat2[][]
 * and returns result.
 * (m1) x (m2) and (n1) x (n2) are dimensions
 * of given matrices.
 * 
 * WARNING: do not compute in place!! 
 * For instance, do not multiply m1 and m2 and store the result into m1 directly. 
 */
void multiply(int m1, int m2, const double mat1[][m2], int n1,
              int n2, const double mat2[][n2], double res[m1][n2]);

/* Inverses the given matrix in place
*/
void inverse(int matsize,double mat[matsize][matsize]);

/* Adds two matrices mat1[][] and mat2[][] and returns result.
 * (m1) x (m2) are dimensions of given matrices.
 */
void add(int m1, int m2, const double mat1[m1][m2], const double mat2[m1][m2], double res[m1][m2]);

/* Substracts matrice mat2[][] from mat1[][] and returns result.
 * (m1) x (m2) are dimensions of given matrices.
 */
void substract(int m1, int m2, const double mat1[m1][m2], const double mat2[m1][m2], double res[m1][m2]);

/* Prints the matrix of dimensions NxM to stdout
 */ 
void print(int N, int M, double mat[N][M]);

#endif //UTIL_H