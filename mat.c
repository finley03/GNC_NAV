#include "mat.h"


#include <math.h>


void mat_identity(uint16_t n, float* writeback) {
	for (uint32_t i = 0; i < n * n; ++i) {
		writeback[i] = 0;
	}

	for (uint16_t i = 0; i < n; ++i) {
		writeback[i * n + i] = 1;
	}
}


// matrix multiply
// requires row major input
// gives row major output
// returns 0 for success, other values for failiures
int mat_multiply(float* matA, uint16_t Am, uint16_t An, float* matB, uint16_t Bm, uint16_t Bn, float* writeback) {
	// check matrices can be multiplied
	if (An != Bm) return -1;

	// iterate through different positions
	for (uint16_t n = 0; n < Bn; ++n) {
		for (uint16_t m = 0; m < Am; ++m) {
			float sum = 0;

			for (uint16_t i = 0; i < An; ++i) {
				// set multiply variables
				float a = matA[i + (An * m)];
				float b = matB[(i * Bn) + n];

				// skip optimisations
				if (
					// zero
					(a == 0 || b == 0)
				) {
					continue;
				}
				
				// multiply accumulate
				sum += a * b;
			}

			// set value in writebackarray
			writeback[n + (m * Bn)] = sum;
		}
	}

	return 0;
}


void mat_transpose(float* mat, uint16_t m, uint16_t n, float* writeback) {
	for (uint16_t i = 0; i < m; ++i) {
		for (uint16_t j = 0; j < n; j++) {
			writeback[i + (j * m)] = mat[j + (i * n)];
		}
	}
}


// transposes a square matrix in place
void mat_transpose_sm(float* mat, uint16_t n) {
	// rows
	for (uint16_t i = 0; i < n - 1; ++i) {
		// columns
		for (uint16_t j = i + 1; j < n; ++j) {
			float val1 = mat[i * n + j];
			float val2 = mat[j * n + i];
			mat[j * n + i] = val1;
			mat[i * n + j] = val2;
		}
	}
}


void mat_add(float* matA, float* matB, uint32_t size, float* writeback) {
	for (uint32_t i = 0; i < size; ++i) {
		writeback[i] = matA[i] + matB[i];
	}
}


void mat_subtract(float* matA, float* matB, uint32_t size, float* writeback) {
	for (uint32_t i = 0; i < size; ++i) {
		writeback[i] = matA[i] - matB[i];
	}
}


void mat_scalar_product(float* mat, float scalar, uint32_t size, float* writeback) {
	for (uint32_t i = 0; i < size; ++i) {
		writeback[i] = mat[i] * scalar;
	}
}


void mat_copy(float* mat, uint32_t size, float* writeback) {
	for (uint32_t i = 0; i < size; ++i) {
		writeback[i] = mat[i];
	}
}


float mat_det_2x2(float* mat) {
	return (mat[0] * mat[3]) - (mat[1] * mat[2]);
}


float mat_det_3x3(float* mat) {
	float plus = (mat[0] * mat[4] * mat[8]) + (mat[1] * mat[5] * mat[6]) + (mat[2] * mat[3] * mat[7]);
	float minus = (mat[2] * mat[4] * mat[6]) + (mat[0] * mat[5] * mat[7]) + (mat[1] * mat[3] * mat[8]);
	return plus - minus;
}


void mat_inverse_2x2(float* mat, float* writeback) {
	float det = mat_det_2x2(mat);
	float invdet = 1 / det;

	writeback[0] = mat[3] * invdet;
	writeback[1] = -mat[1] * invdet;
	writeback[2] = -mat[2] * invdet;
	writeback[3] = mat[0] * invdet;
}


void mat_inverse_3x3(float* mat, float* writeback) {
	float det = mat_det_3x3(mat);
	float invdet = 1 / det;
	
	// create cofactor matrix
	float cofactors[9];
	
	cofactors[0] = mat[4] * mat[8] - mat[5] * mat[7];
	cofactors[1] = mat[5] * mat[6] - mat[3] * mat[8];
	cofactors[2] = mat[3] * mat[7] - mat[4] * mat[6];
	cofactors[3] = mat[2] * mat[7] - mat[1] * mat[8];
	cofactors[4] = mat[0] * mat[8] - mat[2] * mat[6];
	cofactors[5] = mat[1] * mat[6] - mat[0] * mat[7];
	cofactors[6] = mat[1] * mat[5] - mat[2] * mat[4];
	cofactors[7] = mat[2] * mat[3] - mat[0] * mat[5];
	cofactors[8] = mat[0] * mat[4] - mat[1] * mat[3];
	
	float cofactors_t[9];
	
	// transpose cofactors
	mat_transpose(cofactors, 3, 3, cofactors_t);
	
	// multiply by inverse determinant
	mat_scalar_product(cofactors_t, invdet, 9, writeback);
}


// decomposes matrix into L and U triangular matrices
void mat_LU_decompose(float* mat, uint16_t n, float* L, float* U) {
	// set L matrix to identity
	mat_identity(n, L);

	// copy input to U matrix
	// U matrix is working matrix
	mat_copy(mat, n * n, U);

	// copy first row of input to U matrix
	for (uint16_t i = 0; i < n; ++i) {
		U[i] = mat[i];
	}

	// iterate through pivots doing gaussian elimination
	// to create L and U triangular matrices
	// iterates to n - 1 because gaussian elimination is not
	// performed on the last column
	for (uint16_t i = 0; i < n - 1; ++i) {
		// iterate over the rows for gaussian elimination
		for (uint16_t j = i + 1; j < n; ++j) {
			// calculate multiplier for row
			// i increments columns, j increments rows
			float multiplier = U[i + j * n] / U[i * n + i];

			// set multiplier in L matrix
			L[i + j * n] = multiplier;

			// loop through numbers in rows in U matrix
			for (uint16_t k = 0; k < n; ++k) {
				U[k + j * n] -= U[k + i * n] * multiplier;
			}
		}

	}
}


// finds matrix inverse through LU decomposition
// you have to pass a vector for the function to work with
void mat_LU_inverse_n(float* L, float* U, float* workingVector, uint16_t n, float* writeback) {
	// loop through dimensions
	// solve columns individually
	// and place in rows
	for (uint16_t i = 0; i < n; ++i) {
		for (uint16_t j = 0; j < n; ++j) {
			workingVector[j] = 0;
		}
		workingVector[i] = 1;

		mat_LU_solve_n(L, U, workingVector, n, writeback + i * n);
	}

	// transpose final matrix to put rows into columns
	mat_transpose_sm(writeback, n);
}


// solves system of equations Ax = b through LU decomposition
void mat_LU_solve_n(float* L, float* U, float* vector, uint16_t n, float* writeback) {
	// first let y = Ux where x is vector
	// writeback will be working space
	for (uint16_t i = 0; i < n; ++i) {
		float value = vector[i];

		for (uint16_t j = 1; j <= i; ++j) {
			value -= L[i * n + i - j] * writeback[i - j];
		}

		writeback[i] = value;
	}

	// transfer y from writeback to vector
	for (uint16_t i = 0; i < n; ++i) {
		vector[i] = writeback[i];
	}

	// now solve Ux = y with y in vector
	// write x to writeback
	// test for i < n because once 0 is decremented it goes to max value
	for (uint16_t i = n - 1; i < n; --i) {
		float value = vector[i];

		for (uint16_t j = n - 1; j > i; --j) {
			value -= U[i * n + j] * writeback[j];
		}

		value /= U[i * n + i];

		writeback[i] = value;
	}
}


void mat_crossp(float* mat1, float* mat2, float* writeback) {
	writeback[0] = mat1[1] * mat2[2] - mat1[2] * mat2[1];
	writeback[1] = mat1[2] * mat2[0] - mat1[0] * mat2[2];
	writeback[2] = mat1[0] * mat2[1] - mat1[1] * mat2[0];
}


void mat_3_normalize(float* mat, float* writeback) {
	float scalar = 1 / sqrt(mat[0] * mat[0] + mat[1] * mat[1] + mat[2] * mat[2]);
	
	writeback[0] = mat[0] * scalar;
	writeback[1] = mat[1] * scalar;
	writeback[2] = mat[2] * scalar;
}