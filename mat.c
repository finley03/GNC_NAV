#include "mat.h"

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


void mat_add(float* matA, float* matB, uint32_t size, float* writeback) {
	for (uint32_t i = 0; i < size; ++i) {
		writeback[i] = matA[i] + matB[i];
	}
}


float mat_det_2x2(float* mat) {
	return (mat[0] * mat[3]) - (mat[1] * mat[2]);
}


void mat_inverse_2x2(float* mat, float* writeback) {
	float det = mat_det_2x2(mat);
	float invdet = 1 / det;

	writeback[0] = mat[3] * invdet;
	writeback[1] = -mat[1] * invdet;
	writeback[2] = -mat[2] * invdet;
	writeback[3] = mat[0] * invdet;
}
