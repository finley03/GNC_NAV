// define DATA_SIZE to number of datapoints
// use function elipsefit_[DATA_SIZE]
// paramters are returned for equation:
// Ax^2 + By^2 + Cz^2 + Dxy + Exz + Fyz + Gx + Hy + Kz + L = 0
//
// Based on
// "An algorithm for fitting an ellipsoid to data"
// "D. A. Turner, December 1, 1999"
//
// nonlinear least-squares regression on this equation
// U(x^2 + y^2 - 2z^2) V(x^2 - 2y^2 + z^2) + Mxy + Nxz + Pyz + Qx + Ry + Sz + T = x^2 + y^2 + z^2
// parameters = (U, V, M, N, P, Q, R, S, T)^T

// number of data points
#ifndef DATA_SIZE
	#define DATA_SIZE
	#error DATA_SIZE undefined
#endif

#ifdef CCAT2
#undef CCAT2
#endif
#ifdef CCAT
#undef CCAT
#endif
#ifdef FN
#undef FN
#endif

#define CCAT2(x, y) x##y
#define CCAT(x, y) CCAT2(x, y)
#define FN(x) CCAT(x, DATA_SIZE)

#ifndef ABS
#define ABS(a) ((a < 0) ? -(a) : a)
#endif

#include <stdint.h>
#include <math.h>


// geometrically fits an ellipsoid to data through nonlinear regression
void FN(ellipsoidfit_)(float* data, float* writeback) {
	// create matrix variables
	// dimensions m = DATA_SIZE
	// n = degrees of freedom = 9
	float x_mat[DATA_SIZE * 9];
	// dependent vector
	float y_vec[DATA_SIZE];

	// solve equation y = Xb
	// where b = output values a to j
	// through XTy = XTXb
	float x_mat_t[DATA_SIZE * 9];
	float XTy[9];
	float XTX[81];
	
	// populate x and y matrices
	for (uint16_t i = 0; i < DATA_SIZE; ++i) {
		float x = data[i * 3];
		float y = data[i * 3 + 1];
		float z = data[i * 3 + 2];
		float x2 = x * x;
		float y2 = y * y;
		float z2 = z * z;

		// populate x matrix with values for regression
		x_mat[i * 9] = x2 + y2 - 2 * z2;
		x_mat[i * 9 + 1] = x2 - 2 * y2 + z2;
		x_mat[i * 9 + 2] = x * y;
		x_mat[i * 9 + 3] = x * z;
		x_mat[i * 9 + 4] = y * z;
		x_mat[i * 9 + 5] = x;
		x_mat[i * 9 + 6] = y;
		x_mat[i * 9 + 7] = z;
		x_mat[i * 9 + 8] = 1;

		// populate y vector
		y_vec[i] = x2 + y2 + z2;
	}

	// get matrix transpose
	mat_transpose(x_mat, DATA_SIZE, 9, x_mat_t);

	// multiply transpose of mat by y to get vector for regression
	mat_multiply(x_mat_t, 9, DATA_SIZE, y_vec, DATA_SIZE, 1, XTy);

	// multiply mat by its transpose
	mat_multiply(x_mat_t, 9, DATA_SIZE, x_mat, DATA_SIZE, 9, XTX);

	// writeback matrices for LU decomposition
	float L[81];
	float U[81];
	// run LU decomposition on XTX
	mat_LU_decompose(XTX, 9, L, U);
	// intermediate matrix containing 9 parameter output
	float in[9];
	// solve equation using L and U matrices
	mat_LU_solve_n(L, U, XTy, 9, in);
	// convert 9 parameter output to 10 parameter
	// standard algebraic form
	writeback[0] = in[0] + in[1] - 1;
	writeback[1] = in[0] - 2 * in[1] - 1;
	writeback[2] = in[1] - 2 * in[0] - 1;
	writeback[3] = in[2];
	writeback[4] = in[3];
	writeback[5] = in[4];
	writeback[6] = in[5];
	writeback[7] = in[6];
	writeback[8] = in[7];
	writeback[9] = in[8];
}


// populates distortion vector (A) and offset vector(b)
void FN(ellipsoidcorrection_)(float* data, float* A, float* b) {
	// vector storing ellipsoid parameters
	float parameters[10];
	// fit ellipsoid to data
	FN(ellipsoidfit_)(data, parameters);

	// create matrix for ellipsoid
	float mat[9] = {
		parameters[0], parameters[3] * 0.5, parameters[4] * 0.5,
		parameters[3] * 0.5, parameters[1], parameters[5] * 0.5,
		parameters[4] * 0.5, parameters[5] * 0.5, parameters[2]
	};

	// solve for center of ellipsoid
	float L[9];
	float U[9];
	float xyz_vec[3] = {-parameters[6] * 0.5, -parameters[7] * 0.5, -parameters[8] * 0.5};
	// take LU decomposition of matrix
	mat_LU_decompose(mat, 3, L, U);
	// use LU triangular matrices to solve equation
	mat_LU_solve_n(L, U, xyz_vec, 3, b);

	// calculate eigenvalues of matrix
	float evals[3];
	mat_3_eigenvalues(mat, evals);

	// create matrix of eigenvectors
	// populate vectors on rows of matrix
	float evec_mat[9];
	mat_3_eigenvector(mat, evals[0], evec_mat);
	mat_3_eigenvector(mat, evals[1], evec_mat + 3);
	mat_3_eigenvector(mat, evals[2], evec_mat + 6);

	// take transpose of eigenvector mat to get eigenvectors
	// in columns
	mat_transpose_sm(evec_mat, 3);

	// calculate radii along eigenvectors from eigenvalues
	float radii[3];
	for (uint8_t i = 0; i < 3; ++i) {
		radii[i] = sqrt(1 / ABS((evals[i])));
	}

	// take inverse of evec_mat
	float evec_mat_inv[9];
	mat_inverse_3x3(evec_mat, evec_mat_inv);

	// construct scale matrix
	float scale_mat[9] = {
		1 / radii[0], 0, 0,
		0, 1 / radii[1], 0,
		0, 0, 1 / radii[2]
	};

	// scale evec_mat_inv
	float cardinal_mat[9];
	mat_multiply(scale_mat, 3, 3, evec_mat_inv, 3, 3, cardinal_mat);

	// rotate back to evec space
	mat_multiply(evec_mat, 3, 3, cardinal_mat, 3, 3, A);
}


#undef DATA_SIZE
#undef CCAT2
#undef CCAT
#undef FN
//#undef ABS
