#pragma once

#include <stdint.h>
#include "global.h"

// #define RAD_2_DEGREE 57.295779513


struct row_col_t
{
  uint8_t row;
  uint8_t col;
};

struct matrix_3x3f_t
{
  // row_col_t size;
  float value[3][3];
};

struct matrix_3f_t
{
  // uint8_t size = 3;
  float value[3];
};


// matrix
// --------------------------------------------

void math_matrix_add_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result);
void math_matrix_add_f(matrix_3f_t *mat1, matrix_3f_t *mat2, matrix_3f_t *result);
void math_matrix_substract_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result);
void math_matrix_multiply_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result);
void math_matrix_multiply_f(matrix_3x3f_t *mat1, matrix_3f_t *mat2, matrix_3f_t *result);

void math_matrix_cofactor_f(matrix_3x3f_t *mat, matrix_3x3f_t *result);
float math_matrix_determinant_f(matrix_3x3f_t *mat);
void math_matrix_adjoint_f(matrix_3x3f_t *mat, matrix_3x3f_t *adj);
bool math_matrix_inverse_f(matrix_3x3f_t *mat, matrix_3x3f_t *mat_inv);


void math_matrix_copy_f(matrix_3x3f_t *src, matrix_3x3f_t *dest);


// general
// --------------------------------------------
// float sqrt_newton_raphson(float s);
float math_sqrt_newton_raphson(double s);
// float sqrt_approx(float value);
void math_solve_quadratic(double a, double b, double c, double y, double &x1, double &x2, uint8_t &numRoots);
