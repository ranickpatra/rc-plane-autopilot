#pragma once

#include <stdint.h>
#include "global.h"


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


// float abs(float f);

// matrix

// void matrix_float_add(float mat1[3][3], float mat2[3][3], float result[3][3], row_col_t size);
// void matrix_float_substract(float mat1[3][3], float mat2[3][3], float result[3][3], row_col_t size);
// bool matrix_float_multiply(float mat1[3][3], row_col_t mat1_size, float mat2[3][3], row_col_t mat2_size, float result[3][3]);
// bool matrix_float_inverse(float mat[3][3], float mat_inv[3][3], uint8_t size);

// void matrix_float_copy(float mat_dest[3][3], float mat_src[3][3], row_col_t size);




// void matrix_add_f(float *mat1, float *mat2, uint8_t row, uint8_t col, float *result);
void matrix_add_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result);
void matrix_add_f(matrix_3f_t *mat1, matrix_3f_t *mat2, matrix_3f_t *result);
// void matrix_substract_f(float *mat1, float *mat2, uint8_t row, uint8_t col, float *result);
void matrix_substract_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result);
// bool matrix_multiply_f(float *mat1, uint8_t size1_row, uint8_t size1_col, float *mat2, uint8_t size2_row, uint8_t size2_col, float *result);
void matrix_multiply_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result);
void matrix_multiply_f(matrix_3x3f_t *mat1, matrix_3f_t *mat2, matrix_3f_t *result);

void matrix_cofactor_f(matrix_3x3f_t *mat, matrix_3x3f_t *result);
float matrix_determinant_f(matrix_3x3f_t *mat);
void matrix_adjoint_f(matrix_3x3f_t *mat, matrix_3x3f_t *adj);
bool matrix_inverse_f(matrix_3x3f_t *mat, matrix_3x3f_t *mat_inv);


void matrix_copy_f(matrix_3x3f_t *src, matrix_3x3f_t *dest);