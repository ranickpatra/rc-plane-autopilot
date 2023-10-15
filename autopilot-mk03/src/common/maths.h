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


// matrix

void matrix_add_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result);
void matrix_add_f(matrix_3f_t *mat1, matrix_3f_t *mat2, matrix_3f_t *result);
void matrix_substract_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result);
void matrix_multiply_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result);
void matrix_multiply_f(matrix_3x3f_t *mat1, matrix_3f_t *mat2, matrix_3f_t *result);

void matrix_cofactor_f(matrix_3x3f_t *mat, matrix_3x3f_t *result);
float matrix_determinant_f(matrix_3x3f_t *mat);
void matrix_adjoint_f(matrix_3x3f_t *mat, matrix_3x3f_t *adj);
bool matrix_inverse_f(matrix_3x3f_t *mat, matrix_3x3f_t *mat_inv);


void matrix_copy_f(matrix_3x3f_t *src, matrix_3x3f_t *dest);