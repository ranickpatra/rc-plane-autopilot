#pragma once

#include <stdint.h>
#include "global.h"

const float matrix_eye[4][4] = {
  {0.0, 0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0, 0.0}
};







void matrix_float_add(float **mat1, float **mat2, float **result, row_col_t size);
void matrix_float_substract(float **mat1, float **mat2, float **result, row_col_t size);
bool matrix_float_multiply(float **mat1, row_col_t mat1_size, float **mat2, row_col_t mat2_size, float **result);
bool matrix_float_inverse(float **mat, float **mat_inv, uint8_t size);

void matrix_float_copy(float **mat_dest, float **mat_src, row_col_t size);
