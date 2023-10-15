#include "maths.h"

#include <Arduino.h>

#ifndef abs
#define abs(x) ((x) > 0 ? (x) : -(x))
#endif

// global variables
// ----------------------------------------------------------------------------------

// temporary variabls to hold values
matrix_3x3f_t mat_tmp_3x3f_1;

float determinant, determinant_inv; // for determinant operations

// matrix
// ----------------------------------------------------------------------------------
void matrix_add_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result) {
    result->value[0][0] = mat1->value[0][0] + mat2->value[0][0];
    result->value[0][1] = mat1->value[0][1] + mat2->value[0][1];
    result->value[0][2] = mat1->value[0][2] + mat2->value[0][2];
    result->value[1][0] = mat1->value[1][0] + mat2->value[1][0];
    result->value[1][1] = mat1->value[1][1] + mat2->value[1][1];
    result->value[1][2] = mat1->value[1][2] + mat2->value[1][2];
    result->value[2][0] = mat1->value[2][0] + mat2->value[2][0];
    result->value[2][1] = mat1->value[2][1] + mat2->value[2][1];
    result->value[2][2] = mat1->value[2][2] + mat2->value[2][2];
}

void matrix_add_f(matrix_3f_t *mat1, matrix_3f_t *mat2, matrix_3f_t *result) {
    // matrix_add_f((float *)mat1->value, (float *)mat2->value, 1, 3, (float *)result->value);
    result->value[0] = mat1->value[0] + mat2->value[0];
    result->value[1] = mat1->value[1] + mat2->value[1];
    result->value[2] = mat1->value[2] + mat2->value[2];
}

void matrix_substract_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result) {
    result->value[0][0] = mat1->value[0][0] - mat2->value[0][0];
    result->value[0][1] = mat1->value[0][1] - mat2->value[0][1];
    result->value[0][2] = mat1->value[0][2] - mat2->value[0][2];
    result->value[1][0] = mat1->value[1][0] - mat2->value[1][0];
    result->value[1][1] = mat1->value[1][1] - mat2->value[1][1];
    result->value[1][2] = mat1->value[1][2] - mat2->value[1][2];
    result->value[2][0] = mat1->value[2][0] - mat2->value[2][0];
    result->value[2][1] = mat1->value[2][1] - mat2->value[2][1];
    result->value[2][2] = mat1->value[2][2] - mat2->value[2][2];
}

void matrix_multiply_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result) {
    result->value[0][0] = mat1->value[0][0] * mat2->value[0][0] + mat1->value[0][1] * mat2->value[1][0] + mat1->value[0][2] * mat2->value[2][0];
    result->value[0][1] = mat1->value[0][0] * mat2->value[0][1] + mat1->value[0][1] * mat2->value[1][1] + mat1->value[0][2] * mat2->value[2][1];
    result->value[0][2] = mat1->value[0][0] * mat2->value[0][2] + mat1->value[0][1] * mat2->value[1][2] + mat1->value[0][2] * mat2->value[2][2];

    result->value[1][0] = mat1->value[1][0] * mat2->value[0][0] + mat1->value[1][1] * mat2->value[1][0] + mat1->value[1][2] * mat2->value[2][0];
    result->value[1][1] = mat1->value[1][0] * mat2->value[0][1] + mat1->value[1][1] * mat2->value[1][1] + mat1->value[1][2] * mat2->value[2][1];
    result->value[1][2] = mat1->value[1][0] * mat2->value[0][2] + mat1->value[1][1] * mat2->value[1][2] + mat1->value[1][2] * mat2->value[2][2];

    result->value[2][0] = mat1->value[2][0] * mat2->value[0][0] + mat1->value[2][1] * mat2->value[1][0] + mat1->value[2][2] * mat2->value[2][0];
    result->value[2][1] = mat1->value[2][0] * mat2->value[0][1] + mat1->value[2][1] * mat2->value[1][1] + mat1->value[2][2] * mat2->value[2][1];
    result->value[2][2] = mat1->value[2][0] * mat2->value[0][2] + mat1->value[2][1] * mat2->value[1][2] + mat1->value[2][2] * mat2->value[2][2];
}

void matrix_multiply_f(matrix_3x3f_t *mat1, matrix_3f_t *mat2, matrix_3f_t *result) {
    result->value[0] = mat1->value[0][0] * mat2->value[0] + mat1->value[0][1] * mat2->value[1] + mat1->value[0][2] * mat2->value[2];
    result->value[1] = mat1->value[1][0] * mat2->value[0] + mat1->value[1][1] * mat2->value[1] + mat1->value[1][2] * mat2->value[2];
    result->value[2] = mat1->value[2][0] * mat2->value[0] + mat1->value[2][1] * mat2->value[1] + mat1->value[2][2] * mat2->value[2];
}

void matrix_cofactor_f(matrix_3x3f_t *mat, matrix_3x3f_t *result) {
    result->value[0][0] = mat->value[1][1] * mat->value[2][2] - mat->value[1][2] * mat->value[2][1];
    result->value[0][1] = mat->value[1][2] * mat->value[2][0] - mat->value[1][0] * mat->value[2][2];
    result->value[0][2] = mat->value[1][0] * mat->value[2][1] - mat->value[1][1] * mat->value[2][0];
    result->value[1][0] = mat->value[0][2] * mat->value[2][1] - mat->value[0][1] * mat->value[2][2];
    result->value[1][1] = mat->value[0][0] * mat->value[2][2] - mat->value[0][2] * mat->value[2][0];
    result->value[1][2] = mat->value[0][1] * mat->value[2][0] - mat->value[0][0] * mat->value[2][1];
    result->value[2][0] = mat->value[0][1] * mat->value[1][2] - mat->value[0][2] * mat->value[1][1];
    result->value[2][1] = mat->value[0][2] * mat->value[1][0] - mat->value[0][0] * mat->value[1][2];
    result->value[2][2] = mat->value[0][0] * mat->value[1][1] - mat->value[0][1] * mat->value[1][0];
}

float matrix_determinant_f(matrix_3x3f_t *mat) {
    return mat->value[0][0] * (mat->value[1][1] * mat->value[2][2] - mat->value[2][1] * mat->value[1][2]) -
           mat->value[0][1] * (mat->value[1][0] * mat->value[2][2] - mat->value[1][2] * mat->value[2][0]) +
           mat->value[0][2] * (mat->value[1][0] * mat->value[2][1] - mat->value[1][1] * mat->value[2][0]);
}

void matrix_adjoint_f(matrix_3x3f_t *mat, matrix_3x3f_t *adj) {
    adj->value[0][0] = mat->value[1][1] * mat->value[2][2] - mat->value[1][2] * mat->value[2][1];
    adj->value[1][0] = mat->value[1][2] * mat->value[2][0] - mat->value[1][0] * mat->value[2][2];
    adj->value[2][0] = mat->value[1][0] * mat->value[2][1] - mat->value[1][1] * mat->value[2][0];
    adj->value[0][1] = mat->value[0][2] * mat->value[2][1] - mat->value[0][1] * mat->value[2][2];
    adj->value[1][1] = mat->value[0][0] * mat->value[2][2] - mat->value[0][2] * mat->value[2][0];
    adj->value[2][1] = mat->value[0][1] * mat->value[2][0] - mat->value[0][0] * mat->value[2][1];
    adj->value[0][2] = mat->value[0][1] * mat->value[1][2] - mat->value[0][2] * mat->value[1][1];
    adj->value[1][2] = mat->value[0][2] * mat->value[1][0] - mat->value[0][0] * mat->value[1][2];
    adj->value[2][2] = mat->value[0][0] * mat->value[1][1] - mat->value[0][1] * mat->value[1][0];
}

bool matrix_inverse_f(matrix_3x3f_t *mat, matrix_3x3f_t *mat_inv) {
    determinant = matrix_determinant_f(mat);  // get the determinant
    if (determinant == 0) return false;

    determinant_inv = 1 / determinant;  // inverse of determinant

    matrix_adjoint_f(mat, mat_inv);
    mat_inv->value[0][0] *= determinant_inv;
    mat_inv->value[0][1] *= determinant_inv;
    mat_inv->value[0][2] *= determinant_inv;
    mat_inv->value[1][0] *= determinant_inv;
    mat_inv->value[1][1] *= determinant_inv;
    mat_inv->value[1][2] *= determinant_inv;
    mat_inv->value[2][0] *= determinant_inv;
    mat_inv->value[2][1] *= determinant_inv;
    mat_inv->value[2][2] *= determinant_inv;

    return true;
}

void matrix_copy_f(matrix_3x3f_t *src, matrix_3x3f_t *dest) {
    memccpy(dest, src, 1, 36);
}