#include "maths.h"

#include <Arduino.h>

// global variables
// ----------------------------------------------------------------------------------
float math_tmp1_f, math_tmp2_f;
double math_tmp1_d, math_tmp2_d;
int math_tmp1_i;

// temporary variabls to hold values
matrix_3x3f_t mat_tmp_3x3f_1;

float determinant, determinant_inv;  // for determinant operations

// matrix
// ----------------------------------------------------------------------------------
void math_matrix_add_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result) {
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

void math_matrix_add_f(matrix_3f_t *mat1, matrix_3f_t *mat2, matrix_3f_t *result) {
    // matrix_add_f((float *)mat1->value, (float *)mat2->value, 1, 3, (float *)result->value);
    result->value[0] = mat1->value[0] + mat2->value[0];
    result->value[1] = mat1->value[1] + mat2->value[1];
    result->value[2] = mat1->value[2] + mat2->value[2];
}

void math_matrix_substract_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result) {
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

void math_matrix_multiply_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result) {
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

void math_matrix_multiply_f(matrix_3x3f_t *mat1, matrix_3f_t *mat2, matrix_3f_t *result) {
    result->value[0] = mat1->value[0][0] * mat2->value[0] + mat1->value[0][1] * mat2->value[1] + mat1->value[0][2] * mat2->value[2];
    result->value[1] = mat1->value[1][0] * mat2->value[0] + mat1->value[1][1] * mat2->value[1] + mat1->value[1][2] * mat2->value[2];
    result->value[2] = mat1->value[2][0] * mat2->value[0] + mat1->value[2][1] * mat2->value[1] + mat1->value[2][2] * mat2->value[2];
}

void math_matrix_cofactor_f(matrix_3x3f_t *mat, matrix_3x3f_t *result) {
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

float math_matrix_determinant_f(matrix_3x3f_t *mat) {
    return mat->value[0][0] * (mat->value[1][1] * mat->value[2][2] - mat->value[2][1] * mat->value[1][2]) -
           mat->value[0][1] * (mat->value[1][0] * mat->value[2][2] - mat->value[1][2] * mat->value[2][0]) +
           mat->value[0][2] * (mat->value[1][0] * mat->value[2][1] - mat->value[1][1] * mat->value[2][0]);
}

void math_matrix_adjoint_f(matrix_3x3f_t *mat, matrix_3x3f_t *adj) {
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

bool math_matrix_inverse_f(matrix_3x3f_t *mat, matrix_3x3f_t *mat_inv) {
    determinant = math_matrix_determinant_f(mat);  // get the determinant
    if (determinant == 0) return false;

    determinant_inv = 1 / determinant;  // inverse of determinant

    math_matrix_adjoint_f(mat, mat_inv);
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

void math_matrix_copy_f(matrix_3x3f_t *src, matrix_3x3f_t *dest) {
    memccpy(dest, src, 1, 36);
}

// squareroot
// float sqrt_newton_raphson(float s) {
//     tmp1_f = s * 0.5;                      // initial guess
//     tmp1_f = (tmp1_f + s / tmp1_f) * 0.5;  // 0
//     tmp1_f = (tmp1_f + s / tmp1_f) * 0.5;  // 1
//     tmp1_f = (tmp1_f + s / tmp1_f) * 0.5;  // 2
//     tmp1_f = (tmp1_f + s / tmp1_f) * 0.5;  // 3
//     tmp1_f = (tmp1_f + s / tmp1_f) * 0.5;  // 4
//     tmp1_f = (tmp1_f + s / tmp1_f) * 0.5;  // 5
//     return tmp1_f;
// }

float math_sqrt_newton_raphson(double s) {
    math_tmp1_d = s * 0.5;                      // initial guess
    math_tmp1_d = (math_tmp1_d + s / math_tmp1_d) * 0.5;  // 1
    math_tmp1_d = (math_tmp1_d + s / math_tmp1_d) * 0.5;  // 2
    math_tmp1_d = (math_tmp1_d + s / math_tmp1_d) * 0.5;  // 3
    math_tmp1_d = (math_tmp1_d + s / math_tmp1_d) * 0.5;  // 4
    math_tmp1_d = (math_tmp1_d + s / math_tmp1_d) * 0.5;  // 5
    math_tmp1_d = (math_tmp1_d + s / math_tmp1_d) * 0.5;  // 6
    math_tmp1_d = (math_tmp1_d + s / math_tmp1_d) * 0.5;  // 7
    math_tmp1_d = (math_tmp1_d + s / math_tmp1_d) * 0.5;  // 8
    math_tmp1_d = (math_tmp1_d + s / math_tmp1_d) * 0.5;  // 9
    math_tmp1_d = (math_tmp1_d + s / math_tmp1_d) * 0.5;  // 10
    return math_tmp1_d;
}

// // to calculate the square root (basic and approximate)
// float sqrt_approx(float value) {
//     tmp1_f = 0.5f * value;
//     tmp1_i = *(int *)&value;              // Get bits for floating value
//     tmp1_i = 0x5f3759df - (tmp1_i >> 1);  // Initial guess for Newton's method
//     value = *(float *)&tmp1_i;
//     value = value * (1.5f - (tmp1_f * value * value));  // Newton's method step
//     return 1.0f / value;
// }

// to solve quadratic equation
void math_solve_quadratic(double a, double b, double c, double y, double &x1, double &x2, uint8_t &numRoots) {
    math_tmp1_d = c - y;                   // c_prime
    math_tmp2_d = b * b - 4 * a * math_tmp1_d;  // discriminant

    if (math_tmp2_d > 0) {
        math_tmp2_d = math_sqrt_newton_raphson(math_tmp2_d);
        x1 = (-b + math_tmp2_d) / (2 * a);
        x2 = (-b - math_tmp2_d) / (2 * a);
        numRoots = 2;
    } else if (math_tmp2_d == 0) {
        x1 = -b / (2 * a);
        x2 = x1;
        numRoots = 1;
    } else {
        numRoots = 0;  // No real roots
    }
}