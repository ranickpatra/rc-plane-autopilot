#include "maths.h"

#ifndef abs
#define abs(x) ((x) > 0 ? (x) : -(x))
#endif

matrix_3x3f_t mat_tmp_3x3f_1;

// /**
//  * Function to add two matrices of floats.
//  *
//  * @param mat1    First matrix for addition.
//  * @param mat2    Second matrix for addition.
//  * @param result  Matrix to store the result of the addition.
//  * @param row     Number of rows in the matrices.
//  * @param col     Number of columns in the matrices.
//  */
// void matrix_float_add(float mat1[3][3], float mat2[3][3], float result[3][3], row_col_t size) {
//   // Iterate over each row
//   for (uint8_t i = 0; i < size.row; i++) {
//     // Iterate over each column
//     for (uint8_t j = 0; j < size.col; j++) {
//       // Add corresponding elements of mat1 and mat2 and store in result matrix
//       result[i][j] = mat1[i][j] + mat2[i][j];
//     }
//   }
// }

// /**
//  * Subtracts two matrices of the same dimensions.
//  *
//  * This function subtracts each element in `mat2` from its corresponding element
//  * in `mat1` and stores the result in the `result` matrix. Both input matrices
//  * must have the same dimensions, defined by the number of rows and columns
//  * provided in the `size` parameter. The operation performed is a cell-by-cell
//  * subtraction, and it's executed for each element in the matrices.
//  *
//  * @param mat1   Pointer to the first matrix (minuend) - a 2D array of floats.
//  * @param mat2   Pointer to the second matrix (subtrahend) - a 2D array of
//  * floats.
//  * @param result Pointer to the result matrix where the difference of mat1 and
//  * mat2 will be stored.
//  * @param size     A row_col_t struct holding the number of rows and columns for
//  * the input matrices.
//  */
// void matrix_float_substract(float mat1[3][3], float mat2[3][3], float result[3][3], row_col_t size) {
//   // Iterate over each row of the matrices.
//   for (uint8_t i = 0; i < size.row; i++) {
//     // Iterate over each column within the current row.
//     for (uint8_t j = 0; j < size.col; j++) {
//       // Subtract the element in mat2 from the corresponding element in mat1
//       // and store the result in the corresponding element of the result matrix.
//       result[i][j] = mat1[i][j] - mat2[i][j];
//     }
//   }
// }

// /**
//  * Multiplies two matrices and stores the result in a third matrix.
//  *
//  * The number of columns in the first matrix must match the number of rows in
//  * the second matrix.
//  *
//  * @param mat1     Pointer to the first matrix.
//  * @param mat1_size  Struct containing the number of rows and columns for mat1.
//  * @param mat2     Pointer to the second matrix.
//  * @param mat2_size  Struct containing the number of rows and columns for mat2.
//  * @param result   Pointer to the matrix where the result will be stored.
//  * @return         True if multiplication is successful, false otherwise.
//  */
// bool matrix_float_multiply(float mat1[3][3], row_col_t mat1_size, float mat2[3][3], row_col_t mat2_size, float result[3][3]) {
//   // Validate matrix dimensions for multiplication.
//   // The number of columns in the first matrix (mat1) must be equal
//   // to the number of rows in the second matrix (mat2).
//   if (mat1_size.col != mat2_size.row) return false;

//   // Iterate through each row of the first matrix.
//   for (uint8_t i = 0; i < mat1_size.row; i++) {
//     // Iterate through each column of the second matrix.
//     for (uint8_t j = 0; j < mat2_size.col; j++) {
//       // Initialize the result cell to zero before summing up.
//       result[i][j] = 0;

//       // Calculate the dot product of the i-th row of mat1 with
//       // the j-th column of mat2, and accumulate the result.
//       for (uint8_t k = 0; k < mat1_size.col; k++) {
//         result[i][j] += mat1[i][k] * mat2[k][j];
//       }
//     }
//   }
//   return true;  // Matrix multiplication successful.
// }

// /**
//  * Computes the inverse of a square matrix using the Gauss-Jordan elimination
//  * method.
//  *
//  * @param mat      Pointer to the matrix to be inverted.
//  * @param mat_inv  Pointer to the matrix where the inverse will be stored.
//  * @param size     Size of the matrix (number of rows or columns as it's
//  * square).
//  * @return         True if the matrix is invertible, false otherwise (i.e.,
//  * singular).
//  */
// bool matrix_float_inverse(float mat[3][3], float mat_inv[3][3], uint8_t size) {
//   // Initialize the 'mat_inv' matrix as an identity matrix.
//   // This identity matrix will gradually be transformed into the inverse.
//   for (uint8_t i = 0; i < size; i++)
//     for (uint8_t j = 0; j < size; j++) mat_inv[i][j] = (i == j) ? 1 : 0;

//   // Start the Gauss-Jordan elimination process.

//   // Forward elimination process.
//   for (uint8_t i = 0; i < size; i++) {
//     // Normalize rows such that diagonal elements of 'mat' become 1.
//     float diagValue = mat[i][i];
//     for (uint8_t j = 0; j < size; j++) {
//       mat[i][j] /= diagValue;
//       mat_inv[i][j] /= diagValue;
//     }

//     // Make the non-diagonal elements of the current column i in other rows
//     // zero.
//     for (uint8_t k = 0; k < size; k++) {
//       // Skip the normalization row.
//       if (k != i) {
//         // Calculate factor from the current element.
//         float factor = mat[k][i];
//         for (uint8_t j = 0; j < size; j++) {
//           // Subtract the proportionate amount from the 'k' row.
//           mat[k][j] -= factor * mat[i][j];
//           mat_inv[k][j] -= factor * mat_inv[i][j];
//         }
//       }
//     }
//   }

//   // Check the diagonal elements of the 'mat' matrix.
//   // If any diagonal element is zero after the process, the matrix is singular.
//   for (uint8_t i = 0; i < size; i++)
//     if (mat[i][i] == 0) return false;  // Matrix is singular and non-invertible.

//   return true;  // Matrix inversion was successful.
// }

// /**
//  * Copies the content of one matrix into another.
//  *
//  * This function iterates over every element in the source matrix `mat_src` and
//  * copies each element into the corresponding location in the destination matrix
//  * `mat_dest`. Both matrices must have the same dimensions, defined by the
//  * `size` parameter, which contains the number of rows and columns. The copying
//  * is done element by element.
//  *
//  * Note: This function assumes that `mat_dest` is already allocated with the
//  * same dimensions as `mat_src`. If `mat_dest` points to unallocated memory, or
//  * memory that is not sufficient to contain the contents of `mat_src`, the
//  * behavior is undefined.
//  *
//  * @param mat_dest Pointer to the destination matrix - a 2D array of floats
//  * where the data will be copied to.
//  * @param mat_src  Pointer to the source matrix - a 2D array of floats which
//  * contains the data to be copied.
//  * @param size     A row_col_t struct specifying the number of rows and columns
//  * in the source and destination matrices.
//  */
// void matrix_float_copy(float mat_dest[3][3], float mat_src[3][3], row_col_t size) {
//   // Iterate over each row of the matrices.
//   for (uint8_t i = 0; i < size.row; i++) {
//     // Iterate over each column within the current row.
//     for (uint8_t j = 0; j < size.col; j++) {
//       // Copy the content of the current element in the source matrix
//       // to the corresponding location in the destination matrix.
//       mat_dest[i][j] = mat_src[i][j];
//     }
//   }
// }

// -------------------------------------------------------------------------------------------------------------------------------------------------------

// matrix
// ----------------------------------------------------------------------------------

/**
 * Adds two matrices of type float element-wise.
 *
 * @param mat1 Pointer to the first matrix.
 * @param mat2 Pointer to the second matrix.
 * @param row Number of rows in each matrix.
 * @param col Number of columns in each matrix.
 * @param result Pointer to the result matrix.
 */
// void matrix_add_f(float *mat1, float *mat2, uint8_t row, uint8_t col, float *result) {
//   // Loop through each row of the matrices
//   for (uint8_t i = 0; i < row; i++) {
//     // Loop through each column of the matrices
//     for (uint8_t j = 0; j < col; j++) {
//       // Perform element-wise addition and store the result
//       result[i * col + j] = mat1[i * col + j] + mat2[i * col + j];
//     }
//   }
// }

/**
 * Function to add two 3x3 matrices of floats, utilizing a previously defined
 * function for matrix addition.
 *
 * @param mat1 Pointer to the first 3x3 matrix of type matrix_3x3f.
 * @param mat2 Pointer to the second 3x3 matrix of type matrix_3x3f.
 * @param result Pointer to the result matrix where the sum is stored, of type
 * matrix_3x3f.
 */
void matrix_add_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result) {
    // Call the matrix addition function, casting the 3x3 matrix values to float
    // pointers. The size (rows and columns) of the matrices are derived from the
    // 'size' attribute of the matrices.
    // matrix_add_f((float *)mat1->value, (float *)mat2->value, 3, 3, (float *)result->value);
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

// void matrix_substract_f(float *mat1, float *mat2, uint8_t row, uint8_t col, float *result) {
//   for (uint8_t i = 0; i < row; i++) {
//     for (uint8_t j = 0; j < col; j++) {
//       result[i * col + j] = mat1[i * col + j] - mat2[i * col + j];
//     }
//   }
// }

void matrix_substract_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result) {
    // matrix_substract_f((float *)mat1->value, (float *)mat2->value, 3, 3, (float *)result->value);
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

// bool matrix_multiply_f(float *mat1, uint8_t size1_row, uint8_t size1_col, float *mat2, uint8_t size2_row, uint8_t size2_col, float *result) {
//     if (size1_col != size2_row) return false;

//     for (uint8_t i = 0; i < size1_row; i++) {
//         for (uint8_t j = 0; j < size2_col; j++) {
//             result[i * size2_col + j] = 0;

//             for (uint8_t k = 0; k < size1_col; k++) {
//                 result[i * size2_col + j] += mat1[i * size1_col + k] * mat2[k * size2_col + j];
//             }
//         }
//     }
//     return true;  // Matrix multiplication successful.
// }

void matrix_multiply_f(matrix_3x3f_t *mat1, matrix_3x3f_t *mat2, matrix_3x3f_t *result) {
    // return matrix_multiply_f((float *)mat1->value, 3, 3, (float *)mat2->value, 3, 3, (float *)result->value);
    result->value[0][0] = mat1->value[0][0]*mat2->value[0][0] + mat1->value[0][1]*mat2->value[1][0] + mat1->value[0][2]*mat2->value[2][0];
    result->value[0][1] = mat1->value[0][0]*mat2->value[0][1] + mat1->value[0][1]*mat2->value[1][1] + mat1->value[0][2]*mat2->value[2][1];
    result->value[0][2] = mat1->value[0][0]*mat2->value[0][2] + mat1->value[0][1]*mat2->value[1][2] + mat1->value[0][2]*mat2->value[2][2];

    result->value[1][0] = mat1->value[1][0]*mat2->value[0][0] + mat1->value[1][1]*mat2->value[1][0] + mat1->value[1][2]*mat2->value[2][0];
    result->value[1][1] = mat1->value[1][0]*mat2->value[0][1] + mat1->value[1][1]*mat2->value[1][1] + mat1->value[1][2]*mat2->value[2][1];
    result->value[1][2] = mat1->value[1][0]*mat2->value[0][2] + mat1->value[1][1]*mat2->value[1][2] + mat1->value[1][2]*mat2->value[2][2];

    result->value[2][0] = mat1->value[2][0]*mat2->value[0][0] + mat1->value[2][1]*mat2->value[1][0] + mat1->value[2][2]*mat2->value[2][0];
    result->value[2][1] = mat1->value[2][0]*mat2->value[0][1] + mat1->value[2][1]*mat2->value[1][1] + mat1->value[2][2]*mat2->value[2][1];
    result->value[2][2] = mat1->value[2][0]*mat2->value[0][2] + mat1->value[2][1]*mat2->value[1][2] + mat1->value[2][2]*mat2->value[2][2];

}

void matrix_multiply_f(matrix_3x3f_t *mat1, matrix_3f_t *mat2, matrix_3f_t *result) {
    // return matrix_multiply_f((float *)mat1->value, 3, 3, (float *)mat2->value, 3, 1, (float *)result->value);
    result->value[0] = mat1->value[0][0]*mat2->value[0] + mat1->value[0][1]*mat2->value[1] + mat1->value[0][2]*mat2->value[2];
    result->value[1] = mat1->value[1][0]*mat2->value[0] + mat1->value[1][1]*mat2->value[1] + mat1->value[1][2]*mat2->value[2];
    result->value[2] = mat1->value[2][0]*mat2->value[0] + mat1->value[2][1]*mat2->value[1] + mat1->value[2][2]*mat2->value[2];
}

void matrix_cofactor_f(matrix_3x3f_t *mat, matrix_3x3f_t *result) {
    float det;
    float minor[2][2] = {0.0f};

    for (uint8_t p = 0; p < 3; p++) {
        for (uint8_t q = 0; q < 3; q++) {
            // Initialize minor matrix coordinates.
            uint8_t i = 0, j = 0;

            // Loop through the matrix to construct the minor.
            for (uint8_t row = 0; row < 3; row++) {
                for (uint8_t col = 0; col < 3; col++) {
                    if (row != p && col != q) {
                        minor[i][j] = mat->value[row][col];
                        j = (j + 1) % 2;  // Move to next column, wrap around after 2.
                        if (j == 0) {
                            i++;  // Move to next row after filling a row.
                        }
                    }
                }
            }

            // Calculate the determinant for the 2x2 minor.
            det = (minor[0][0] * minor[1][1]) - (minor[0][1] * minor[1][0]);

            // Assign the cofactor to the result matrix, adjusting sign as necessary.
            result->value[p][q] = ((p + q) % 2 == 0 ? 1 : -1) * det;
        }
    }
}

float matrix_determinant_f(matrix_3x3f_t *mat) {
    return mat->value[0][0] * (mat->value[1][1] * mat->value[2][2] - mat->value[2][1] * mat->value[1][2]) -
           mat->value[0][1] * (mat->value[1][0] * mat->value[2][2] - mat->value[1][2] * mat->value[2][0]) +
           mat->value[0][2] * (mat->value[1][0] * mat->value[2][1] - mat->value[1][1] * mat->value[2][0]);
}

void matrix_adjoint_f(matrix_3x3f_t *mat, matrix_3x3f_t *adj) {
    matrix_cofactor_f(mat, &mat_tmp_3x3f_1);  // Find cofactor matrix
    // Transpose the cofactor matrix to get the adjoint
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            adj->value[j][i] = mat_tmp_3x3f_1.value[i][j];
        }
    }
}

bool matrix_inverse_f(matrix_3x3f_t *mat, matrix_3x3f_t *mat_inv) {
    // copy into temporary matrix
    matrix_copy_f(mat, &mat_tmp_3x3f_1);
    // Initializing the inverse matrix as an identity matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == j)
                mat_inv->value[i][j] = 1;
            else
                mat_inv->value[i][j] = 0;
        }
    }

    // Performing the Gauss-Jordan elimination
    for (int i = 0; i < 3; i++) {
        // Finding the maximum element of the column and its row index
        float maxEl = abs(mat_tmp_3x3f_1.value[i][i]);
        uint8_t maxRow = i;
        for (uint8_t k = i + 1; k < 3; k++) {
            if (abs(mat_tmp_3x3f_1.value[k][i]) > maxEl) {
                maxEl = abs(mat_tmp_3x3f_1.value[k][i]);
                maxRow = k;
            }
        }

        // Check for singular matrix
        if (mat_tmp_3x3f_1.value[maxRow][i] == 0) {
            // Matrix is singular!
            return false;
        }

        // Swapping the maximum row with the current row
        for (uint8_t k = i; k < 3; k++) {
            float tmp = mat_tmp_3x3f_1.value[maxRow][k];
            mat_tmp_3x3f_1.value[maxRow][k] = mat_tmp_3x3f_1.value[i][k];
            mat_tmp_3x3f_1.value[i][k] = tmp;
        }
        for (uint8_t k = 0; k < 3; k++) {
            float tmp = mat_inv->value[maxRow][k];
            mat_inv->value[maxRow][k] = mat_inv->value[i][k];
            mat_inv->value[i][k] = tmp;
        }

        // Subtracting rows to get the upper triangular matrix form
        for (uint8_t k = 0; k < 3; k++) {
            if (k != i) {
                float c = -mat_tmp_3x3f_1.value[k][i] / mat_tmp_3x3f_1.value[i][i];
                for (uint8_t j = 0; j < 3; j++) {
                    mat_tmp_3x3f_1.value[k][j] += c * mat_tmp_3x3f_1.value[i][j];
                    mat_inv->value[k][j] += c * mat_inv->value[i][j];
                }
            }
        }

        // Dividing the diagonal elements by themselves to get 1
        float c = 1 / mat_tmp_3x3f_1.value[i][i];
        for (uint8_t j = 0; j < 3; j++) {
            mat_tmp_3x3f_1.value[i][j] *= c;
            mat_inv->value[i][j] *= c;
        }
    }

    return true;
}

void matrix_copy_f(matrix_3x3f_t *src, matrix_3x3f_t *dest) {
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            dest->value[i][j] = src->value[i][j];
        }
    }
}