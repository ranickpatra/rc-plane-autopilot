#include "maths.h"

/**
 * Function to add two matrices of floats.
 *
 * @param mat1    First matrix for addition.
 * @param mat2    Second matrix for addition.
 * @param result  Matrix to store the result of the addition.
 * @param row     Number of rows in the matrices.
 * @param col     Number of columns in the matrices.
 */
void matrix_float_add(float **mat1, float **mat2, float **result, row_col_t size)
{

  // Iterate over each row
  for (uint8_t i = 0; i < size.row; i++)
  {
    // Iterate over each column
    for (uint8_t j = 0; j < size.col; j++)
    {
      // Add corresponding elements of mat1 and mat2 and store in result matrix
      result[i][j] = mat1[i][j] + mat2[i][j];
    }
  }
}

/**
 * Subtracts two matrices of the same dimensions.
 *
 * This function subtracts each element in `mat2` from its corresponding element in `mat1`
 * and stores the result in the `result` matrix. Both input matrices must have the same dimensions,
 * defined by the number of rows and columns provided in the `size` parameter. The operation performed
 * is a cell-by-cell subtraction, and it's executed for each element in the matrices.
 *
 * @param mat1   Pointer to the first matrix (minuend) - a 2D array of floats.
 * @param mat2   Pointer to the second matrix (subtrahend) - a 2D array of floats.
 * @param result Pointer to the result matrix where the difference of mat1 and mat2 will be stored.
 * @param size     A row_col_t struct holding the number of rows and columns for the input matrices.
 */
void matrix_float_substract(float **mat1, float **mat2, float **result, row_col_t size)
{
  // Iterate over each row of the matrices.
  for (uint8_t i = 0; i < size.row; i++)
  {
    // Iterate over each column within the current row.
    for (uint8_t j = 0; j < size.col; j++)
    {
      // Subtract the element in mat2 from the corresponding element in mat1
      // and store the result in the corresponding element of the result matrix.
      result[i][j] = mat1[i][j] - mat2[i][j];
    }
  }
}

/**
 * Multiplies two matrices and stores the result in a third matrix.
 *
 * The number of columns in the first matrix must match the number of rows in the second matrix.
 *
 * @param mat1     Pointer to the first matrix.
 * @param mat1_size  Struct containing the number of rows and columns for mat1.
 * @param mat2     Pointer to the second matrix.
 * @param mat2_size  Struct containing the number of rows and columns for mat2.
 * @param result   Pointer to the matrix where the result will be stored.
 * @return         True if multiplication is successful, false otherwise.
 */
bool matrix_float_multiply(float **mat1, row_col_t mat1_size, float **mat2, row_col_t mat2_size, float **result)
{
  // Validate matrix dimensions for multiplication.
  // The number of columns in the first matrix (mat1) must be equal
  // to the number of rows in the second matrix (mat2).
  if (mat1_size.col != mat2_size.row)
    return false;

  // Iterate through each row of the first matrix.
  for (uint8_t i = 0; i < mat1_size.row; i++)
  {
    // Iterate through each column of the second matrix.
    for (uint8_t j = 0; j < mat2_size.col; j++)
    {
      // Initialize the result cell to zero before summing up.
      result[i][j] = 0;

      // Calculate the dot product of the i-th row of mat1 with
      // the j-th column of mat2, and accumulate the result.
      for (uint8_t k = 0; k < mat1_size.col; k++)
      {
        result[i][j] += mat1[i][k] * mat2[k][j];
      }
    }
  }
  return true; // Matrix multiplication successful.
}

/**
 * Computes the inverse of a square matrix using the Gauss-Jordan elimination method.
 *
 * @param mat      Pointer to the matrix to be inverted.
 * @param mat_inv  Pointer to the matrix where the inverse will be stored.
 * @param size     Size of the matrix (number of rows or columns as it's square).
 * @return         True if the matrix is invertible, false otherwise (i.e., singular).
 */
bool matrix_float_inverse(float **mat, float **mat_inv, uint8_t size)
{
  // Initialize the 'mat_inv' matrix as an identity matrix.
  // This identity matrix will gradually be transformed into the inverse.
  for (uint8_t i = 0; i < size; i++)
    for (uint8_t j = 0; j < size; j++)
      mat_inv[i][j] = (i == j) ? 1 : 0;

  // Start the Gauss-Jordan elimination process.

  // Forward elimination process.
  for (uint8_t i = 0; i < size; i++)
  {
    // Normalize rows such that diagonal elements of 'mat' become 1.
    float diagValue = mat[i][i];
    for (uint8_t j = 0; j < size; j++)
    {
      mat[i][j] /= diagValue;
      mat_inv[i][j] /= diagValue;
    }

    // Make the non-diagonal elements of the current column i in other rows zero.
    for (uint8_t k = 0; k < size; k++)
    {
      // Skip the normalization row.
      if (k != i)
      {
        // Calculate factor from the current element.
        float factor = mat[k][i];
        for (uint8_t j = 0; j < size; j++)
        {
          // Subtract the proportionate amount from the 'k' row.
          mat[k][j] -= factor * mat[i][j];
          mat_inv[k][j] -= factor * mat_inv[i][j];
        }
      }
    }
  }

  // Check the diagonal elements of the 'mat' matrix.
  // If any diagonal element is zero after the process, the matrix is singular.
  for (uint8_t i = 0; i < size; i++)
    if (mat[i][i] == 0)
      return false; // Matrix is singular and non-invertible.

  return true; // Matrix inversion was successful.
}

/**
 * Copies the content of one matrix into another.
 *
 * This function iterates over every element in the source matrix `mat_src` and copies
 * each element into the corresponding location in the destination matrix `mat_dest`.
 * Both matrices must have the same dimensions, defined by the `size` parameter, 
 * which contains the number of rows and columns. The copying is done element by element.
 *
 * Note: This function assumes that `mat_dest` is already allocated with the same 
 * dimensions as `mat_src`. If `mat_dest` points to unallocated memory, or memory 
 * that is not sufficient to contain the contents of `mat_src`, the behavior is undefined.
 *
 * @param mat_dest Pointer to the destination matrix - a 2D array of floats where the data will be copied to.
 * @param mat_src  Pointer to the source matrix - a 2D array of floats which contains the data to be copied.
 * @param size     A row_col_t struct specifying the number of rows and columns in the source and destination matrices.
 */
void matrix_float_copy(float **mat_dest, float **mat_src, row_col_t size)
{
  // Iterate over each row of the matrices.
  for (uint8_t i = 0; i < size.row; i++)
  {
    // Iterate over each column within the current row.
    for (uint8_t j = 0; j < size.col; j++)
    {
      // Copy the content of the current element in the source matrix
      // to the corresponding location in the destination matrix.
      mat_dest[i][j] = mat_src[i][j];
    }
  }
}
