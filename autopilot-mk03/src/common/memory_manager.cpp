#include "memory_manager.h"

/**
 * Initializes a 2D array of floats, with all elements stored contiguously in memory.
 *
 * This function dynamically allocates memory for a 2D array of dimensions 'row' x 'col'.
 * The memory is contiguous, meaning that the entire 2D array is stored in a single
 * block of memory. This can improve cache coherency and performance when iterating
 * over the array, and simplifies memory management by requiring only two 'delete' operations.
 *
 * @param row  Number of rows in the 2D array.
 * @param col  Number of columns in each row of the 2D array.
 * @return     A pointer to the 2D array (array of pointers to the rows).
 */
float **array_float_init_2d(uint8_t row, uint8_t col)
{
  // Allocate memory for 'row' number of float pointers, forming the first dimension of the 2D array.
  // Each pointer will eventually point to the beginning of each row in the 2D array.
  float **array2D = new float *[row];

  // Allocate contiguous memory for the entire 2D array (i.e., 'row' times 'col' floats).
  // The '()' at the end initializes the allocated memory to zero (for each float).
  float *pool = new float[row * col]();

  // Link each pointer in 'array2D' to the corresponding "row" within the contiguous block ('pool').
  // 'pool' is advanced by 'col' elements after each row is set up, moving the pointer to the start of the next row.
  for (unsigned i = 0; i < row; ++i, pool += col)
    array2D[i] = pool; // Set the 'i-th' row's start pointer

  // Return the base pointer of the 2D array. The caller will use this pointer to access the rows and columns of the array.
  return array2D;
}
