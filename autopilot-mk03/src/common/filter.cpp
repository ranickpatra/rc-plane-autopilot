#include "filter.h"
#include "maths.h"
#include "memory_manager.h"
#include "axis.h"

#ifdef XYZ_AXIS_COUNT
#if XYZ_AXIS_COUNT == 3
float ekf_P[3][3] = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0},
};
float ekf_Q[3][3] = {
    {0.01, 0.0, 0.0},
    {0.0, 0.01, 0.0},
    {0.0, 0.0, 0.0},
};
float ekf_R[3][3] = {
    {0.1, 0.0, 0.0},
    {0.0, 0.1, 0.0},
    {0.0, 0.0, 0.1},
};

float ekf_K[3][3] = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
};

float ekf_state[3] = {0.0, 0.0, 0.0};
float ekf_residual[3] = {0.0, 0.0, 0.0};

simple_low_pass_filter_1d_t ekf_low_pass_filter_acc;
row_col_t ekf_size = {
    .row = XYZ_AXIS_COUNT,
    .col = XYZ_AXIS_COUNT};

#endif
#endif

// temporary matrix
float tmp_matrix_1[XYZ_AXIS_COUNT][XYZ_AXIS_COUNT];
float tmp_matrix_2[XYZ_AXIS_COUNT][XYZ_AXIS_COUNT];

/**
 * @brief Initializes a simple low-pass filter.
 *
 * The low-pass filter is defined by its alpha parameter which determines the degree
 * to which the previous value affects the new value. An alpha of 1 will make
 * the filter hold its previous value, while an alpha of 0 will make the filter
 * fully adapt to the new value.
 *
 * @param filter Pointer to the filter structure to be initialized.
 * @param alpha Weighting factor. Must be between 0.0 and 1.0.
 */
void simple_low_pass_filter_init(simple_low_pass_filter_t *filter, float alpha)
{
    filter->data = 0;
    filter->alpha = alpha;
}

/**
 * @brief Updates the low-pass filter with a new data point.
 *
 * The function uses the alpha parameter of the filter to determine the weighting
 * of the previous value versus the new value.
 *
 * @param filter Pointer to the filter structure to be updated.
 * @param data New data point.
 */
void simple_low_pass_filter_update(simple_low_pass_filter_t *filter, float data)
{
    filter->data = filter->alpha * filter->data + (1 - filter->alpha) * data;
}

/**
 * Initializes a simple 1D low pass filter.
 *
 * @param filter  Pointer to the filter structure to initialize.
 * @param size    The number of data points the filter will handle.
 * @param alpha   The smoothing factor, typically between 0 (no smoothing) and 1.
 */
void simple_low_pass_filter_init(simple_low_pass_filter_1d_t *filter, uint8_t size, float alpha)
{
    // Allocate memory for the data based on the provided size
    filter->data = new float[size];

    // Initialize all data points to 0
    for (uint8_t i = 0; i < size; i++)
    {
        filter->data[i] = 0;
    }

    // Set the size and alpha attributes of the filter
    filter->size = size;
    filter->alpha = alpha;
}

/**
 * Updates the filter's data with new input data using a low-pass filter algorithm.
 *
 * @param filter  Pointer to the filter structure to update.
 * @param data    Pointer to an array of new data to be filtered.
 */
void simple_low_pass_filter_update(simple_low_pass_filter_1d_t *filter, float *data)
{
    // For each data point, apply the low-pass filter formula
    // which is a weighted average between the existing data and new data
    for (uint8_t i = 0; i < filter->size; i++)
    {
        filter->data[i] = filter->alpha * filter->data[i] + (1 - filter->alpha) * data[i];
    }
}

/**
 * @brief Initializes a mean accumulator.
 *
 * The mean accumulator is used to calculate the mean of a series of values.
 *
 * @param filter Pointer to the accumulator structure to be initialized.
 */
void mean_accumulator_init(mean_accumulator_t *filter)
{
    filter->data = 0;
    filter->count = 0;
}

/**
 * @brief Adds a new data point to the mean accumulator.
 *
 * This function updates the sum of data and the count of data points.
 *
 * @param filter Pointer to the accumulator structure to be updated.
 * @param data New data point.
 */
void mean_accumulator_add(mean_accumulator_t *filter, int32_t data)
{
    filter->data += data;
    filter->count++;
}

/**
 * @brief Calculates the mean value of the accumulated data points.
 *
 * After calculation, the accumulator is reinitialized.
 *
 * @param filter Pointer to the accumulator structure.
 * @param default_value Value to be returned if there are no data points accumulated.
 * @return The mean value of the accumulated data points or the default_value.
 */
uint16_t mean_accumulator_calculate(mean_accumulator_t *filter, int16_t default_value)
{
    if (filter->count)
    {
        uint16_t result = filter->data / filter->count;
        mean_accumulator_init(filter);
        return result;
    }
    return default_value;
}

void ekf_init(float alpha)
{
    simple_low_pass_filter_init(&ekf_low_pass_filter_acc, XYZ_AXIS_COUNT, alpha);
}

void ekf_update(float *gyro, float *acc)
{
    // predictions steps

    for (uint8_t i = 0; i < XYZ_AXIS_COUNT; i++)
    {
        ekf_state[i] += gyro[i] * LOOP_TIME;
    }

    // Jacobians and other necessary matrices should be computed here

    matrix_float_add((float **)ekf_P, (float **)ekf_Q, (float **)ekf_P, ekf_size);

    // update steps
    simple_low_pass_filter_update(&ekf_low_pass_filter_acc, acc);
    // Compute measurement residual
    for (uint8_t i = 0; i < ekf_size.row; i++)
    {
        ekf_residual[i] = ekf_low_pass_filter_acc.data[i] - ekf_state[i];
    }
    // Compute kalman gain
    matrix_float_add((float **)ekf_P, (float **)ekf_R, (float **)tmp_matrix_1, ekf_size);
    matrix_float_inverse((float **)tmp_matrix_1, (float **)tmp_matrix_2, ekf_size.row);
    matrix_float_multiply((float **)ekf_P, ekf_size, (float **)tmp_matrix_2, ekf_size, (float **)ekf_K);
    // Update state estimate
    matrix_float_multiply((float **)ekf_K, ekf_size, (float **)ekf_residual, ekf_size, (float **)tmp_matrix_1);
    matrix_float_add((float **)ekf_state, (float **)tmp_matrix_1, (float **)ekf_state, ekf_size);
    // Update error covariance
    matrix_float_substract((float **)matrix_eye, (float **)ekf_K, (float **)tmp_matrix_1, ekf_size);
    matrix_float_multiply((float **)tmp_matrix_1, ekf_size, (float **)ekf_P, ekf_size, (float **)tmp_matrix_2);
    matrix_float_copy((float **)ekf_P, (float **)tmp_matrix_2, ekf_size);
}
