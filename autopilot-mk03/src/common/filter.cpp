#include "filter.h"

#include "axis.h"
#include "io/indiactor.h"
#include "memory_manager.h"

#ifdef XYZ_AXIS_COUNT
#if XYZ_AXIS_COUNT == 3

matrix_3x3f_t ekf_P = {.value = {
                               {1.0, 0.0, 0.0},
                               {0.0, 1.0, 0.0},
                               {0.0, 0.0, 1.0},
                       }};

matrix_3x3f_t ekf_Q = {.value = {
                               {0.01, 0.0, 0.0},
                               {0.0, 0.01, 0.0},
                               {0.0, 0.0, 0.0},
                       }};

matrix_3x3f_t ekf_R = {.value = {
                               {0.1, 0.0, 0.0},
                               {0.0, 0.1, 0.0},
                               {0.0, 0.0, 0.1},
                       }};

matrix_3x3f_t ekf_K = {.value = {
                               {0.0, 0.0, 0.0},
                               {0.0, 0.0, 0.0},
                               {0.0, 0.0, 0.0},
                       }};

matrix_3f_t ekf_state = {.value = {0.0, 0.0, 0.0}};
matrix_3f_t ekf_residual = {.value = {0.0, 0.0, 0.0}};

simple_low_pass_filter_1d_t ekf_low_pass_filter_acc;
// row_col_t ekf_size = {.row = XYZ_AXIS_COUNT, .col = XYZ_AXIS_COUNT};
// row_col_t ekf_1d_size = {.row = XYZ_AXIS_COUNT, .col = 1};

matrix_3x3f_t matrix_eye = {.value = {
                                    {1.0, 0.0, 0.0},
                                    {0.0, 1.0, 0.0},
                                    {0.0, 0.0, 1.0},
                            }};

#endif
#endif

// temporary matrix
matrix_3x3f_t tmp_matrix_3x3f_1;
matrix_3x3f_t tmp_matrix_3x3f_2;
matrix_3f_t tmp_matrix_3f_1;

void simple_low_pass_filter_init(simple_low_pass_filter_t *filter, float alpha) {
    filter->data = 0;
    filter->alpha = alpha;
}

void simple_low_pass_filter_update(simple_low_pass_filter_t *filter, float data) {
    filter->data = filter->alpha * filter->data + (1 - filter->alpha) * data;
}

void simple_low_pass_filter_init(simple_low_pass_filter_1d_t *filter, uint8_t size, float alpha) {
    // Allocate memory for the data based on the provided size
    filter->data = new float[size];

    // Initialize all data points to 0
    for (uint8_t i = 0; i < size; i++) {
        filter->data[i] = 0;
    }

    // Set the size and alpha attributes of the filter
    filter->size = size;
    filter->alpha = alpha;
}

void simple_low_pass_filter_update(simple_low_pass_filter_1d_t *filter, float *data) {
    // For each data point, apply the low-pass filter formula
    // which is a weighted average between the existing data and new data
    for (uint8_t i = 0; i < filter->size; i++) {
        filter->data[i] = filter->alpha * filter->data[i] + (1 - filter->alpha) * data[i];
    }
}

void mean_accumulator_init(mean_accumulator_t *filter) {
    filter->data = 0;
    filter->count = 0;
}
void mean_accumulator_add(mean_accumulator_t *filter, int32_t data) {
    filter->data += data;
    filter->count++;
}

uint16_t mean_accumulator_calculate(mean_accumulator_t *filter, int16_t default_value) {
    if (filter->count) {
        uint16_t result = filter->data / filter->count;
        mean_accumulator_init(filter);
        return result;
    }
    return default_value;
}

void ekf_init(float alpha) {
    simple_low_pass_filter_init(&ekf_low_pass_filter_acc, XYZ_AXIS_COUNT, alpha);
}

void ekf_update(float *gyro, float *acc) {
    // predictions steps
    ekf_state.value[0] += gyro[0] * LOOP_TIME;
    ekf_state.value[1] += gyro[1] * LOOP_TIME;
    ekf_state.value[2] += gyro[2] * LOOP_TIME;

    // Jacobians and other necessary matrices should be computed here

    matrix_add_f(&ekf_P, &ekf_Q, &ekf_P);

    // update steps
    simple_low_pass_filter_update(&ekf_low_pass_filter_acc, acc);
    // Compute measurement residual
    ekf_residual.value[0] = ekf_low_pass_filter_acc.data[0] - ekf_state.value[0];
    ekf_residual.value[1] = ekf_low_pass_filter_acc.data[1] - ekf_state.value[1];
    ekf_residual.value[2] = ekf_low_pass_filter_acc.data[2] - ekf_state.value[2];

    // Compute kalman gain
    matrix_add_f(&ekf_P, &ekf_R, &tmp_matrix_3x3f_1);
    matrix_inverse_f(&tmp_matrix_3x3f_1, &tmp_matrix_3x3f_2);
    matrix_multiply_f(&ekf_P, &tmp_matrix_3x3f_2, &ekf_K);

    // Update state estimate
    matrix_multiply_f(&ekf_K, &ekf_residual, &tmp_matrix_3f_1);
    matrix_add_f(&ekf_state, &tmp_matrix_3f_1, &ekf_state);

    // Update error covariance
    matrix_substract_f(&matrix_eye, &ekf_K, &tmp_matrix_3x3f_1);
    matrix_multiply_f(&tmp_matrix_3x3f_1, &ekf_P, &tmp_matrix_3x3f_2);
    matrix_copy_f(&tmp_matrix_3x3f_2, &ekf_P);
}

matrix_3f_t *ekf_get_state() {
    return &ekf_state;
}
