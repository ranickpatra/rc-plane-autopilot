#include "filter.h"

#include "axis.h"
#include "io/indiactor.h"

#ifdef XYZ_AXIS_COUNT
#if XYZ_AXIS_COUNT == 3

matrix_3x3f_t filter_ekf_P = {.value = {
                               {1.0, 0.0, 0.0},
                               {0.0, 1.0, 0.0},
                               {0.0, 0.0, 1.0},
                       }};

matrix_3x3f_t filter_ekf_Q = {.value = {
                               {0.002, 0.0, 0.0},
                               {0.0, 0.002, 0.0},
                               {0.0, 0.0, 0.0},
                       }};

matrix_3x3f_t filter_ekf_R = {.value = {
                               {100, 0.0, 0.0},
                               {0.0, 100, 0.0},
                               {0.0, 0.0, 100},
                       }};

matrix_3x3f_t filter_ekf_K = {.value = {
                               {0.0, 0.0, 0.0},
                               {0.0, 0.0, 0.0},
                               {0.0, 0.0, 0.0},
                       }};

matrix_3f_t filter_ekf_state = {.value = {0.0, 0.0, 0.0}};
matrix_3f_t filter_ekf_residual = {.value = {0.0, 0.0, 0.0}};

simple_low_pass_filter_3f_t filter_ekf_low_pass_filter_acc;

matrix_3x3f_t filter_matrix_eye = {.value = {
                                    {1.0, 0.0, 0.0},
                                    {0.0, 1.0, 0.0},
                                    {0.0, 0.0, 1.0},
                            }};

#endif
#endif

// temporary matrix
matrix_3x3f_t filter_tmp_matrix_3x3f_1;
matrix_3x3f_t filter_tmp_matrix_3x3f_2;
matrix_3f_t filter_tmp_matrix_3f_1;

void filter_simple_low_pass_filter_init(simple_low_pass_filter_t *filter, float alpha) {
    filter->data = 0;
    filter->alpha = alpha;
}

void filter_simple_low_pass_filter_update(simple_low_pass_filter_t *filter, float data) {
    filter->data = (1 - filter->alpha) * filter->data + filter->alpha * data;
}

void filter_simple_low_pass_filter_init(simple_low_pass_filter_3f_t *filter, float alpha) {
    // Allocate memory for the data based on the provided size
    filter->data.value[0] = 0;
    filter->data.value[1] = 0;
    filter->data.value[2] = 0;
    filter->alpha = alpha;
}

void filter_simple_low_pass_filter_update(simple_low_pass_filter_3f_t *filter, matrix_3f_t *data) {
    // For each data point, apply the low-pass filter formula
    // which is a weighted average between the existing data and new data
    filter->data.value[0] = (1 - filter->alpha) * filter->data.value[0] + filter->alpha * data->value[0];
    filter->data.value[1] = (1 - filter->alpha) * filter->data.value[1] + filter->alpha * data->value[1];
    filter->data.value[2] = (1 - filter->alpha) * filter->data.value[2] + filter->alpha * data->value[2];
}

void filter_mean_accumulator_init(mean_accumulator_t *filter) {
    filter->data = 0;
    filter->count = 0;
}
void filter_mean_accumulator_add(mean_accumulator_t *filter, int32_t data) {
    filter->data += data;
    filter->count++;
}

uint16_t filter_mean_accumulator_calculate(mean_accumulator_t *filter, int16_t default_value) {
    if (filter->count) {
        uint16_t result = filter->data / filter->count;
        filter_mean_accumulator_init(filter);
        return result;
    }
    return default_value;
}

void filter_ekf_init(float alpha) {
    filter_simple_low_pass_filter_init(&filter_ekf_low_pass_filter_acc, alpha);
}

void filter_ekf_update(matrix_3f_t *gyro, matrix_3f_t *acc) {
    // predictions steps
    filter_ekf_state.value[0] += gyro->value[0] * LOOP_TIME;
    filter_ekf_state.value[1] += gyro->value[1] * LOOP_TIME;
    filter_ekf_state.value[2] += gyro->value[2] * LOOP_TIME;

    // normalizing yaw angle
    if (filter_ekf_state.value[FD_YAW] > 180.0) {
        filter_ekf_state.value[FD_YAW] -= 360.0;
    } else if (filter_ekf_state.value[FD_YAW] < -180.0) {
        filter_ekf_state.value[FD_YAW] += 360.0;
    }

    // Jacobians and other necessary matrices should be computed here

    math_matrix_add_f(&filter_ekf_P, &filter_ekf_Q, &filter_ekf_P);

    // update steps
    filter_simple_low_pass_filter_update(&filter_ekf_low_pass_filter_acc, acc);
    // Compute measurement residual
    filter_ekf_residual.value[0] = filter_ekf_low_pass_filter_acc.data.value[0] - filter_ekf_state.value[0];
    filter_ekf_residual.value[1] = filter_ekf_low_pass_filter_acc.data.value[1] - filter_ekf_state.value[1];
    filter_ekf_residual.value[2] = filter_ekf_low_pass_filter_acc.data.value[2] - filter_ekf_state.value[2];

    // Compute kalman gain
    math_matrix_add_f(&filter_ekf_P, &filter_ekf_R, &filter_tmp_matrix_3x3f_1);
    math_matrix_inverse_f(&filter_tmp_matrix_3x3f_1, &filter_tmp_matrix_3x3f_2);
    math_matrix_multiply_f(&filter_ekf_P, &filter_tmp_matrix_3x3f_2, &filter_ekf_K);

    // Update state estimate
    math_matrix_multiply_f(&filter_ekf_K, &filter_ekf_residual, &filter_tmp_matrix_3f_1);
    math_matrix_add_f(&filter_ekf_state, &filter_tmp_matrix_3f_1, &filter_ekf_state);

    // Update error covariance
    math_matrix_substract_f(&filter_matrix_eye, &filter_ekf_K, &filter_tmp_matrix_3x3f_1);
    math_matrix_multiply_f(&filter_tmp_matrix_3x3f_1, &filter_ekf_P, &filter_tmp_matrix_3x3f_2);
    math_matrix_copy_f(&filter_tmp_matrix_3x3f_2, &filter_ekf_P);
}

matrix_3f_t *filter_ekf_get_state() {
    return &filter_ekf_state;
}
