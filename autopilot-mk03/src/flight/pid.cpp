#include "pid.h"

#include "common/axis.h"
#include "common/craft.h"
#include "common/filter.h"
#include "io/propeller.h"

// #define USE_LOWPASS_FILTER

// pid_coefficient_t pid_coefficients[FLIGHT_DYMANICS_INDEX_COUNT] = {
//         {5.0, 0.0, 1},
//         {5.0, 0.0, 1},
//         {0.0, 0.0, 0.0},
// };
pid_coefficient_t pid_coefficients[FLIGHT_DYMANICS_INDEX_COUNT] = {
        {.Kp = 0.0, .Ki = 0.0, .Kd = 0.0},
        {.Kp = 0.0, .Ki = 0.0, .Kd = 0.0},
        {.Kp = 0.0, .Ki = 0.0, .Kd = 0.0},
};
pid_data_t pid_data[FLIGHT_DYMANICS_INDEX_COUNT] = {
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
};

float prop_input_inverse_quad_value = 0;
float fin_max_range = 10.0;

matrix_3f_t target_angles = {.value = {0.0, 0.0, 0.0}};
float errors[FLIGHT_DYMANICS_INDEX_COUNT], prev_errors[FLIGHT_DYMANICS_INDEX_COUNT];
matrix_3f_t errors_dt;
uint8_t is_curve_used = 0;


void pid_init() {
}

void set_target_angle(float roll, float pitch, float yaw) {
    target_angles.value[FD_ROLL] = roll;
    target_angles.value[FD_PITCH] = pitch;
    target_angles.value[FD_YAW] = yaw;
}

void pid_set_angle(float f) {
    fin_max_range = f;
}

float pid_get_angle() {
    return fin_max_range;
}

void pid_set_with_curve(uint8_t i) {
    is_curve_used = i;
}

void pid_update(matrix_3f_t* angles) {
    errors[FD_ROLL] = target_angles.value[FD_ROLL] - angles->value[FD_ROLL];
    errors[FD_PITCH] = target_angles.value[FD_PITCH] - angles->value[FD_PITCH];
    errors[FD_YAW] = target_angles.value[FD_YAW] - angles->value[FD_YAW];

    pid_data[FD_ROLL].P = pid_coefficients[FD_ROLL].Kp * errors[FD_ROLL];
    pid_data[FD_PITCH].P = pid_coefficients[FD_PITCH].Kp * errors[FD_PITCH];
    pid_data[FD_YAW].P = pid_coefficients[FD_YAW].Kp * errors[FD_YAW];

    // errors_dt.value[FD_ROLL] = errors[FD_ROLL] - prev_errors[FD_ROLL];
    // errors_dt.value[FD_PITCH] = errors[FD_PITCH] - prev_errors[FD_PITCH];
    // errors_dt.value[FD_YAW] = errors[FD_YAW] - prev_errors[FD_YAW];


    if (errors[FD_ROLL] > -5.0 && errors[FD_ROLL] < 5.0)
        pid_data[FD_ROLL].I += pid_coefficients[FD_ROLL].Ki * errors[FD_ROLL];
    else
        pid_data[FD_ROLL].I = 0;

    if (errors[FD_PITCH] > -5.0 && errors[FD_PITCH] < 5.0)
        pid_data[FD_PITCH].I += pid_coefficients[FD_PITCH].Ki * errors[FD_PITCH];
    else
        pid_data[FD_PITCH].I = 0;

    if (errors[FD_YAW] > -5.0 && errors[FD_YAW] < 5.0)
        pid_data[FD_YAW].I += pid_coefficients[FD_YAW].Ki * errors[FD_YAW];
    else
        pid_data[FD_YAW].I = 0;

    pid_data[FD_ROLL].D = pid_coefficients[FD_ROLL].Kd * (errors[FD_ROLL] - prev_errors[FD_ROLL]) * LOOP_TIME_INVERSE;
    pid_data[FD_PITCH].D = pid_coefficients[FD_PITCH].Kd * (errors[FD_PITCH] - prev_errors[FD_PITCH]) * LOOP_TIME_INVERSE;
    pid_data[FD_YAW].D = pid_coefficients[FD_YAW].Kd * (errors[FD_YAW] - prev_errors[FD_YAW]) * LOOP_TIME_INVERSE;

    prop_input_inverse_quad_value = get_propeller_input();
    // prop_input_inverse_quad_value = 1.2 - prop_input_inverse_quad_value * prop_input_inverse_quad_value;
    prop_input_inverse_quad_value = 1.2 - prop_input_inverse_quad_value * prop_input_inverse_quad_value * prop_input_inverse_quad_value *
                                                  prop_input_inverse_quad_value;

    if(is_curve_used) {
        if (prop_input_inverse_quad_value > 1.0) prop_input_inverse_quad_value = 1.0;
    }
    else {
        prop_input_inverse_quad_value = 1.0;
    }

    pid_data[FD_ROLL].sum = (pid_data[FD_ROLL].P + pid_data[FD_ROLL].I + pid_data[FD_ROLL].D) * prop_input_inverse_quad_value;
    pid_data[FD_PITCH].sum = (pid_data[FD_PITCH].P + pid_data[FD_PITCH].I + pid_data[FD_PITCH].D) * prop_input_inverse_quad_value;
    pid_data[FD_YAW].sum = (pid_data[FD_YAW].P + pid_data[FD_YAW].I + pid_data[FD_YAW].D) * prop_input_inverse_quad_value;

    prev_errors[FD_ROLL] = errors[FD_ROLL];
    prev_errors[FD_PITCH] = errors[FD_PITCH];
    prev_errors[FD_YAW] = errors[FD_YAW];

    if (pid_data[FD_ROLL].sum > fin_max_range)
        pid_data[FD_ROLL].sum = fin_max_range;
    else if (pid_data[FD_ROLL].sum < -fin_max_range)
        pid_data[FD_ROLL].sum = -fin_max_range;
    if (pid_data[FD_PITCH].sum > fin_max_range)
        pid_data[FD_PITCH].sum = fin_max_range;
    else if (pid_data[FD_PITCH].sum < -fin_max_range)
        pid_data[FD_PITCH].sum = -fin_max_range;
    if (pid_data[FD_YAW].sum > fin_max_range)
        pid_data[FD_YAW].sum = fin_max_range;
    else if (pid_data[FD_YAW].sum < -fin_max_range)
        pid_data[FD_YAW].sum = -fin_max_range;
}

pid_data_t* pid_get_data() {
    return (pid_data_t*)&pid_data;
}

void pid_set_p(float p) {
    pid_coefficients[FD_ROLL].Kp = p;
    pid_coefficients[FD_PITCH].Kp = p;
}

void pid_set_d(float d) {
    pid_coefficients[FD_ROLL].Kd = d;
    pid_coefficients[FD_PITCH].Kd = d;
}

pid_coefficient_t* pid_get() {
    return pid_coefficients;
}