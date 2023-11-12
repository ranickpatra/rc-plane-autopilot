#include "pid.h"

#include "common/axis.h"
#include "common/craft.h"
#include "common/filter.h"

// #define USE_LOWPASS_FILTER

// pid_coefficient_t pid_coefficients[FLIGHT_DYMANICS_INDEX_COUNT] = {
//         {5.0, 0.0, 1},
//         {5.0, 0.0, 1},
//         {0.0, 0.0, 0.0},
// };
pid_coefficient_t pid_coefficients[FLIGHT_DYMANICS_INDEX_COUNT] = {
        {.Kp=0.0, .Ki=0.0, .Kd=0.0},
        {.Kp=0.0, .Ki=0.0, .Kd=0.0},
        {.Kp=0.0, .Ki=0.0, .Kd=0.0},
};
pid_data_t pid_data[FLIGHT_DYMANICS_INDEX_COUNT] = {
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
};

matrix_3f_t target_angles = {.value={0.0, 0.0, 0.0}};
float errors[FLIGHT_DYMANICS_INDEX_COUNT], prev_errors[FLIGHT_DYMANICS_INDEX_COUNT];
matrix_3f_t errors_dt;

simple_low_pass_filter_3f_t pid_error_dt_lowpass_filter;

void pid_init() {
    simple_low_pass_filter_init(&pid_error_dt_lowpass_filter, 0.4);
}

void pid_update(matrix_3f_t* angles) {
    errors[FD_ROLL] = target_angles.value[FD_ROLL] - angles->value[FD_ROLL];
    errors[FD_PITCH] = target_angles.value[FD_PITCH] - angles->value[FD_PITCH];
    errors[FD_YAW] = target_angles.value[FD_YAW] - angles->value[FD_YAW];

    pid_data[FD_ROLL].P = pid_coefficients[FD_ROLL].Kp * errors[FD_ROLL];
    pid_data[FD_PITCH].P = pid_coefficients[FD_PITCH].Kp * errors[FD_PITCH];
    pid_data[FD_YAW].P = pid_coefficients[FD_YAW].Kp * errors[FD_YAW];

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

#ifdef USE_LOWPASS_FILTER
    errors_dt.value[FD_ROLL] = prev_errors[FD_ROLL] - errors[FD_ROLL];
    errors_dt.value[FD_PITCH] = prev_errors[FD_PITCH] - errors[FD_PITCH];
    errors_dt.value[FD_YAW] = prev_errors[FD_YAW] - errors[FD_YAW];

    simple_low_pass_filter_update(&pid_error_dt_lowpass_filter, &errors_dt);

    pid_data[FD_ROLL].D = pid_coefficients[FD_ROLL].Kd * pid_error_dt_lowpass_filter.data.value[FD_ROLL] * LOOP_TIME_INVERSE;
    pid_data[FD_PITCH].D = pid_coefficients[FD_PITCH].Kd * pid_error_dt_lowpass_filter.data.value[FD_PITCH] * LOOP_TIME_INVERSE;
    pid_data[FD_YAW].D = pid_coefficients[FD_YAW].Kd * pid_error_dt_lowpass_filter.data.value[FD_YAW] * LOOP_TIME_INVERSE;
#else
    pid_data[FD_ROLL].D = pid_coefficients[FD_ROLL].Kd * (prev_errors[FD_ROLL] - errors[FD_ROLL]) * LOOP_TIME_INVERSE;
    pid_data[FD_PITCH].D = pid_coefficients[FD_PITCH].Kd * (prev_errors[FD_PITCH] - errors[FD_PITCH]) * LOOP_TIME_INVERSE;
    pid_data[FD_YAW].D = pid_coefficients[FD_YAW].Kd * (prev_errors[FD_YAW] - errors[FD_YAW]) * LOOP_TIME_INVERSE;
#endif

    pid_data[FD_ROLL].sum = pid_data[FD_ROLL].P + pid_data[FD_ROLL].I + pid_data[FD_ROLL].D;
    pid_data[FD_PITCH].sum = pid_data[FD_PITCH].P + pid_data[FD_PITCH].I + pid_data[FD_PITCH].D;
    pid_data[FD_YAW].sum = pid_data[FD_YAW].P + pid_data[FD_YAW].I + pid_data[FD_YAW].D;

    prev_errors[FD_ROLL] = errors[FD_ROLL];
    prev_errors[FD_PITCH] = errors[FD_PITCH];
    prev_errors[FD_YAW] = errors[FD_YAW];

    if (pid_data[FD_ROLL].sum > 45.0)
        pid_data[FD_ROLL].sum = 45.0;
    else if (pid_data[FD_ROLL].sum < -45.0)
        pid_data[FD_ROLL].sum = -45.0;
    if (pid_data[FD_PITCH].sum > 45.0)
        pid_data[FD_PITCH].sum = 45.0;
    else if (pid_data[FD_PITCH].sum < -45.0)
        pid_data[FD_PITCH].sum = -45.0;
    if (pid_data[FD_YAW].sum > 45.0)
        pid_data[FD_YAW].sum = 45.0;
    else if (pid_data[FD_YAW].sum < -45.0)
        pid_data[FD_YAW].sum = -45.0;
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