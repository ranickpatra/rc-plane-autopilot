#include "pid.h"

#include "common/axis.h"
#include "common/craft.h"
#include "common/filter.h"
#include "io/propeller.h"
#include "physics.h"
#include "sensors/prop_rpm_sensor.h"

pid_coefficient_t pid_coefficients[FLIGHT_DYMANICS_INDEX_COUNT] = {
        {.Kp = 30.0, .Ki = 0.01, .Kd = 10.0},
        {.Kp = 30.0, .Ki = 0.01, .Kd = 10.0},
        {.Kp = 10.0, .Ki = 0.0, .Kd = 5.0},
};

pid_data_t pid_data[FLIGHT_DYMANICS_INDEX_COUNT] = {
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
};

// float factor = 0.001;
double pid_factor = 0.0017696;

matrix_3f_t pid_target_angles = {.value = {0.0, 0.0, 0.0}};
float pid_errors[FLIGHT_DYMANICS_INDEX_COUNT], pid_prev_errors[FLIGHT_DYMANICS_INDEX_COUNT];

void pid_init() {
}

void pid_set_target_angle(float roll, float pitch, float yaw) {
    pid_target_angles.value[FD_ROLL] = roll;
    pid_target_angles.value[FD_PITCH] = pitch;
    pid_target_angles.value[FD_YAW] = yaw;
}

// Function to update the PID controller with the current drone angles
void pid_update(matrix_3f_t* angles) {
    // Calculate the error for each axis by subtracting the current angle from the target angle
    pid_errors[FD_ROLL] = pid_target_angles.value[FD_ROLL] - angles->value[FD_ROLL];
    pid_errors[FD_PITCH] = pid_target_angles.value[FD_PITCH] - angles->value[FD_PITCH];
    pid_errors[FD_YAW] = pid_target_angles.value[FD_YAW] - angles->value[FD_YAW];

    // normalize
    if (pid_errors[FD_YAW] > 180.0) {
        pid_errors[FD_YAW] -= 360.0;
    } else if (pid_errors[FD_YAW] < -180.0) {
        pid_errors[FD_YAW] += 360.0;
    }

    // Calculate the Proportional term for each axis using the corresponding PID coefficient
    pid_data[FD_ROLL].P = pid_coefficients[FD_ROLL].Kp * pid_errors[FD_ROLL];
    pid_data[FD_PITCH].P = pid_coefficients[FD_PITCH].Kp * pid_errors[FD_PITCH];
    pid_data[FD_YAW].P = pid_coefficients[FD_YAW].Kp * pid_errors[FD_YAW];

    // Calculate the Integral term for roll. Reset to 0 if error is larger than threshold to prevent windup
    if (abs(pid_errors[FD_ROLL]) < 5.0)
        pid_data[FD_ROLL].I += pid_coefficients[FD_ROLL].Ki * pid_errors[FD_ROLL];
    else
        pid_data[FD_ROLL].I = 0;

    // Calculate the Integral term for pitch, with the same windup prevention mechanism
    if (abs(pid_errors[FD_PITCH]) < 5.0)
        pid_data[FD_PITCH].I += pid_coefficients[FD_PITCH].Ki * pid_errors[FD_PITCH];
    else
        pid_data[FD_PITCH].I = 0;

    // Calculate the Integral term for yaw, with the same windup prevention mechanism
    if (abs(pid_errors[FD_YAW]) < 5.0)
        pid_data[FD_YAW].I += pid_coefficients[FD_YAW].Ki * pid_errors[FD_YAW];
    else
        pid_data[FD_YAW].I = 0;

    // Calculate the Derivative term for each axis using the rate of change of error and a time constant
    pid_data[FD_ROLL].D = pid_coefficients[FD_ROLL].Kd * (pid_errors[FD_ROLL] - pid_prev_errors[FD_ROLL]) * LOOP_TIME_INVERSE;
    pid_data[FD_PITCH].D = pid_coefficients[FD_PITCH].Kd * (pid_errors[FD_PITCH] - pid_prev_errors[FD_PITCH]) * LOOP_TIME_INVERSE;
    pid_data[FD_YAW].D = pid_coefficients[FD_YAW].Kd * (pid_errors[FD_YAW] - pid_prev_errors[FD_YAW]) * LOOP_TIME_INVERSE;

    // Sum the PID terms to get the total control output for each axis
    pid_data[FD_ROLL].sum = (pid_data[FD_ROLL].P + pid_data[FD_ROLL].I + pid_data[FD_ROLL].D) * pid_factor;
    pid_data[FD_PITCH].sum = (pid_data[FD_PITCH].P + pid_data[FD_PITCH].I + pid_data[FD_PITCH].D) * pid_factor;
    pid_data[FD_YAW].sum = (pid_data[FD_YAW].P + pid_data[FD_YAW].I + pid_data[FD_YAW].D) * pid_factor;

    // Update the previous errors for the next cycle
    pid_prev_errors[FD_ROLL] = pid_errors[FD_ROLL];
    pid_prev_errors[FD_PITCH] = pid_errors[FD_PITCH];
    pid_prev_errors[FD_YAW] = pid_errors[FD_YAW];
}

pid_data_t* pid_get_data() {
    return (pid_data_t*)&pid_data;
}

void pid_set_p(float p) {
    pid_coefficients[FD_ROLL].Kp = p;
    pid_coefficients[FD_PITCH].Kp = p;
}

void pid_set_i(float i) {
    pid_coefficients[FD_ROLL].Ki = i;
    pid_coefficients[FD_PITCH].Ki = i;
}

void pid_set_d(float d) {
    pid_coefficients[FD_ROLL].Kd = d;
    pid_coefficients[FD_PITCH].Kd = d;
}

void pid_set_p_yaw(float p) {
    pid_coefficients[FD_YAW].Kp = p;
}

void pid_set_d_yaw(float d) {
    pid_coefficients[FD_YAW].Kd = d;
}
