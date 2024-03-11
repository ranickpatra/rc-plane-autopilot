#include "pid.h"

#include "common/axis.h"
#include "common/craft.h"
#include "common/filter.h"
#include "io/propeller.h"
#include "physics.h"
#include "sensors/prop_rpm_sensor.h"

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

float pid_fin_angles[4] = {0.0, 0.0, 0.0, 0.0};

// float factor = 0.001;
double factor = 0.0017696;

float prop_speed;

matrix_3f_t target_angles = {.value = {0.0, 0.0, 0.0}};
float errors[FLIGHT_DYMANICS_INDEX_COUNT], prev_errors[FLIGHT_DYMANICS_INDEX_COUNT];
matrix_3f_t errors_dt;

void pid_init() {
}

void set_target_angle(float roll, float pitch, float yaw) {
    target_angles.value[FD_ROLL] = roll;
    target_angles.value[FD_PITCH] = pitch;
    target_angles.value[FD_YAW] = yaw;
}

// Function to update the PID controller with the current drone angles
void pid_update(matrix_3f_t* angles) {
    // Calculate the error for each axis by subtracting the current angle from the target angle
    errors[FD_ROLL] = target_angles.value[FD_ROLL] - angles->value[FD_ROLL];
    errors[FD_PITCH] = target_angles.value[FD_PITCH] - angles->value[FD_PITCH];
    errors[FD_YAW] = target_angles.value[FD_YAW] - angles->value[FD_YAW];

    // Calculate the Proportional term for each axis using the corresponding PID coefficient
    pid_data[FD_ROLL].P = pid_coefficients[FD_ROLL].Kp * errors[FD_ROLL];
    pid_data[FD_PITCH].P = pid_coefficients[FD_PITCH].Kp * errors[FD_PITCH];
    pid_data[FD_YAW].P = pid_coefficients[FD_YAW].Kp * errors[FD_YAW];

    // The Derivative term calculations are commented out, might be a placeholder for future implementation
    // errors_dt.value[FD_ROLL] = errors[FD_ROLL] - prev_errors[FD_ROLL];
    // errors_dt.value[FD_PITCH] = errors[FD_PITCH] - prev_errors[FD_PITCH];
    // errors_dt.value[FD_YAW] = errors[FD_YAW] - prev_errors[FD_YAW];

    // Calculate the Integral term for roll. Reset to 0 if error is larger than threshold to prevent windup
    if (abs(errors[FD_ROLL]) < 5.0)
        pid_data[FD_ROLL].I += pid_coefficients[FD_ROLL].Ki * errors[FD_ROLL];
    else
        pid_data[FD_ROLL].I = 0;

    // Calculate the Integral term for pitch, with the same windup prevention mechanism
    if (abs(errors[FD_PITCH]) < 5.0)
        pid_data[FD_PITCH].I += pid_coefficients[FD_PITCH].Ki * errors[FD_PITCH];
    else
        pid_data[FD_PITCH].I = 0;

    // Calculate the Integral term for yaw, with the same windup prevention mechanism
    if (abs(errors[FD_YAW]) < 5.0)
        pid_data[FD_YAW].I += pid_coefficients[FD_YAW].Ki * errors[FD_YAW];
    else
        pid_data[FD_YAW].I = 0;

    // Calculate the Derivative term for each axis using the rate of change of error and a time constant
    pid_data[FD_ROLL].D = pid_coefficients[FD_ROLL].Kd * (errors[FD_ROLL] - prev_errors[FD_ROLL]) * LOOP_TIME_INVERSE;
    pid_data[FD_PITCH].D = pid_coefficients[FD_PITCH].Kd * (errors[FD_PITCH] - prev_errors[FD_PITCH]) * LOOP_TIME_INVERSE;
    pid_data[FD_YAW].D = pid_coefficients[FD_YAW].Kd * (errors[FD_YAW] - prev_errors[FD_YAW]) * LOOP_TIME_INVERSE;

    // Sum the PID terms to get the total control output for each axis
    pid_data[FD_ROLL].sum = pid_data[FD_ROLL].P + pid_data[FD_ROLL].I + pid_data[FD_ROLL].D;
    pid_data[FD_PITCH].sum = pid_data[FD_PITCH].P + pid_data[FD_PITCH].I + pid_data[FD_PITCH].D;
    pid_data[FD_YAW].sum = pid_data[FD_YAW].P + pid_data[FD_YAW].I + pid_data[FD_YAW].D;

    // Update the previous errors for the next cycle
    prev_errors[FD_ROLL] = errors[FD_ROLL];
    prev_errors[FD_PITCH] = errors[FD_PITCH];
    prev_errors[FD_YAW] = errors[FD_YAW];

    // Get the current propeller speed to calculate the fin angles
    prop_speed = propeller_rpm_sensor_get_speed_rps();

    // Convert the control outputs to fin angles considering the force and propeller speed
    // pid_data[FD_ROLL].sum = physics_get_fin_angle_from_force(pid_data[FD_ROLL].sum * 0.5 * factor, prop_speed);
    // pid_data[FD_PITCH].sum = physics_get_fin_angle_from_force(pid_data[FD_PITCH].sum * 0.5 * factor, prop_speed);

    // TODO calculate force for each fin
    pid_fin_angles[0] = (pid_data[FD_ROLL].sum * 0.5 + pid_data[FD_YAW].sum * 0.25) * factor;
    pid_fin_angles[1] = (pid_data[FD_PITCH].sum * 0.5 + pid_data[FD_YAW].sum * 0.25) * factor;
    pid_fin_angles[2] = (pid_data[FD_ROLL].sum * 0.5 - pid_data[FD_YAW].sum * 0.25) * factor;
    pid_fin_angles[3] = (pid_data[FD_PITCH].sum * 0.5 - pid_data[FD_YAW].sum * 0.25) * factor;

    pid_fin_angles[0] = physics_get_fin_angle_from_force(pid_fin_angles[0], prop_speed);
    pid_fin_angles[1] = -physics_get_fin_angle_from_force(pid_fin_angles[1], prop_speed);
    pid_fin_angles[2] = physics_get_fin_angle_from_force(pid_fin_angles[2], prop_speed);
    pid_fin_angles[3] = -physics_get_fin_angle_from_force(pid_fin_angles[3], prop_speed);
}

pid_data_t* pid_get_data() {
    return (pid_data_t*)&pid_data;
}

float* pid_get_fin_angle() {
    return pid_fin_angles;
}

void pid_set_p(float p) {
    pid_coefficients[FD_ROLL].Kp = p;
    pid_coefficients[FD_PITCH].Kp = p;
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

pid_coefficient_t* pid_get() {
    return pid_coefficients;
}