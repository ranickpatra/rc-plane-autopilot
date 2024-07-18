#pragma once

#include <stdint.h>

#include "common/maths.h"

struct pid_data_t {
    float P;
    float I;
    float D;

    float sum;
};

struct pid_coefficient_t {
    float Kp;
    float Ki;
    float Kd;
};

void pid_init();
void pid_set_target_angle(float roll, float pitch, float yaw);
void pid_update(matrix_3f_t* angles);
void pid_set_p(float p);
void pid_set_d(float d);
void pid_set_i(float i);
void pid_set_p_yaw(float p);
void pid_set_d_yaw(float d);

pid_data_t* pid_get_data();
