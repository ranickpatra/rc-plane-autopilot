#include <Arduino.h>
#include "MPU6050.h"

#ifndef _UAV_H_
#define _UAV_H_

class UAV
{

public:
    void init();

    void set_orientation(float* ypr);    // set the orentation to achieve
    void fly(); // fly the uav

private:
    // sensors
    MPU6050 mpu;

    // controlls
    uint16_t prop;
    uint16_t left_aileron, right_aileron, ruddar, elevator;
    float desired_ypr[3], current_ypr[3];
    float error_ypr[3] = {0,0,0,}, prev_error_ypr[3] = {0,0,0,};

    const float pid_ypr_KP[3] = {0,0,0}, pid_ypr_KI[3] = {0,0,0}, pid_ypr_KD[3] = {0,0,0};
    float pid_ypr_P[3], pid_ypr_I[3], pid_ypr_D[3], pid_ypr[3];


    /**
     * FUNCTIONS
     */

    void caculate_pid(); // PID for UAV

};

#endif