#include "uav.h"

// init UAV
void UAV::init()
{
    // initilize mpu6050
    mpu.initialize();
    delay(100); // wait for sometime
}

void UAV::set_orientation(float *ypr)
{
    this->desired_ypr[0] = ypr[0];
    this->desired_ypr[1] = ypr[1];
    this->desired_ypr[2] = ypr[2];
}

// fly the uav
void UAV::fly(uint16_t deltaT)
{
    // get imu orientation
    this->get_IMU_orientation(deltaT);


    // calculate PID
    this->caculate_pid();
    
}

// calculation PID
void UAV::caculate_pid()
{
    // calculate error
    error_ypr[0] = desired_ypr[0] - current_ypr[0];
    error_ypr[1] = desired_ypr[1] - current_ypr[1];
    error_ypr[2] = desired_ypr[2] - current_ypr[2];

    // calculate PID
    pid_ypr_P[0] = pid_ypr_KP[0] * error_ypr[0];
    pid_ypr_P[1] = pid_ypr_KP[1] * error_ypr[1];
    pid_ypr_P[2] = pid_ypr_KP[2] * error_ypr[2];

    pid_ypr_D[0] = pid_ypr_KD[0] * (prev_error_ypr[0] - error_ypr[0]);
    pid_ypr_D[1] = pid_ypr_KD[1] * (prev_error_ypr[1] - error_ypr[1]);
    pid_ypr_D[2] = pid_ypr_KD[2] * (prev_error_ypr[2] - error_ypr[2]);


    pid_ypr[0] = pid_ypr_P[0] + pid_ypr_I[0] + pid_ypr_D[0];
    pid_ypr[1] = pid_ypr_P[1] + pid_ypr_I[1] + pid_ypr_D[1];
    pid_ypr[2] = pid_ypr_P[2] + pid_ypr_I[2] + pid_ypr_D[2];


    // current error to previous error
    prev_error_ypr[0] = error_ypr[0];
    prev_error_ypr[1] = error_ypr[1];
    prev_error_ypr[2] = error_ypr[2];
}


/// get orientation from IMU
void UAV::get_IMU_orientation(uint16_t deltaT) {
    mpu.read(); // 620us
    mpu.getGyroData(this->gyro, deltaT);
}

