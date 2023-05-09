#include <Arduino.h>
#include "../helper/constants.h"
#include "./fin.h"
#include "./propeller.h"
#include "../helper/pid.h"

#ifndef _DRONE_
#define _DRONE_

class Drone
{

public:
    Propeller propeller;
    Fin fin1 = Fin(-45.0, 45.0, 1570); 
    Fin fin2 = Fin(-45.0, 45.0, 1420); 
    Fin fin3 = Fin(-45.0, 45.0, 1380); 
    Fin fin4 = Fin(-45.0, 45.0, 1540);
    Drone();
    // void setPIDConstants(double, double, double);
    void update();
    void set_receiver_channel(uint16_t *);

private:
    uint16_t *receiver_channel;
    PidConstant pid_k_yaw = PidConstant(0.0,0.0,0.0);
    PidConstant pid_k_pitch_roll = PidConstant(0.0,0.0,0.0);

    double target_angle[3] = {0.0, 0.0, 0.0};  // target angle
    double current_angle[3] = {0.0, 0.0, 0.0};  // target angle
    double pid_proportion_ypr[3] = {0.0,0.0,0.0};
    double pid_integral_ypr[3] = {0.0,0.0,0.0};
    double pid_derevative_ypr[3] = {0.0,0.0,0.0};

    double error_ypr[3] = {0.0, 0.0, 0.0};
    double previous_error_ypr[3] = {0.0, 0.0, 0.0};
    double delta_error_ypr[3] = {0.0, 0.0, 0.0};

    void calculate_pid(); // calculate pid for drone
};

#endif