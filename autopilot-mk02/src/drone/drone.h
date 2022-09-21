#include "../helper/constants.h"
#include <Arduino.h>
#include "./fin.h"
#include "./propeller.h"
#include "../helper/pid.h"

#ifndef _DRONE_
#define _DRONE_

class Drone
{

public:
    Propeller propeller;
    Fin fin1 = Fin(-45.0, 45.0, 1570), 
        fin2 = Fin(-45.0, 45.0, 1420), 
        fin3 = Fin(-45.0, 45.0, 1380), 
        fin4 = Fin(-45.0, 45.0, 1540);
    unsigned long ltime = LOOP_TIME;
    Drone();
    // void setPIDConstants(double, double, double);
    void update();
    void setReceiverChannel(uint16_t *);

private:
    uint16_t *receiverChannel;
    PidConstant pidKYaw = PidConstant(0.0,0.0,0.0);
    PidConstant pidKPitchRoll = PidConstant(0.0,0.0,0.0);

    double targetAngle[3] = {0.0, 0.0, 0.0};  // target angle
    double currentAngle[3] = {0.0, 0.0, 0.0};  // target angle
    double pidProportionYPR[3] = {0.0,0.0,0.0};
    double pidIntegralYPR[3] = {0.0,0.0,0.0};
    double pidDerevativeYPR[3] = {0.0,0.0,0.0};

    double errorYPR[3] = {0.0, 0.0, 0.0};
    double previousErrorYPR[3] = {0.0, 0.0, 0.0};
    double deltaErrorYPR[3] = {0.0, 0.0, 0.0};

    void calculatePID(); // calculate pid for drone
};

#endif