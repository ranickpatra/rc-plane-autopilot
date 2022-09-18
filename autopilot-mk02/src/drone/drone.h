#include <Arduino.h>
#include "./fin.h"
#include "./propeller.h"
#include "../helper/pid.h"

#ifndef _DRONE_
#define _DRONE_

class Drone
{

public:
    Propeller *propeller;
    Fin *fin1, *fin2, *fin3, *fin4;
    Drone(Propeller *, Fin *, Fin *, Fin *, Fin *);
    void setPIDConstants(double, double, double);
    void update();

private:
    PidConstant pidKYaw = PidConstant(0.0,0.0,0.0);
    PidConstant pidKPitchRoll = PidConstant(0.0,0.0,0.0);

    double targetYPR[3] = {0.0, 0.0, 0.0};  // target angle
    double pidProportionYPR[3] = {0.0,0.0,0.0};
    double pidIntegralYPR[3] = {0.0,0.0,0.0};
    double pidDerevativeYPR[3] = {0.0,0.0,0.0};

    double errorYPR[3] = {0.0, 0.0, 0.0};
    double previousErrorYPR[3] = {0.0, 0.0, 0.0};
    double deltaErrorYPR[3] = {0.0, 0.0, 0.0};

    void calculatePID(); // calculate pid for drone
};

#endif