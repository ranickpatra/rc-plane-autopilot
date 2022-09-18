#include "./drone.h"

Drone::Drone(Propeller *propeller,
             Fin *fin1, Fin *fin2, Fin *fin3, Fin *fin4)
{
    this->propeller = propeller;
    this->fin1 = fin1;
    this->fin2 = fin2;
    this->fin3 = fin3;
    this->fin4 = fin4;
}

void Drone::setPIDConstants(double kp, double ki, double kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void Drone::update()
{
    this->calculatePID();   // pid calculation
}

// calculate the pid for drone
void Drone::calculatePID()
{
    // calculate errors
    this->errorYPR[0] = this->targetAngle[0] - this->currentAngle[0];
    this->errorYPR[1] = this->targetAngle[1] - this->currentAngle[1];
    this->errorYPR[2] = this->targetAngle[2] - this->currentAngle[2];

    // delta error
    this->deltaErrorYPR[0] = this->previousErrorYPR[0] this->errorYPR[0];
    this->deltaErrorYPR[1] = this->previousErrorYPR[1] this->errorYPR[1];
    this->deltaErrorYPR[2] = this->previousErrorYPR[2] this->errorYPR[2];

    // pid Proportion
    this->pidProportionYPR[0] = this->errorYPR[0] * this->pidKYaw.kp;
    this->pidProportionYPR[1] = this->errorYPR[1] * this->pidKPitchRoll.kp;
    this->pidProportionYPR[2] = this->errorYPR[2] * this->pidKPitchRoll.kp;

    // pid Integral
    if((this->errorYPR[0] * (this->errorYPR[0] < 0 ? -1 : 1)) < 0.1 )
        this->pidIntegralYPR[0] += this->errorYPR[0] * this->pidKYaw.ki;
    else
        this->pidIntegralYPR[0] = 0;
    if((this->errorYPR[1] * (this->errorYPR[1] < 0 ? -1 : 1)) < 0.1 )
        this->pidIntegralYPR[1] += this->errorYPR[1] * this->pidKPitchRoll.ki;
    else
        this->pidIntegralYPR[1] = 0;
    if((this->errorYPR[2] * (this->errorYPR[2] < 0 ? -1 : 1)) < 0.1 )
        this->pidIntegralYPR[2] += this->errorYPR[2] * this->pidKPitchRoll.ki;
    else
        this->pidIntegralYPR[2] = 0;

    // pid derevative
    this->pidDerevativeYPR[0] = this->deltaErrorYPR[0] * this->pidKYaw.kd;
    this->pidDerevativeYPR[1] = this->deltaErrorYPR[1] * this->pidKPitchRoll.kd;
    this->pidDerevativeYPR[2] = this->deltaErrorYPR[2] * this->pidKPitchRoll.kd;

    // set error to previous error
    this->previousError[0] = this->errorYPR[0];
    this->previousError[1] = this->errorYPR[1];
    this->previousError[2] = this->errorYPR[2];
}