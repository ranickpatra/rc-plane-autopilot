#include "./pid.h"

PidConstant::PidConstant(double kp, double ki, double kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}