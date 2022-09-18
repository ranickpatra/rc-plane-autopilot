
#ifndef _PID_CONSTANT_
#define _PID_CONSTANT_
class PidConstant
{
public:
    double kp, ki, kd;
    PidConstant(double, double, double);
};

#endif