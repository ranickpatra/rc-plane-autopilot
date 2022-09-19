#include <Arduino.h>

#ifndef _FIN_
#define _FIN_

class Fin {
    public:
        double angle;
        double minAngle, maxAngle;

        Fin(double, double);
        void update();
};

#endif