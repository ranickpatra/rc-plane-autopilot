#include <Arduino.h>

#ifndef _PROPELLER_
#define _PROPELLER_
class Propeller {

    public:
        int16_t speed;
        int16_t minSpeed, maxSpeed;

        Propeller(int16_t, int16_t);
        void update();
};

#endif