#include <Arduino.h>

#ifndef _PROPELLER_
#define _PROPELLER_
class Propeller {

    public:
        double speed;
        void update();
        uint16_t getPWM();
};

#endif