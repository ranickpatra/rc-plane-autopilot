#include <Arduino.h>

#ifndef _FIN_
#define _FIN_

class Fin {
    public:
        uint8_t pin;
        int16_t angle;
        int16_t minAngle, maxAngle;

        Fin(uint8_t, int16_t, int16_t);
        void update();
};

#endif