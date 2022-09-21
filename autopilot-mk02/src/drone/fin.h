#include <Arduino.h>

#ifndef _FIN_
#define _FIN_

class Fin {
    public:
        double angle;
        double minAngle, maxAngle;
        
        Fin(double, double, uint16_t);
        void update();
        uint16_t getPWM();
    
    private:
        uint16_t center;
};

#endif