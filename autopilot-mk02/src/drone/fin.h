#include <Arduino.h>

#ifndef _FIN_
#define _FIN_

class Fin {
    public:
        double angle;
        double min_angle, max_angle;
        
        Fin(double, double, uint16_t);
        void update();
        uint16_t get_microseconds();
    
    private:
        uint16_t center;
        uint16_t signal;
};

#endif