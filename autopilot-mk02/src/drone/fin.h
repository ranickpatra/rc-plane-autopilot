#include <Arduino.h>

#ifndef _FIN_
#define _FIN_

class Fin {
    public:
        int8_t angle;
        int8_t min_angle, max_angle;
        
        Fin(int8_t, int8_t, uint16_t);
        void update();
        uint16_t get_microseconds();
    
    private:
        uint16_t center;
};

#endif