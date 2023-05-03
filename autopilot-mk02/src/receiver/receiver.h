#include <Arduino.h>
#include <IBusBM.h>

#ifndef RECEIVER_CHANNEL_COUNT
#define RECEIVER_CHANNEL_COUNT 4
#endif

// to get data from serial monitor 
// #define SERIAL_MONITOR

#ifndef _RECEIVER_
#define _RECEIVER_

class Receiver {
    public:
        uint16_t channel[RECEIVER_CHANNEL_COUNT];    // modified data from receiver 1000 -> 2000
        void init();
        void update();

    private:
        #ifndef SERIAL_MONITOR
            IBusBM IBus;
        #endif

};
#endif

