#include <Arduino.h>
#include <IBusBM.h>

#ifndef RECEIVER_CHANNEL_COUNT
#define RECEIVER_CHANNEL_COUNT 6
#endif

#ifndef _RECEIVER_
#define _RECEIVER_

class Receiver {
    public:
        int channel[RECEIVER_CHANNEL_COUNT];    // modified data from receiver 1000 -> 2000
        void init();
        void update();

    private:
        IBusBM IBus;
};
#endif

