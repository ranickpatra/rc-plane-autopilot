#include <Arduino.h>
#include "../helper/constants.h"

#ifndef RECEIVER_CHANNEL_COUNT
#define RECEIVER_CHANNEL_COUNT 4
#endif

#ifndef _RECEIVER_
#define _RECEIVER_

class Receiver {
    public:
        uint16_t channel[RECEIVER_CHANNEL_COUNT];    // modified data from receiver 1000 -> 2000
        void init();
        void update();
};
#endif

