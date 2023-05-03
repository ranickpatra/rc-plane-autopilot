#include "./receiver.h"

void Receiver::init()
{
#ifdef SERIAL_MONITOR
    Serial.begin(115200);
#else
    // this->IBus.begin(Serial);
#endif
    // set initial value
    for (uint8_t i = 0; i < RECEIVER_CHANNEL_COUNT; i++)
    {
        this->channel[i] = 1000;
    }
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
}

void Receiver::update()
{
#ifdef SERIAL_MONITOR
    /**
     * get data from serial port for testing
     *
     */
    Serial.println(this->channel[2]);
#else
    if (this->channel[2] > 1500)
        digitalWrite(LED_BUILTIN, HIGH);
    else
        digitalWrite(LED_BUILTIN, LOW);
#endif
}