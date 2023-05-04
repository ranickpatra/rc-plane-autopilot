#include "./receiver.h"

void Receiver::init()
{
    // set initial value
    for (uint8_t i = 0; i < RECEIVER_CHANNEL_COUNT; i++)
    {
        this->channel[i] = 1000;
    }
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void Receiver::update()
{
    if (this->channel[2] > 1500)
        digitalWrite(LED_BUILTIN, HIGH);
    else
        digitalWrite(LED_BUILTIN, LOW);
}