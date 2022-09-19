#include "./receiver.h"

void Receiver::init() {
    this->IBus.begin(Serial);
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
}

void Receiver::update()
{
    for (int i = 0; i < RECEIVER_CHANNEL_COUNT; i++)
    {
        this->channel[i] = this->IBus.readChannel(i);
        this->channel[i] = (int)(this->channel[i] > 2000 ? 2000 : (this->channel[i] < 1000 ? 1000 : this->channel[i]));
    }

    if (this->channel[2] > 1200)
        digitalWrite(13, HIGH);
    else
        digitalWrite(13, LOW);
}