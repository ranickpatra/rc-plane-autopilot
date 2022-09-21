#include "./receiver.h"

void Receiver::init()
{
#ifdef SERIAL_MONITOR
    Serial.begin(115200);
#else
    this->IBus.begin(Serial);
#endif
    // set initial value
    for (uint16_t i = 0; i < RECEIVER_CHANNEL_COUNT; i++)
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
    if (Serial.available() > 0)
    {
        char ch = Serial.read();
        uint16_t pwmValue = this->channel[0];
        while (Serial.available() > 0)
            Serial.read();

        if (ch == '1')
            pwmValue += 10;
        else if (ch == '0')
            pwmValue -= 10;
        for (int i = 0; i < RECEIVER_CHANNEL_COUNT; i++)
        {
            this->channel[i] = pwmValue;
        }

        Serial.println(this->channel[2]);
    }
#else
/**
 * get data from receiver using IBUS
 * 
 */
    for (uint16_t i = 0; i < RECEIVER_CHANNEL_COUNT; i++)
    {
        this->channel[i] = this->IBus.readChannel(i);
        this->channel[i] = (int)(this->channel[i] > 2000 ? 2000 : (this->channel[i] < 1000 ? 1000 : this->channel[i]));
    }

    if(this->channel[1] > 1200)
        digitalWrite(13, HIGH);
    else
        digitalWrite(13, LOW);
#endif
}