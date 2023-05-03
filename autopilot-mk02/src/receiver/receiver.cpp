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
        uint16_t pwm_value = this->channel[0];
        while (Serial.available() > 0)
            Serial.read();

        if (ch == '1')
            pwm_value += 10;
        else if (ch == '0')
            pwm_value -= 10;
        for (int i = 0; i < RECEIVER_CHANNEL_COUNT; i++)
        {
            this->channel[i] = pwm_value;
        }

        Serial.println(this->channel[2]);
    }
#else
    /**
     * get data from receiver using IBUS
     *
     */
    for (uint8_t i = 0; i < RECEIVER_CHANNEL_COUNT; i++)
    {
        // this->channel[i] = this->IBus.readChannel(i);
        /**
         * 1st 10s 1200
         * next 10s 1500
         * next 10s 1700
         * repeat
         *
         */
        uint8_t ff = (millis() / 10000) % 3;
        switch (ff)
        {
        case 0:
            this->channel[i] = 1200;
            break;
        case 1:
            this->channel[i] = 1500;
            break;
        case 2:
            this->channel[i] = 1700;
            break;
        }

        if (this->channel[i] > 2000)
            this->channel[i] = 2000;
        else if (this->channel[i] < 1000)
            this->channel[i] = 1000;
    }

    this->channel[2] = 1000;

    if (this->channel[2] > 1200)
        digitalWrite(LED_BUILTIN, HIGH);
    else
        digitalWrite(LED_BUILTIN, LOW);
#endif
}