#include <Arduino.h>
#include <Wire.h>

#define MPU6060_ADDRESS 0x68

class MPU6050
{
    private:
    uint8_t gyro_calibration[3] = {0,0,0};
    uint8_t gyro[3] = {0,0,0};
    uint8_t accelerometer[3] = {0,0,0};
    uint8_t temperature = 0;


    public:

    void init();
    void read();
    
};