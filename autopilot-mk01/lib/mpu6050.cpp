#include "mpu6050.h"

void MPU6050::init()
{
    Wire.begin(); // Initiate the Wire library and join the I2C bus as a master

    /*set clockspeed 400KHz for fast access default is 100 KHz
       if you want to use default one just comment out line 27
    */
    Wire.setClock(400000);
    delay(500); // wait for some time to get ready MPU6050

    // set up the MPU6050
    Wire.beginTransmission(MPU6060_ADDRESS); // start communicating with mpu6050
    Wire.write(0x6B);             // we want to write in 0x6B register which is powermanagement register
    Wire.write(0x00);             // write 0x00 to that register
    Wire.endTransmission();       // end the transmission

    Wire.beginTransmission(MPU6060_ADDRESS); // start communicating with mpu6050
    Wire.write(0x1B);             // we want to write in 0x1B which is gyro configaration register
    Wire.write(0x08);             // set the gyro for 500 deg pre sec full scale
    Wire.endTransmission();       // end the transmission

    Wire.beginTransmission(MPU6060_ADDRESS); // start communicating with mpu6050
    Wire.write(0x1C);             // we want to write in 0x1C register which is Accelerometer Configuration register
    Wire.write(0x10);             // set the accelerometer with +-8g
    Wire.endTransmission();       // end the transmission

    Wire.beginTransmission(MPU6060_ADDRESS); // start communicating with mpu6050
    Wire.write(0x1A);             // we want to write in 0x1A which is used to gyro and accelerometer synchronizing
    Wire.write(0x03);             // write 0x03 in that register
    Wire.endTransmission();       // end the transmission
}

void MPU6050::read()
{
    Wire.beginTransmission(MPU6060_ADDRESS); // start communicating with MPU6050
    Wire.write(0x3B);             // start reading from 0x3B
    Wire.endTransmission();       // end the transmission
    Wire.requestFrom(MPU6060_ADDRESS, 14);   // request 14 bytes
    while (Wire.available() < 14)
        ;                                                    // wait until 14 bytes are received
    this->accelerometer[0] = Wire.read() << 8 | Wire.read(); // read acc x data
    this->accelerometer[1] = Wire.read() << 8 | Wire.read(); // read acc y data
    this->accelerometer[2] = Wire.read() << 8 | Wire.read(); // read acc z data
    this->temperature = Wire.read() << 8 | Wire.read();      // read temperature data [we don't use it here]
    this->gyro[1] = Wire.read() << 8 | Wire.read();          // read gyro pitch data
    this->gyro[0] = Wire.read() << 8 | Wire.read();          // read gyro roll data
    this->gyro[2] = Wire.read() << 8 | Wire.read();          // read gyro yaw data

    // substract the calibration value to get proper data
    this->gyro[0] -= this->gyro_calibration[0];
    this->gyro[1] -= this->gyro_calibration[1];
    this->gyro[2] -= this->gyro_calibration[2];
}