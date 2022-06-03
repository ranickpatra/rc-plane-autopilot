#include "MPU6050.h"

// MPU6050::MPU6050(uint8_t address){
//     devAddr = address;
// }

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050::initialize()
{
  // set up the MPU6050
  Wire.beginTransmission(devAddr);   // start communicating with mpu6050
  Wire.write(MPU6050_RA_PWR_MGMT_1); // we want to write in 0x6B register which is powermanagement register
  Wire.write(0x00);                  // write 0x00 to that register
  Wire.endTransmission();            // end the transmission

  Wire.beginTransmission(devAddr);    // start communicating with mpu6050
  Wire.write(MPU6050_RA_GYRO_CONFIG); // we want to write in 0x1B which is gyro configaration register
  Wire.write(0x08);                   // set the gyro for 500 deg pre sec full scale
  Wire.endTransmission();             // end the transmission

  Wire.beginTransmission(devAddr); // start communicating with mpu6050
  Wire.write(0x1C);                // we want to write in 0x1C register which is Accelerometer Configuration register
  Wire.write(0x10);                // set the accelerometer with +-8g
  Wire.endTransmission();          // end the transmission

  Wire.beginTransmission(0x68);        // start communicating with mpu6050
  Wire.write(MPU6050_RA_ACCEL_CONFIG); // we want to write in 0x1A which is used to gyro and accelerometer synchronizing
  Wire.write(0x03);                    // write 0x03 in that register
  Wire.endTransmission();              // end the transmission

  // random check registers wheather the values is written correctly or not
  Wire.beginTransmission(devAddr);
  Wire.write(MPU6050_RA_GYRO_CONFIG);
  Wire.endTransmission();
  Wire.beginTransmission(devAddr);
  Wire.requestFrom(devAddr, 1);
  while (Wire.available() < 1)
    ;
  byte data = Wire.read();
  Wire.endTransmission();
  if (data != 0x08)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) // if not written this loop continues untill a next reset takes place
      delay(10);
  }
  // random check registers wheather the values is written correctly or not
  Wire.beginTransmission(devAddr);
  Wire.write(MPU6050_RA_ACCEL_CONFIG);
  Wire.endTransmission();
  Wire.beginTransmission(devAddr);
  Wire.requestFrom(devAddr, 1);
  while (Wire.available() < 1)
    ;
  data = Wire.read();
  Wire.endTransmission();
  if (data != 0x10)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) // if not written this loop continues untill a next reset takes place
      delay(10);
  }
}

// get raw data from mpu6050
void MPU6050::dump()
{
  Wire.beginTransmission(devAddr);     // start communicating with MPU6050
  Wire.write(MPU6050_RA_ACCEL_XOUT_H); // start reading from 0x3B
  Wire.endTransmission();              // end the transmission
  Wire.requestFrom(0x68, 14);          // request 14 bytes
  while (Wire.available() < 14)
    ; // wait until 14 bytes are received

  accel[0] = Wire.read() << 8 | Wire.read();    // read acc x data
  accel[1] = Wire.read() << 8 | Wire.read();    // read acc y data
  accel[2] = Wire.read() << 8 | Wire.read();    // read acc z data
  temperature = Wire.read() << 8 | Wire.read(); // read temperature data [we don't use it here]
  gyro[1] = Wire.read() << 8 | Wire.read();     // read gyro pitch data
  gyro[0] = Wire.read() << 8 | Wire.read();     // read gyro roll data
  gyro[2] = Wire.read() << 8 | Wire.read();     // read gyro yaw data

  // substract calibrated value
  gyro[0] -= gyro_cal[0]; // yaw
  gyro[1] -= gyro_cal[1]; // pitch
  gyro[2] -= gyro_cal[2]; // role
}