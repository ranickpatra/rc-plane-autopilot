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

  Wire.beginTransmission(devAddr);     // start communicating with mpu6050
  Wire.write(MPU6050_RA_ACCEL_CONFIG); // we want to write in 0x1C register which is Accelerometer Configuration register
  Wire.write(0x10);                    // set the accelerometer with +-8g
  Wire.endTransmission();              // end the transmission

  Wire.beginTransmission(0x68);  // start communicating with mpu6050
  Wire.write(MPU6050_RA_CONFIG); // we want to write in 0x1A which is used to gyro and accelerometer synchronizing
  Wire.write(0x03);              // write 0x03 in that register
  Wire.endTransmission();        // end the transmission

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
void MPU6050::read()
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
  gyro[0] = Wire.read() << 8 | Wire.read();     // read gyro x data
  gyro[1] = Wire.read() << 8 | Wire.read();     // read gyro y data
  gyro[2] = Wire.read() << 8 | Wire.read();     // read gyro z data

  // substract calibrated value
  gyro[0] = (int16_t)(gyro[0] - gyro_cal[0]);
  gyro[1] = (int16_t)(gyro[1] - gyro_cal[1]);
  gyro[2] = (int16_t)(gyro[2] - gyro_cal[2]);
}

void MPU6050::getRawAccelData(VectorInt16 *accel)
{
  accel->x = this->accel[0];
  accel->y = this->accel[1];
  accel->z = this->accel[2];
}

void MPU6050::getRawGyroData(VectorInt16 *gyro)
{
  gyro->x = this->gyro[0];
  gyro->y = this->gyro[1];
  gyro->z = this->gyro[2];
}

void MPU6050::getAccelVector(VectorFloat *v)
{
  // for 8g 4096 LSb/g
  v->x = this->accel[0] / 4096.0f;
  v->y = this->accel[1] / 4096.0f;
  v->z = this->accel[2] / 4096.0f;
}

void MPU6050::getGyroData(float *gyroRate, float deltaT)
{
  // for gyro configuration 500 deg / s
  /*
         1
      --------- = 0.0000611   ///// 65.5 is the value for the gyro for 500°/sec configure and rotate 1°/sec and 250 is refresh rate
      65.5*250

      refresh rate  = 1 / deltaT
  */

  deltaT /= 1000000.0f;

  // gyroRate[0] = this->gyro[0] * deltaT / 65.5f;  // in dgrees
  // gyroRate[1] = this->gyro[1] * deltaT / 65.5f;  // in dgrees
  // gyroRate[2] = this->gyro[2] * deltaT / 65.5f;  // in dgrees
  gyroRate[0] = this->gyro[0] * deltaT * 0.000266462f; // in radiun
  gyroRate[1] = this->gyro[1] * deltaT * 0.000266462f; // in radiun
  gyroRate[2] = this->gyro[2] * deltaT * 0.000266462f; // in radiun
}