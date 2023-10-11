#include "imu.h"
#include "common/filter.h"

imu_raw_t raw_data = {0, 0, 0, 0, 0, 0};
imu_calibration_t calibration_data = {0, 0, 0, 0, 0, 0};

// initilize imu
boolean imu_init()
{
  // set up the MPU6050
  Wire.beginTransmission(0x68); // start communicating with mpu6050
  Wire.write(0x6B);             // we want to write in 0x6B register which is powermanagement register
  Wire.write(0x00);             // write 0x00 to that register
  Wire.endTransmission();       // end the transmission

  Wire.beginTransmission(0x68); // start communicating with mpu6050
  Wire.write(0x1B);             // we want to write in 0x1B which is gyro configaration register
  Wire.write(0x08);             // set the gyro for 500 deg pre sec full scale
  Wire.endTransmission();       // end the transmission

  Wire.beginTransmission(0x68); // start communicating with mpu6050
  Wire.write(0x1C);             // we want to write in 0x1C register which is Accelerometer Configuration register
  Wire.write(0x10);             // set the accelerometer with +-8g
  Wire.endTransmission();       // end the transmission

  Wire.beginTransmission(0x68); // start communicating with mpu6050
  Wire.write(0x1A);             // we want to write in 0x1A which is used to gyro and accelerometer synchronizing
  Wire.write(0x03);             // write 0x03 in that register
  Wire.endTransmission();       // end the transmission

  // random check registers wheather the values is written correctly or not
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.requestFrom(0x68, 1);
  while (Wire.available() < 1)
    ;
  byte data = Wire.read();
  Wire.endTransmission();
  if (data != 0x08)
    return false;
  
  // random check registers wheather the values is written correctly or not
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.requestFrom(0x68, 1);
  while (Wire.available() < 1)
    ;
  data = Wire.read();
  Wire.endTransmission();
  if (data != 0x10)
    return false;

  return true;
}

// read raw data from imu
void imu_read_data()
{
  Wire.beginTransmission(0x68); // start communicating with MPU6050
  Wire.write(0x3B);             // start reading from 0x3B
  Wire.endTransmission();       // end the transmission
  Wire.requestFrom(0x68, 14);   // request 14 bytes
  while (Wire.available() < 14)
    ;                                           // wait until 14 bytes are received
  raw_data.ax = Wire.read() << 8 | Wire.read(); // read acc x data
  raw_data.ay = Wire.read() << 8 | Wire.read(); // read acc y data
  raw_data.az = Wire.read() << 8 | Wire.read(); // read acc z data
  Wire.read(); Wire.read();               // read temperature data [we don't use it here]
  raw_data.gx = Wire.read() << 8 | Wire.read(); // read gyro x data
  raw_data.gy = Wire.read() << 8 | Wire.read(); // read gyro y data
  raw_data.gz = Wire.read() << 8 | Wire.read(); // read gyro z data

  // calibration
  raw_data.gx -= calibration_data.gx;
  raw_data.gy -= calibration_data.gy;
  raw_data.gz -= calibration_data.gz;
  raw_data.ax -= calibration_data.ax;
  raw_data.ay -= calibration_data.ay;
  raw_data.az -= calibration_data.az;
}

// calibrate imu
void imu_calibrate()
{
  long tmp_imu_calibration_data[3] = {0, 0, 0};
  unsigned long int loop_time = micros() + LOOP_TIME_MICROSECONDS;
  // calibrate
  for (uint16_t i = 0; i < IMU_CALIBRATION_STEPS; i++)
  {
    imu_read_data();
    tmp_imu_calibration_data[0] += (long)raw_data.gx;
    tmp_imu_calibration_data[1] += (long)raw_data.gy;
    tmp_imu_calibration_data[2] += (long)raw_data.gz;

    // blink indicator 
    if (i % 50 == 0)
      indicator_blink();

    // wait for loop time 
    while (loop_time > micros());
    loop_time = micros() + LOOP_TIME_MICROSECONDS;
  }

  indicator_off();  // turn off the indicator

  // average the calibration data
  calibration_data.gx = (int)(tmp_imu_calibration_data[0] / IMU_CALIBRATION_STEPS);
  calibration_data.gy = (int)(tmp_imu_calibration_data[1] / IMU_CALIBRATION_STEPS);
  calibration_data.gz = (int)(tmp_imu_calibration_data[2] / IMU_CALIBRATION_STEPS);



  // TODO remove
  ExtendedKalmanFilter ekf = ExtendedKalmanFilter(3, 0.98);
  float gyro[3];
  float acc[3];
  ekf.update(gyro, acc);
}

// imu get raw data
void imu_get_raw_data(imu_raw_t *data)
{
  data->gx = raw_data.gx;
  data->gy = raw_data.gy;
  data->gz = raw_data.gz;
  data->ax = raw_data.ax;
  data->ay = raw_data.ay;
  data->az = raw_data.az;
}