#include <math.h>
#include "imu.h"
#include "common/filter.h"
#include "sensors/MPU6050.h"

imu_raw_t raw_data = {0, 0, 0, 0, 0, 0};
imu_calibration_t calibration_data = {0, 0, 0, 0, 0, 0};

// Function to initialize the IMU sensor
boolean imu_init()
{
  // Call the initialization function for the MPU6050
  // Returns true if initialization was successful, false otherwise
  return mpu6050_init();
}

// Function to read and calibrate raw data from the IMU sensor
void imu_read_data()
{
  // Read raw sensor data from the MPU6050
  mpu6050_read_raw_data();
  // Store the raw sensor data in the 'raw_data' structure
  mpu6050_get_raw_data(&raw_data);

  // Calibrate the raw data by subtracting out the calibration values
  // This is necessary to account for sensor drift or bias
  raw_data.gx -= calibration_data.gx; // Calibrate gyroscope X-axis data
  raw_data.gy -= calibration_data.gy; // Calibrate gyroscope Y-axis data
  raw_data.gz -= calibration_data.gz; // Calibrate gyroscope Z-axis data
  raw_data.ax -= calibration_data.ax; // Calibrate accelerometer X-axis data
  raw_data.ay -= calibration_data.ay; // Calibrate accelerometer Y-axis data
  raw_data.az -= calibration_data.az; // Calibrate accelerometer Z-axis data
}

// Function to calibrate the IMU sensor
void imu_calibrate()
{
  // Temporary storage for the accumulated sensor readings during calibration
  long tmp_imu_calibration_data[3] = {0, 0, 0};
  // Record the next expected time to read sensor data based on loop time
  unsigned long int loop_time = micros() + LOOP_TIME_MICROSECONDS;

  // Calibrate by reading the sensor multiple times
  for (uint16_t i = 0; i < IMU_CALIBRATION_STEPS; i++)
  {
    // Read current sensor data
    imu_read_data();
    // Accumulate the gyroscope readings
    tmp_imu_calibration_data[0] += (long)raw_data.gx;
    tmp_imu_calibration_data[1] += (long)raw_data.gy;
    tmp_imu_calibration_data[2] += (long)raw_data.gz;

    // Blink an indicator LED every 50 readings to show that calibration is in progress
    if (i % 50 == 0)
      indicator_blink();

    // Wait until it's time for the next reading
    while (loop_time > micros())
      ;
    // Update the next expected time for a sensor read
    loop_time = micros() + LOOP_TIME_MICROSECONDS;
  }

  // Turn off the indicator LED to show that calibration is complete
  indicator_off();

  // Average the accumulated readings to get the calibration data
  // This will give the bias values for the sensor
  calibration_data.gx = (int16_t)(tmp_imu_calibration_data[0] / IMU_CALIBRATION_STEPS);
  calibration_data.gy = (int16_t)(tmp_imu_calibration_data[1] / IMU_CALIBRATION_STEPS);
  calibration_data.gz = (int16_t)(tmp_imu_calibration_data[2] / IMU_CALIBRATION_STEPS);
}

void imu_get_raw_data(imu_raw_t *data)
{
  data->gx = raw_data.gx;
  data->gy = raw_data.gy;
  data->gz = raw_data.gz;
  data->ax = raw_data.ax;
  data->ay = raw_data.ay;
  data->az = raw_data.az;
}

void imu_get_data(imu_data_t *data)
{
  // 500deg/sec config output will be 65.5/deg/s
  // 1/65.5 = 0.015267176
  data->gx = ((float)raw_data.gx) * 0.015267176;
  data->gy = ((float)raw_data.gy) * 0.015267176;
  data->gz = ((float)raw_data.gz) * 0.015267176;

  // 8g configuration output will be 4096/g
  // i.e. 4096 for 9.81 m/s
  // i.e. for 1 m/s value will be 417.53313
  // 1/417.53313 = 0.00239502
  data->ax = ((float)raw_data.ax) * 0.00239502;
  data->ay = ((float)raw_data.ay) * 0.00239502;
  data->az = ((float)raw_data.az) * 0.00239502;
}