#include "imu.h"

#include <math.h>

#include "common/craft.h"
#include "common/filter.h"
#include "sensors/MPU6050.h"

imu_raw_t imu_raw_data = {0, 0, 0, 0, 0, 0}; // Declare global variable to store raw IMU data
imu_calibration_t imu_calibration_data = IMU_CALIBRATION_DATA; // Declare global variable to store calibration data

// Function to initialize the IMU sensor
boolean imu_init() {
    // Call the initialization function for the MPU6050
    // Returns true if initialization was successful, false otherwise
    return mpu6050_init();
}

// Function to read and calibrate raw data from the IMU sensor
void imu_read_data() {
    // Read raw sensor data from the MPU6050
    mpu6050_read_raw_data();
    // Store the raw sensor data in the 'raw_data' structure
    mpu6050_get_raw_data(&imu_raw_data);

    // Calibrate the raw data by subtracting out the calibration values
    // This is necessary to account for sensor drift or bias
    imu_raw_data.gx -= imu_calibration_data.gx;  // Calibrate gyroscope X-axis data
    imu_raw_data.gy -= imu_calibration_data.gy;  // Calibrate gyroscope Y-axis data
    imu_raw_data.gz -= imu_calibration_data.gz;  // Calibrate gyroscope Z-axis data
    imu_raw_data.ax -= imu_calibration_data.ax;  // Calibrate accelerometer X-axis data
    imu_raw_data.ay -= imu_calibration_data.ay;  // Calibrate accelerometer Y-axis data
    imu_raw_data.az -= imu_calibration_data.az;  // Calibrate accelerometer Z-axis data
}

// Function to calibrate the IMU sensor
void imu_calibrate() {
    // Temporary storage for the accumulated sensor readings during calibration
    long tmp_imu_calibration_data[3] = {0, 0, 0};
    // Record the next expected time to read sensor data based on loop time
    unsigned long int loop_time = micros() + LOOP_TIME_MICROSECONDS;

    // Calibrate by reading the sensor multiple times
    for (uint16_t i = 0; i < IMU_CALIBRATION_STEPS; i++) {
        // Read current sensor data
        imu_read_data();
        // Accumulate the gyroscope readings
        tmp_imu_calibration_data[0] += (long)imu_raw_data.gx;
        tmp_imu_calibration_data[1] += (long)imu_raw_data.gy;
        tmp_imu_calibration_data[2] += (long)imu_raw_data.gz;

        // Blink an indicator LED every 50 readings to show that calibration is in progress
        if (i % 50 == 0) indicator_red_blink();

        // Wait until it's time for the next reading
        while (loop_time > micros())
            ;
        // Update the next expected time for a sensor read
        loop_time = micros() + LOOP_TIME_MICROSECONDS;
    }

    // Turn off the indicator LED to show that calibration is complete
    indicator_red_off();

    // Average the accumulated readings to get the calibration data
    // This will give the bias values for the sensor
    imu_calibration_data.gx = (int16_t)(tmp_imu_calibration_data[0] / IMU_CALIBRATION_STEPS);
    imu_calibration_data.gy = (int16_t)(tmp_imu_calibration_data[1] / IMU_CALIBRATION_STEPS);
    imu_calibration_data.gz = (int16_t)(tmp_imu_calibration_data[2] / IMU_CALIBRATION_STEPS);
}

// Function to get raw data from the IMU sensor
void imu_get_raw_data(imu_raw_t *data) {
    // Copy raw data from global variable to provided data structure
    data->gx = imu_raw_data.gx;
    data->gy = imu_raw_data.gy;
    data->gz = imu_raw_data.gz;
    data->ax = imu_raw_data.ax;
    data->ay = imu_raw_data.ay;
    data->az = imu_raw_data.az;
}

// Function to get calibrated data from the IMU sensor
void imu_get_data(imu_data_t *data) {
    // Convert raw data to calibrated data using predefined conversion factors
    // 500deg/sec config output will be 65.5/deg/s
    // 1/65.5 = 0.015267176
    data->gx = ((float)imu_raw_data.gx) * 0.015267176;
    data->gy = ((float)imu_raw_data.gy) * 0.015267176;
    data->gz = ((float)imu_raw_data.gz) * 0.015267176;

    // 8g configuration output will be 4096/g
    // i.e. 4096 for 9.81 m/s
    // i.e. for 1 m/s value will be 417.53313
    // 1/417.53313 = 0.00239502
    data->ax = ((float)imu_raw_data.ax) * 0.00239502;
    data->ay = ((float)imu_raw_data.ay) * 0.00239502;
    data->az = ((float)imu_raw_data.az) * 0.00239502;
}

// Function to retrieve calibration data from the IMU sensor
imu_calibration_t *imu_get_calibration_data() {
    // Return a pointer to the calibration data
    return &imu_calibration_data;
}
