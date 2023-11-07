#include "MPU6050.h"

// [gx, gy, gz, ax, ay, az, tmp]
int16_t mpu6050_raw_data[7];

// Function to initialize the MPU6050 sensor.
bool mpu6050_init() {
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);  // Start communication with MPU6050.
    Wire.write(MPU6050_RA_PWR_MGMT_1);                // Point to power management register.
    Wire.write(0x00);                                 // Wake up MPU6050 from sleep.
    Wire.endTransmission();                           // End transmission.

    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);                                                      // Start another communication.
    Wire.write(MPU6050_RA_GYRO_CONFIG);                                                                   // Point to gyro configuration register.
    Wire.write(MPU6050_GYRO_FS_500 << (MPU6050_GCONFIG_FS_SEL_BIT - MPU6050_GCONFIG_FS_SEL_LENGTH + 1));  // Set gyro range to ±500°/s.
    Wire.endTransmission();                                                                               // End transmission.

    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);  // Start another communication.
    Wire.write(MPU6050_RA_ACCEL_CONFIG);              // Point to accelerometer configuration register.
    Wire.write(MPU6050_ACCEL_FS_8 << (MPU6050_ACONFIG_AFS_SEL_BIT - MPU6050_ACONFIG_AFS_SEL_LENGTH + 1));  // Set accelerometer range to ±8g.
    Wire.endTransmission();                                                                                // End transmission.

    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);                                                 // Start another communication.
    Wire.write(MPU6050_RA_CONFIG);                                                                   // Point to configuration register.
    Wire.write(MPU6050_DLPF_BW_42 << (MPU6050_CFG_DLPF_CFG_BIT - MPU6050_CFG_DLPF_CFG_LENGTH + 1));  // Set DLPF bandwidth to 42 Hz.
    Wire.endTransmission();                                                                          // End transmission.

    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);  // Start another communication for gyro verification.
    Wire.write(MPU6050_RA_GYRO_CONFIG);               // Point to gyro configuration register.
    Wire.endTransmission();                           // End transmission.
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);  // Request data from gyro configuration register.
    Wire.requestFrom(MPU6050_DEFAULT_ADDRESS, (size_t) 1);     // Request 1 byte of data.
    while (Wire.available() < 1)
        ;  // Wait for data.
    uint8_t data = Wire.read();  // Read the received data.
    Wire.endTransmission();      // End transmission.
    if (data != (MPU6050_GYRO_FS_500 << (MPU6050_GCONFIG_FS_SEL_BIT - MPU6050_GCONFIG_FS_SEL_LENGTH + 1)))
        return false;  // Verify gyro configuration.

    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);  // Start another communication for accelerometer verification.
    Wire.write(MPU6050_RA_ACCEL_CONFIG);              // Point to accelerometer configuration register.
    Wire.endTransmission();                           // End transmission.
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);  // Request data from accelerometer configuration register.
    Wire.requestFrom(MPU6050_DEFAULT_ADDRESS, (size_t) 1);     // Request 1 byte of data.
    while (Wire.available() < 1)                      // Wait for data.
        ;
    data = Wire.read();      // Read the received data.
    Wire.endTransmission();  // End transmission.
    if (data != (MPU6050_ACCEL_FS_8 << (MPU6050_ACONFIG_AFS_SEL_BIT - MPU6050_ACONFIG_AFS_SEL_LENGTH + 1)))
        return false;  // Verify accelerometer configuration.

    return true;  // Configuration successful, return true.
}

// Function to read raw data from the MPU6050's registers
void mpu6050_read_raw_data() {
    // Begin a transmission to the MPU6050 at its default I2C address
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    // Indicate to the MPU6050 that we want to start reading at the ACCEL_XOUT_H register (0x3B)
    Wire.write(MPU6050_RA_ACCEL_XOUT_H);
    // End the I2C transmission
    Wire.endTransmission();
    // Request 14 bytes from the current register set (this reads the accelerometer, temperature, and gyroscope data in one go)
    Wire.requestFrom(MPU6050_DEFAULT_ADDRESS, (size_t) 14);
    // Wait until 14 bytes are ready for reading
    while (Wire.available() < 14)
        ;
    // Read the high and low bytes for each sensor value, combine them, and store them in the mpu6050_raw_data array
    mpu6050_raw_data[3] = Wire.read() << 8 | Wire.read();  // Accelerometer X-axis data
    mpu6050_raw_data[4] = Wire.read() << 8 | Wire.read();  // Accelerometer Y-axis data
    mpu6050_raw_data[5] = Wire.read() << 8 | Wire.read();  // Accelerometer Z-axis data
    mpu6050_raw_data[6] = Wire.read() << 8 | Wire.read();  // Temperature data
    mpu6050_raw_data[0] = Wire.read() << 8 | Wire.read();  // Gyroscope X-axis data
    mpu6050_raw_data[1] = Wire.read() << 8 | Wire.read();  // Gyroscope Y-axis data
    mpu6050_raw_data[2] = Wire.read() << 8 | Wire.read();  // Gyroscope Z-axis data
}

// Function to retrieve the latest raw sensor data from the MPU6050
void mpu6050_get_raw_data(imu_raw_t *data) {
    // Transfer the read raw data into the provided data structure
    data->gx = mpu6050_raw_data[0];  // Gyroscope X-axis raw value
    data->gy = mpu6050_raw_data[1];  // Gyroscope Y-axis raw value
    data->gz = mpu6050_raw_data[2];  // Gyroscope Z-axis raw value
    data->ax = mpu6050_raw_data[3];  // Accelerometer X-axis raw value
    data->ay = mpu6050_raw_data[4];  // Accelerometer Y-axis raw value
    data->az = mpu6050_raw_data[5];  // Accelerometer Z-axis raw value
}
