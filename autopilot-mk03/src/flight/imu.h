#pragma once

// #include <Arduino.h>
#include <Wire.h>
#include "io/indiactor.h"
#include "common/craft.h"

// calibration data
// -87,79,-11

// -82,79,-11 
// -81,78,-12 
// -80,78,-12 
// -79,79,-12
// -78,78,-12 

// -86,81,-11

struct imu_raw_t
{
  int16_t gx;
  int16_t gy;
  int16_t gz;
  int16_t ax;
  int16_t ay;
  int16_t az;
};

struct imu_data_t
{
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
};


struct imu_calibration_t
{
  int16_t gx;
  int16_t gy;
  int16_t gz;
  int16_t ax;
  int16_t ay;
  int16_t az;
};



bool imu_init();
void imu_read_data();
void imu_calibrate();
void imu_get_raw_data(imu_raw_t *data);
void imu_get_data(imu_data_t *data);
imu_calibration_t *imu_get_calibration_data();