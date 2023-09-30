#include <Arduino.h>
#include <Wire.h>
#include "io/indiactor.h"
#include "config.h"

#ifndef FC_IMU_H
#define FC_IMU_H

struct imu_raw_t
{
  int gx;
  int gy;
  int gz;
  int ax;
  int ay;
  int az;
};


struct imu_calibration_t
{
  int gx;
  int gy;
  int gz;
  int ax;
  int ay;
  int az;
};



// initilize imu
bool imu_init();
// read imu
void imu_read_data();
// calibrate imu
void imu_calibrate();
// imu get raw data
void imu_get_raw_data(imu_raw_t* data);



#endif