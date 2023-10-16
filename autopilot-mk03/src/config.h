#pragma once

#define TEST_C

#ifndef TEST_C
  #define LOOP_TIME_MICROSECONDS 4000  // loop time in micro seconds
  #define LOOP_TIME 0.004  // loop time in seconds
#else
  #define LOOP_TIME_MICROSECONDS 10000  // loop time in micro seconds
  #define LOOP_TIME 0.01  // loop time in seconds
#endif


#define IMU_CALIBRATION_STEPS 2000  // steps to calibrate the IMU
