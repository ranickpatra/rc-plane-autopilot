#include <Arduino.h>

#include "common/axis.h"
#include "common/filter.h"
#include "flight/imu.h"
#include "io/indiactor.h"

imu_raw_t imu_raw_data;
imu_data_t imu_data;
float accl_angle[3];
unsigned long int current_time;

uint8_t arr1[3][3] = {
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9},
};


void setup() {
  // init indicator
  indicator_init();

  // stetup IMU
  if (!imu_init()) {
    while (true) {
      indicator_blink();
      delay(50);
    }
  }

  imu_calibrate();  // calibrate the imu

  ekf_init(0.96);

  Serial.begin(115200);

  // while(true);

  current_time = micros() + LOOP_TIME;
}

void loop() {
  // read imu data
  imu_read_data();

  // imu_get_raw_data(&imu_raw_data);
  // Serial.print(imu_raw_data.gx); Serial.print(",");
  // Serial.print(imu_raw_data.gy); Serial.print(",");
  // Serial.print(imu_raw_data.gz); Serial.print(",");
  // Serial.print(imu_raw_data.ax); Serial.print(",");
  // Serial.print(imu_raw_data.ay); Serial.print(",");
  // Serial.print(imu_raw_data.az); //Serial.print(",");
  // Serial.println("");


  imu_get_data(&imu_data);
  // Serial.print(imu_data.gx);
  // Serial.print(",");
  // Serial.print(imu_data.gy);
  // Serial.print(",");
  // Serial.print(imu_data.gz);
  // Serial.print(",");
  // Serial.print(imu_data.ax);
  // Serial.print(",");
  // Serial.print(imu_data.ay);
  // Serial.print(",");
  // Serial.print(imu_data.az); // Serial.print(",");
  // Serial.println("");

  // calculate angle
  accl_angle[0] = atan2f(imu_data.ay, sqrtf(imu_data.ax * imu_data.ax + imu_data.az * imu_data.az)) * 180 / M_PI;
  accl_angle[1] = atan2f(imu_data.ax, sqrtf(imu_data.ay * imu_data.ay + imu_data.az * imu_data.az)) * 180 / M_PI;
  accl_angle[2] = 0;

  float gyro_data[3];
  gyro_data[0] = imu_data.gx;
  gyro_data[1] = imu_data.gy;
  gyro_data[2] = imu_data.gz;


  ekf_update(gyro_data, accl_angle);

  // matrix_3f_t *state = ekf_get_state();

  // Serial.print(state->value[0]); Serial.print(",");
  // Serial.print(state->value[1]); Serial.print(",");
  // Serial.print(state->value[2]); Serial.print(",");
  Serial.println(get_time_diff());
  // Serial.println("");

  // loop time
  while (current_time > micros())
    ;
  current_time = micros() + LOOP_TIME;
}
