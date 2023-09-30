#include <Arduino.h>
#include "flight/imu.h"
#include "io/indiactor.h"

imu_raw_t imu_raw_data;
unsigned long int current_time;

void setup()
{
  // init indicator
  indicator_init();

  // stetup IMU
  if (!imu_init())
  {
    while (true)
    {
      indicator_blink();
      delay(100);
    }
  }

  imu_calibrate(); // calibrate the imu

  Serial.begin(115200);
  current_time = micros() + LOOP_TIME;

}

void loop()
{
  // read imu data
  imu_read_data();
  imu_get_raw_data(&imu_raw_data);

  Serial.print(imu_raw_data.gx); Serial.print(",");
  Serial.print(imu_raw_data.gy); Serial.print(",");
  Serial.print(imu_raw_data.gz); Serial.print(",");
  Serial.print(imu_raw_data.ax); Serial.print(",");
  Serial.print(imu_raw_data.ay); Serial.print(",");
  Serial.print(imu_raw_data.az); //Serial.print(",");
  Serial.println("");

  // calculate angle


  // loop time
  while (current_time > micros());
  current_time = micros() + LOOP_TIME;
}
