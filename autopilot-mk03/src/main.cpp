#include <Arduino.h>

#include "common/axis.h"
#include "common/craft.h"
#include "common/filter.h"
#include "common/timer.h"
#include "fc/init.h"
#include "flight/imu.h"
#include "io/interrupts.h"
// #include "io/indiactor.h"

imu_raw_t imu_raw_data;
imu_data_t imu_data;
float accl_angle[3];
unsigned long loop_time;

void setup() {
    fc_init();
    // stetup IMU
    if (!imu_init()) {
        while (true) {
            indicator_red_blink();
            delay(50);
        }
    }

#ifdef CALIBRATION
    Serial.begin(SERIAL_BAUDRATE);
    indicator_blue_on();
    delay(1000);
    indicator_blue_off();
    imu_calibrate();  // calibrate the imu
#endif

#ifdef FLIGHT
    // ekf_init(0.96);

    loop_time = micros() + LOOP_TIME;
#endif
}

void loop() {
#ifdef CALIBRATION
    imu_calibration_t *calibration_data = imu_get_calibration_data();
    Serial.print(calibration_data->gx); Serial.print(",");
    Serial.print(calibration_data->gy); Serial.print(",");
    Serial.print(calibration_data->gz); Serial.print(" ");
    Serial.println("");
    indicator_green_blink();
    delay(50);
#endif

#ifdef FLIGHT
    // read imu data
    imu_read_data();
    imu_get_data(&imu_data);

    // calculate angle
    // 180 / PI = 57.295779513
    // accl_angle[0] = atan2f(imu_data.ay, sqrtf(imu_data.ax * imu_data.ax + imu_data.az * imu_data.az)) * RAD_2_DEGREE;
    // accl_angle[1] = atan2f(imu_data.ax, sqrtf(imu_data.ay * imu_data.ay + imu_data.az * imu_data.az)) * RAD_2_DEGREE;
    accl_angle[0] = degrees(atan2f(imu_data.ay, sqrtf(imu_data.ax * imu_data.ax + imu_data.az * imu_data.az)));
    accl_angle[1] = degrees(atan2f(imu_data.ax, sqrtf(imu_data.ay * imu_data.ay + imu_data.az * imu_data.az)));
    accl_angle[2] = 0;

    float gyro_data[3];
    gyro_data[0] = imu_data.gx;
    gyro_data[1] = imu_data.gy;
    gyro_data[2] = imu_data.gz;

    start_time_count();

    ekf_update(gyro_data, accl_angle);

    end_time_count();

    matrix_3f_t *state = ekf_get_state();
    // Serial.print(state->value[0]); Serial.print(",");
    // Serial.print(state->value[1]); Serial.print(",");
    // Serial.print(state->value[2]); Serial.print(",");
    Serial.println(get_time_count());
    // Serial.println("");

    // loop time
    while (loop_time > micros())
        ;
    loop_time = micros() + LOOP_TIME;
#endif
}

// STM32F103CBT6 -> CC3D