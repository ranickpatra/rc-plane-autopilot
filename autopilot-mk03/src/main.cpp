#include <Arduino.h>

#include "fc/states.h"
#include "common/axis.h"
#include "common/craft.h"
#include "common/filter.h"
// #include "common/timer.h"
#include "fc/init.h"
#include "flight/imu.h"
#include "io/fin.h"
#include "io/interrupts.h"
#include "io/propeller.h"

imu_raw_t imu_raw_data;
imu_data_t imu_data;
float accl_angle[3];
unsigned long loop_time, pwm_loop_timer;

uint8_t loop_counter = 0;

void setup() {
    fc_init();

#ifdef CALIBRATION
    Serial.begin(SERIAL_BAUDRATE);
    indicator_blue_on();
    delay(5000);
    indicator_blue_off();
    imu_calibrate();  // calibrate the imu
#endif

#ifdef RC_RECEIVER_TEST
    Serial.begin(SERIAL_BAUDRATE);
#endif

#ifdef FIN_TEST
    Serial.begin(SERIAL_BAUDRATE);
#endif

#ifdef FLIGHT
    ekf_init(0.96);

#endif
    loop_time = micros();
}

void loop() {
#ifdef CALIBRATION
    imu_calibration_t *calibration_data = imu_get_calibration_data();
    Serial.print(calibration_data->gx);
    Serial.print(",");
    Serial.print(calibration_data->gy);
    Serial.print(",");
    Serial.print(calibration_data->gz);
    Serial.print(" ");
    Serial.println("");
    indicator_green_blink();
    delay(50);
#endif

#ifdef RC_RECEIVER_TEST
    uint16_t *channel_data = receiver_get_channel_data();
    Serial.print(channel_data[0]);
    Serial.print(",");
    Serial.print(channel_data[1]);
    Serial.print(",");
    Serial.print(channel_data[2]);
    Serial.print(",");
    Serial.print(channel_data[3]);
    Serial.print(" ");
    Serial.println("");
    indicator_green_blink();

    delay(50);
#endif

#ifdef FIN_TEST

    uint16_t pulse_time = 1000;

    switch ((millis() / 10000) % 4) {
        case 0:
            pulse_time = 1000;
            break;
        case 1:
            pulse_time = 2000;
            break;
        case 2:
        case 3:
            pulse_time = 1500;
            break;
    }

    pwm_loop_timer = micros();
    set_fin_pins_high(pulse_time, pulse_time, pulse_time, pulse_time, pwm_loop_timer);

    do {
        pwm_loop_timer = micros();
    } while (update_fins(pwm_loop_timer));

#endif

#ifdef PROPELLER_TEST
    uint16_t pulse_time = 1000;

#ifdef PROPELLER_ESC_CALIBRATION
    // calibration of esc
    unsigned long current_time = millis();
    if (current_time < 20000) {
        if (current_time < 8000)
            pulse_time = 2000;
        else
            pulse_time = 1000;
    } else {
#endif

        // testing of esc
        if (loop_counter % 10 == 0) {
            indicator_green_blink();
        }

        switch ((millis() / 8000) % 4) {
            case 0:
                pulse_time = 1000;
                break;
            case 1:
                pulse_time = 1100;
                break;
            case 2:
                pulse_time = 1500;
                break;
            case 3:
                pulse_time = 2000;
                break;
        }

#ifdef PROPELLER_ESC_CALIBRATION
    }
#endif
    pwm_loop_timer = micros();
    set_propeller_pin_high(pulse_time, pwm_loop_timer);

    do {
        pwm_loop_timer = micros();
    } while (update_propeller(pwm_loop_timer));
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


    ekf_update(gyro_data, accl_angle);

    matrix_3f_t *state = ekf_get_state();
    // Serial.print(state->value[0]); Serial.print(",");
    // Serial.print(state->value[1]); Serial.print(",");
    // Serial.print(state->value[2]); Serial.print(",");
    // Serial.println("");
#endif

    // loop time
    while (micros() - loop_time < LOOP_TIME_MICROSECONDS)
        ;
    loop_time = micros();
    loop_counter++;
}

// STM32F103CBT6 -> CC3D
