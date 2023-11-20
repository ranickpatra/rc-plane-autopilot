#include <Arduino.h>

#include "common/axis.h"
#include "common/craft.h"
#include "common/filter.h"
#include "common/timer.h"
#include "fc/init.h"
#include "fc/states.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "io/fin.h"
#include "io/interrupts.h"
#include "io/propeller.h"

imu_raw_t imu_raw_data;
imu_data_t imu_data;
matrix_3f_t accl_angle;
matrix_3f_t gyro_data;
unsigned long loop_time, pwm_loop_timer;
receiver_channel_data_t* receiver_channel_data;

matrix_3f_t *filtered_angle;
pid_data_t *pid_fin_data;

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
    ekf_init(0.04);

#ifdef SERIAL_DATA
    Serial.begin(SERIAL_BAUDRATE);
#endif

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
    int16_t fin_angle = 0;
    switch ((millis() / 10000) % 4) {
        case 0:
            fin_angle = -45;
            break;
        case 1:
            fin_angle = 45;
            break;
        case 2:
        case 3:
            fin_angle = 0;
            break;
    }

    pwm_loop_timer = micros();
    set_fin_angles(fin_angle, fin_angle, fin_angle, fin_angle, pwm_loop_timer);

    do {
        pwm_loop_timer = micros();
    } while (update_fins(pwm_loop_timer));

#endif

#ifdef PROPELLER_TESTj
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

        if (loop_counter % 10 == 0) {
            indicator_green_blink();
        }

        // testing of esc
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

#ifdef SERIAL_DATA
    start_time_count();
    if (Serial.available()) {
        char ch = Serial.read();
        switch (ch)
        {
        case 'p':
            pid_set_p(Serial.parseFloat());
            break;
        case 'd':
            pid_set_d(Serial.parseFloat());
            break;
        // case 'a':
        //     pid_set_angle(Serial.parseFloat());
        //     break;
        }

        while (Serial.available())
            Serial.read();
        
    }
    
#endif

    receiver_update();
    // read imu data
    imu_read_data();
    imu_get_data(&imu_data);

    // calculate angle
    // 180 / PI = 57.295779513
    accl_angle.value[0] = degrees(atan2f(imu_data.ay, sqrtf(imu_data.ax * imu_data.ax + imu_data.az * imu_data.az)));
    accl_angle.value[1] = degrees(atan2f(-imu_data.ax, sqrtf(imu_data.ay * imu_data.ay + imu_data.az * imu_data.az)));
    accl_angle.value[2] = 0;

    gyro_data.value[0] = imu_data.gx;
    gyro_data.value[1] = imu_data.gy;
    gyro_data.value[2] = imu_data.gz;

    ekf_update(&gyro_data, &accl_angle);

    filtered_angle = ekf_get_state();  // getting the orientation of the craft

    receiver_channel_data = receiver_get_channel_data();    // read receiver channel
    set_target_angle((((int16_t)receiver_channel_data->channel[0]) - 1500)*45/500,
    (((int16_t)receiver_channel_data->channel[1]) - 1500)*45/500,
    0);

    pid_update(filtered_angle);

    pid_fin_data = pid_get_data();

    pwm_loop_timer = micros();
    set_propeller_pin_high(receiver_channel_data->channel[2], pwm_loop_timer);
    set_fin_angles(pid_fin_data[AXIS_X].sum, pid_fin_data[AXIS_Y].sum, pid_fin_data[AXIS_X].sum, pid_fin_data[AXIS_Y].sum, pwm_loop_timer);

    if (loop_counter % 10 == 0) {
        indicator_green_blink();
    }

    do {
        pwm_loop_timer = micros();
    } while (update_propeller(pwm_loop_timer) | update_fins(pwm_loop_timer));

    end_time_count();

#ifdef SERIAL_DATA
    // Serial.print(gyro_data.value[0]); Serial.print(",");
    // Serial.print(gyro_data.value[1]); Serial.print(",");
    // Serial.print(gyro_data.value[2]); Serial.print(",");

    // Serial.print(imu_data.ax); Serial.print(",");
    // Serial.print(imu_data.ay); Serial.print(",");
    // Serial.print(imu_data.az); Serial.print(",");

    Serial.print(filtered_angle->value[0] / 90.0); Serial.print(",");
    // Serial.print(filtered_angle->value[0]); Serial.print(",");
    // Serial.print(filtered_angle->value[1]); Serial.print(",");
    // Serial.print(angle->value[2]); Serial.print(",");

    Serial.print(pid_fin_data[0].sum / 30.0); Serial.print(",");
    // Serial.print(pid_fin_data[1].sum); Serial.print(",");
    // Serial.print(pid_fin_data[2].sum); Serial.print(",");

    // pid_coefficient_t* pppp = pid_get();
    // Serial.print(pppp[0].Kp); Serial.print(", ");
    // Serial.print(pppp[0].Ki); Serial.print(", ");
    // Serial.print(pppp[0].Kd);

    

    Serial.print(get_propeller_input()); Serial.print(",");

    // time
    // Serial.print(get_time_count()); Serial.print(",");

    Serial.println("");
#endif


#endif

    // loop time
    while (micros() - loop_time < LOOP_TIME_MICROSECONDS)
        ;
    loop_time = micros();
    loop_counter++;
}

// STM32F103CBT6 -> CC3D
