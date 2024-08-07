#include <Arduino.h>

#include "common/axis.h"
#include "common/craft.h"
#include "common/filter.h"
#include "common/timer.h"
#include "fc/init.h"
#include "fc/states.h"
#include "flight/flight_control_system.h"
#include "flight/imu.h"
#include "flight/physics.h"
#include "flight/pid.h"
#include "io/fin.h"
#include "io/interrupts.h"
#include "io/propeller.h"
#include "sensors/prop_rpm_sensor.h"

imu_raw_t main_imu_raw_data;
imu_data_t main_imu_data;
matrix_3f_t main_accl_angle;
matrix_3f_t main__gyro_data;
unsigned long main_loop_time, main_pwm_loop_timer;
receiver_channel_data_t *main_receiver_channel_data;
float outer_loop_yaw = 0.0f;

matrix_3f_t *main_filtered_angle;
float *main_fin_angles;

uint8_t main_loop_counter = 0;
uint16_t main_prop_thrust = 1000;

void setup() {
    init_fc_init();

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
    filter_ekf_init(0.04);

#ifdef SERIAL_DATA
    Serial.begin(SERIAL_BAUDRATE);
#endif

#endif
    main_loop_time = micros();
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
    timer_start_time_count();
    if (Serial.available()) {
        char ch = Serial.read();
        switch (ch) {
            case 'p':
                pid_set_p(Serial.parseFloat());
                break;
            case 'i':
                pid_set_i(Serial.parseFloat());
                break;
            case 'd':
                pid_set_d(Serial.parseFloat());
                break;
            case 'P':
                pid_set_p_yaw(Serial.parseFloat());
                break;
            case 'D':
                pid_set_d_yaw(Serial.parseFloat());
                break;
        }

        while (Serial.available()) Serial.read();
    }

#endif

    receiver_update();
    propeller_rpm_sensor_update();
    // read imu data
    imu_read_data();
    imu_get_data(&main_imu_data);

    // calculate angle
    // 180 / PI = 57.295779513
    main_accl_angle.value[0] = degrees(atan2f(main_imu_data.ay, sqrtf(main_imu_data.ax * main_imu_data.ax + main_imu_data.az * main_imu_data.az)));
    main_accl_angle.value[1] = degrees(atan2f(-main_imu_data.ax, sqrtf(main_imu_data.ay * main_imu_data.ay + main_imu_data.az * main_imu_data.az)));
    main_accl_angle.value[2] = 0;

    main__gyro_data.value[0] = main_imu_data.gx;
    main__gyro_data.value[1] = main_imu_data.gy;
    main__gyro_data.value[2] = main_imu_data.gz;

    filter_ekf_update(&main__gyro_data, &main_accl_angle);

    main_filtered_angle = filter_ekf_get_state();  // getting the orientation of the craft

    // outer loop yaw control
    if ((main_receiver_channel_data->channel[3] < 1450 || main_receiver_channel_data->channel[3] > 1550) &&
        main_receiver_channel_data->channel[2] > 1100 && main_receiver_channel_data->channel[3] <= 2000 &&
        main_receiver_channel_data->channel[3] >= 1000) {
        // 150 degree / sec
        outer_loop_yaw += ((((float)main_receiver_channel_data->channel[3]) - 1500) * 150 / 500) * LOOP_TIME;

        // normalizing yaw angle
        if (outer_loop_yaw > 180.0) {
            outer_loop_yaw -= 360.0;
        } else if (outer_loop_yaw < -180.0) {
            outer_loop_yaw += 360.0;
        }
    }

    main_receiver_channel_data = receiver_get_channel_data();  // read receiver channel
    pid_set_target_angle((((int16_t)main_receiver_channel_data->channel[0]) - 1500) * 45 / 500,
                         (((int16_t)main_receiver_channel_data->channel[1]) - 1500) * 45 / 500, outer_loop_yaw);

    pid_update(main_filtered_angle);
    main_fin_angles = flight_control_system_get_force_to_fin_angle(pid_get_data());

    // calculations for thrust based on roll and pitch
    if (main_receiver_channel_data->channel[2] > 1050) {
        main_prop_thrust =
                (uint16_t)((main_receiver_channel_data->channel[2] - 1000) *
                                   physics_get_thrust_from_roll_pitch(main_filtered_angle->value[FD_ROLL], main_filtered_angle->value[FD_PITCH]) +
                           1000);
        if (main_prop_thrust < 1000)
            main_prop_thrust = 1000;
        else if (main_prop_thrust > 2000)
            main_prop_thrust = 2000;
    } else {
        main_prop_thrust = main_receiver_channel_data->channel[2];
    }

    main_pwm_loop_timer = micros();
    propeller_set_pin_high(main_prop_thrust, main_pwm_loop_timer);
    fin_set_angles(main_fin_angles[FIN1], main_fin_angles[FIN2], main_fin_angles[FIN3], main_fin_angles[FIN4], main_pwm_loop_timer);

    if (main_loop_counter % 10 == 0) {
        indicator_green_blink();
    }

    do {
        main_pwm_loop_timer = micros();
    } while (propeller_update(main_pwm_loop_timer) | fin_update(main_pwm_loop_timer));

    timer_end_time_count();

#ifdef SERIAL_DATA
    // Serial.print(main_imu_data.gx);
    // Serial.print(",");
    // Serial.print(main_imu_data.gy);
    // Serial.print(",");
    // Serial.print(main_imu_data.gz);
    // Serial.print(",");

    // Serial.print(main_imu_data.ax);
    // Serial.print(",");
    // Serial.print(main_imu_data.ay);
    // Serial.print(",");
    // Serial.print(main_imu_data.az);
    // Serial.print(",");

    Serial.print(main_filtered_angle->value[FD_ROLL]);
    Serial.print(",");
    Serial.print(main_filtered_angle->value[FD_PITCH]);
    Serial.print(",");
    Serial.print(main_filtered_angle->value[FD_YAW]);
    Serial.print(",");

    Serial.print(propeller_rpm_sensor_get_speed_rps());
    Serial.print(",");

    // Serial.print(main_fin_angles[FIN1]);
    // Serial.print(",");
    // Serial.print(main_fin_angles[FIN2]);
    // Serial.print(",");
    // Serial.print(main_fin_angles[FIN3]);
    // Serial.print(",");
    // Serial.print(main_fin_angles[FIN4]);
    // Serial.print(",");

    // Serial.print(outer_loop_yaw);
    // Serial.print(",");

    // time
    // Serial.print(get_time_count()); Serial.print(",");

    Serial.println("");
#endif

#endif

    // loop time
    while (micros() - main_loop_time < LOOP_TIME_MICROSECONDS);
    main_loop_time = micros();
    main_loop_counter++;
}
