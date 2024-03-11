#pragma once

#define LOOP_TIME_MICROSECONDS 10000  // loop time in micro seconds
#define LOOP_TIME 0.01                // loop time in seconds
#define LOOP_TIME_INVERSE 100        // inverse of loop time 1/LOOP_TIME

// #define LOOP_TIME_MICROSECONDS 20000  // loop time in micro seconds
// #define LOOP_TIME 0.02                // loop time in seconds
// #define LOOP_TIME_INVERSE 50        // inverse of loop time 1/LOOP_TIME


// Indicators
#define INDICATOR_DDR DDRC
#define INDICATOR_PORT PORTC
#define INDICATOR_PIN_BLUE PC0
#define INDICATOR_PIN_GREEN PC1
#define INDICATOR_PIN_RED PC2

// servos and motor
#define ACTUATOR_DDR DDRB
#define ACTUATOR_PORT PORTB
#define SERVO_PIN_1 PB2
#define SERVO_PIN_2 PB3
#define SERVO_PIN_3 PB0
#define SERVO_PIN_4 PB1
#define PROPELLER_PIN PB4

// fins
#define ACTUATOR_PULSE_MAX 2000
#define ACTUATOR_PULSE_MIN 1000
#define FIN_1_DIRECTION_REV
// #define FIN_2_DIRECTION_REV
// #define FIN_3_DIRECTION_REV
// #define FIN_4_DIRECTION_REV
#define FIN_MAX_ANGLE 30.0
#define FIN_MIN_ANGLE -30.0

// receiver
#define RECEIVER_CHANNEL_COUNT 4
#define RECEIVER_DDR DDRD
#define RECEIVER_PORT PIND
#define RECEIVER_CHANNEL_1 PD2
#define RECEIVER_CHANNEL_2 PD3
#define RECEIVER_CHANNEL_3 PD4
#define RECEIVER_CHANNEL_4 PD5

#define RECEIVER_PCIE PCIE2
#define RECEIVER_PCMSK PCMSK2
#define RECEIVER_PCINT_CHANNEL_1 PCINT18
#define RECEIVER_PCINT_CHANNEL_2 PCINT19
#define RECEIVER_PCINT_CHANNEL_3 PCINT20
#define RECEIVER_PCINT_CHANNEL_4 PCINT21

// RPM sensor
#define PROP_BLADE_COUNT 3
#define PROP_RPM_SENSOR_DDR DDRD
#define PROP_RPM_SENSOR_PORT PIND
#define PROP_RPM_SENSOR_RPM_SENSOR_PIN PD6
#define PROP_RPM_SENSOR_PCIE PCIE2
#define PROP_RPM_SENSOR_PCMSK PCMSK2
#define PROP_RPM_SENSOR_PCINT PCINT22

// IMU
#define IMU_CALIBRATION_STEPS 2000  // steps to calibrate the IMU
#define IMU_CALIBRATION_DATA { -80, 78, -12, 0, 0, 0 }










enum fin_index_index_e {
  FIN_1=0,
  FIN_2,
  FIN_3,
  FIN_4
};

#define FIN_COUNT 4