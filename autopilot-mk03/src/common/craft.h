#pragma once

#define LOOP_TIME_MICROSECONDS 10000  // loop time in micro seconds
#define LOOP_TIME 0.01                // loop time in seconds


// Indicators
#define INDICATOR_DDR DDRC
#define INDICATOR_PORT PORTC
#define INDICATOR_PIN_BLUE PC0
#define INDICATOR_PIN_GREEN PC1
#define INDICATOR_PIN_RED PC2

// servos and motor
#define ACTUATOR_DDR DDRB
#define ACTUATOR_PORT PORTB
#define SERVO_PIN_1 PB0
#define SERVO_PIN_2 PB1
#define SERVO_PIN_3 PB2
#define SERVO_PIN_4 PB3
#define PROPELLER_PIN PB4

// fins
#define FIN_PULSE_MAX 2000
#define FIN_PULSE_MIN 1000
// #define FIN_1_DIRECTION_REV
// #define FIN_2_DIRECTION_REV
#define FIN_3_DIRECTION_REV
// #define FIN_4_DIRECTION_REV

// receiver
#define RECEIVER_CHANNEL_COUNT 4
#define RECEIVER_DDR DDRD
#define RECEIVER_PIN PIND
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

// IMU
#define IMU_CALIBRATION_STEPS 2000  // steps to calibrate the IMU
#define IMU_CALIBRATION_DATA { -80, 78, -12, 0, 0, 0 }