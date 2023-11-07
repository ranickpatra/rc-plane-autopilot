#include "fin.h"

#include "common/craft.h"

// to reverse 2000-value+1000
// i.e 3000-value
#define FIN_REV_OFFSET (FIN_PULSE_MAX + FIN_PULSE_MIN)

#define SERVO_PIN_1_BIT (1 << SERVO_PIN_1)
#define SERVO_PIN_2_BIT (1 << SERVO_PIN_2)
#define SERVO_PIN_3_BIT (1 << SERVO_PIN_3)
#define SERVO_PIN_4_BIT (1 << SERVO_PIN_4)

#define SERVO_PINS_ALL (SERVO_PIN_1_BIT | SERVO_PIN_2_BIT | SERVO_PIN_3_BIT | SERVO_PIN_4_BIT)

unsigned long fin_1_time = 0, fin_2_time = 0, fin_3_time = 0, fin_4_time = 0;

void fin_init() {
    ACTUATOR_DDR |= SERVO_PINS_ALL;
    ACTUATOR_PORT &= ~SERVO_PINS_ALL;
}

void set_fin_pins_high(uint16_t f1_time, uint16_t f2_time, uint16_t f3_time, uint16_t f4_time, unsigned long pulse_start_time) {
    ACTUATOR_PORT |= SERVO_PINS_ALL;

#ifndef FIN_1_DIRECTION_REV
    fin_1_time = pulse_start_time + (f1_time > FIN_PULSE_MAX ? FIN_PULSE_MAX : (f1_time < FIN_PULSE_MIN ? FIN_PULSE_MIN : f1_time));
#else
    fin_1_time = pulse_start_time + FIN_REV_OFFSET - (f1_time > FIN_PULSE_MAX ? FIN_PULSE_MAX : (f1_time < FIN_PULSE_MIN ? FIN_PULSE_MIN : f1_time));
#endif

#ifndef FIN_2_DIRECTION_REV
    fin_2_time = pulse_start_time + (f2_time > FIN_PULSE_MAX ? FIN_PULSE_MAX : (f2_time < FIN_PULSE_MIN ? FIN_PULSE_MIN : f2_time));
#else
    fin_2_time = pulse_start_time + FIN_REV_OFFSET - (f2_time > FIN_PULSE_MAX ? FIN_PULSE_MAX : (f2_time < FIN_PULSE_MIN ? FIN_PULSE_MIN : f2_time));
#endif

#ifndef FIN_3_DIRECTION_REV
    fin_3_time = pulse_start_time + (f3_time > FIN_PULSE_MAX ? FIN_PULSE_MAX : (f3_time < FIN_PULSE_MIN ? FIN_PULSE_MIN : f3_time));
#else
    fin_3_time = pulse_start_time + FIN_REV_OFFSET - (f3_time > FIN_PULSE_MAX ? FIN_PULSE_MAX : (f3_time < FIN_PULSE_MIN ? FIN_PULSE_MIN : f3_time));
#endif

#ifndef FIN_4_DIRECTION_REV
    fin_4_time = pulse_start_time + (f4_time > FIN_PULSE_MAX ? FIN_PULSE_MAX : (f4_time < FIN_PULSE_MIN ? FIN_PULSE_MIN : f4_time));
#else
    fin_4_time = pulse_start_time + FIN_REV_OFFSET - (f4_time > FIN_PULSE_MAX ? FIN_PULSE_MAX : (f4_time < FIN_PULSE_MIN ? FIN_PULSE_MIN : f4_time));
#endif
}

uint8_t update_fins(unsigned long current_time) {
    if ((current_time > fin_1_time) && (ACTUATOR_PORT & SERVO_PIN_1_BIT)) ACTUATOR_PORT &= ~SERVO_PIN_1_BIT;
    if ((current_time > fin_2_time) && (ACTUATOR_PORT & SERVO_PIN_2_BIT)) ACTUATOR_PORT &= ~SERVO_PIN_2_BIT;
    if ((current_time > fin_3_time) && (ACTUATOR_PORT & SERVO_PIN_3_BIT)) ACTUATOR_PORT &= ~SERVO_PIN_3_BIT;
    if ((current_time > fin_4_time) && (ACTUATOR_PORT & SERVO_PIN_4_BIT)) ACTUATOR_PORT &= ~SERVO_PIN_4_BIT;

    return ACTUATOR_PORT & SERVO_PINS_ALL;
}
