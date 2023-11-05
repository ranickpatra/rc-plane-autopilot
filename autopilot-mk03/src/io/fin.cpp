#include "fin.h"

#include "common/craft.h"

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

void set_fin_pins_high(unsigned long f1_time, unsigned long f2_time, unsigned long f3_time, unsigned long f4_time) {
    fin_1_time = f1_time;
    fin_2_time = f2_time;
    fin_3_time = f3_time;
    fin_4_time = f4_time;
    ACTUATOR_PORT |= SERVO_PINS_ALL;
}

uint8_t update_fins(unsigned long current_time) {
    if ((current_time > fin_1_time) && (ACTUATOR_PORT & SERVO_PIN_1_BIT)) ACTUATOR_PORT &= ~SERVO_PIN_1_BIT;
    if ((current_time > fin_2_time) && (ACTUATOR_PORT & SERVO_PIN_2_BIT)) ACTUATOR_PORT &= ~SERVO_PIN_2_BIT;
    if ((current_time > fin_3_time) && (ACTUATOR_PORT & SERVO_PIN_3_BIT)) ACTUATOR_PORT &= ~SERVO_PIN_3_BIT;
    if ((current_time > fin_4_time) && (ACTUATOR_PORT & SERVO_PIN_4_BIT)) ACTUATOR_PORT &= ~SERVO_PIN_4_BIT;

    return ACTUATOR_PORT & SERVO_PINS_ALL;
}
