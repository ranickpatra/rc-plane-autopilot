#include "propeller.h"

#include "common/craft.h"

#define PROPELLER_PULSE_RANGE (ACTUATOR_PULSE_MAX - ACTUATOR_PULSE_MIN)
#define PROPELLER_PIN_BIT (1 << PROPELLER_PIN)

unsigned long prop_time = 0;
float prop_input = 0;

void propeller_init() {
    ACTUATOR_DDR |= PROPELLER_PIN_BIT;
    ACTUATOR_PORT &= ~PROPELLER_PIN_BIT;
}

void set_propeller_pin_high(unsigned long p_time, unsigned long pulse_start_time) {
    ACTUATOR_PORT |= PROPELLER_PIN_BIT;
    prop_time = pulse_start_time + (p_time > ACTUATOR_PULSE_MAX ? ACTUATOR_PULSE_MAX : (p_time < ACTUATOR_PULSE_MIN ? ACTUATOR_PULSE_MIN : p_time));
    prop_input = ((float)(p_time > ACTUATOR_PULSE_MAX ? ACTUATOR_PULSE_MAX : (p_time < ACTUATOR_PULSE_MIN ? ACTUATOR_PULSE_MIN : p_time)) - ACTUATOR_PULSE_MIN) / PROPELLER_PULSE_RANGE;
}

uint8_t update_propeller(unsigned long current_time) {
    if ((current_time > prop_time) && (ACTUATOR_PORT & PROPELLER_PIN_BIT)) ACTUATOR_PORT &= ~PROPELLER_PIN_BIT;
    return ACTUATOR_PORT & PROPELLER_PIN_BIT;
}

float get_propeller_input() {
    return prop_input;
}