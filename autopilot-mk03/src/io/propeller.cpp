#include "propeller.h"

#include "common/craft.h"

#define PROPELLER_PIN_BIT (1 << PROPELLER_PIN)

unsigned long prop_time = 0;

void propeller_init() {
    ACTUATOR_DDR |= PROPELLER_PIN_BIT;
    ACTUATOR_PORT &= ~PROPELLER_PIN_BIT;
}

void set_propeller_pin_high(unsigned long p_time, unsigned long pulse_start_time) {
    prop_time = pulse_start_time + (p_time > 2000 ? 2000 : (p_time < 1000 ? 1000 : p_time));
    ACTUATOR_PORT |= PROPELLER_PIN_BIT;
}

uint8_t update_propeller(unsigned long current_time) {
    if ((current_time > prop_time) && (ACTUATOR_PORT & PROPELLER_PIN_BIT)) ACTUATOR_PORT &= ~PROPELLER_PIN_BIT;
    return ACTUATOR_PORT & PROPELLER_PIN_BIT;
}
