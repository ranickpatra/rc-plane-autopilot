#pragma once

#include <Arduino.h>

void propeller_init();
void propeller_set_pin_high(unsigned long p_time, unsigned long pulse_start_time);
// float get_propeller_input();
uint8_t propeller_update(unsigned long current_time);