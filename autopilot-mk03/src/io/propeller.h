#pragma once

#include <Arduino.h>

void propeller_init();
void set_propeller_pin_high(unsigned long p_time, unsigned long pulse_start_time);
uint8_t update_propeller(unsigned long current_time);