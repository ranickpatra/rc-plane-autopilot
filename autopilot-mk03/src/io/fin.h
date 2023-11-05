#pragma once

#include <Arduino.h>



void fin_init();
void set_fin_pins_high(unsigned long f1_time, unsigned long f2_time, unsigned long f3_time, unsigned long f4_time);
uint8_t update_fins(unsigned long current_time);