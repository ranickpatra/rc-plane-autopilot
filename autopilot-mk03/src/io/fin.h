#pragma once

#include <Arduino.h>



void fin_init();
void fin_set_pins_high(uint16_t f1_time, uint16_t f2_time, uint16_t f3_time, uint16_t f4_time, unsigned long pulse_start_time);
void fin_set_angles(int16_t f1, int16_t f2, int16_t f3, int16_t f4, unsigned long pulse_start_time);
uint8_t fin_update(unsigned long current_time);