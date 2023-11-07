#pragma once

#include <Arduino.h>



void fin_init();
void set_fin_pins_high(uint16_t f1_time, uint16_t f2_time, uint16_t f3_time, uint16_t f4_time, unsigned long pulse_start_time);
void set_fin_angles(int16_t f1, int16_t f2, int16_t f3, int16_t f4, unsigned long pulse_start_time);
uint8_t update_fins(unsigned long current_time);