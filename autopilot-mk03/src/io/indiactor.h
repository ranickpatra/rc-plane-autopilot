#pragma once

#include <Arduino.h>

void indicator_init();

void indicator_blue_on();
void indicator_blue_off();
void indicator_green_on();
void indicator_green_off();
void indicator_red_on();
void indicator_red_off();
void indicator_blue_blink();
void indicator_green_blink();
void indicator_red_blink();

void indicator_show_number(uint8_t n);