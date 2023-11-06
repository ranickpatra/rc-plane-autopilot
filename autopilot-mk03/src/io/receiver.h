#pragma once

#include <Arduino.h>

void receiver_init();
void receiver_on_interrupt();
uint16_t *receiver_get_channel_data();
