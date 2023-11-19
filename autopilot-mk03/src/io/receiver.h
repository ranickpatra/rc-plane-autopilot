#pragma once

#include <Arduino.h>

#include "common/craft.h"


struct receiver_channel_data_t {
    uint16_t channel[RECEIVER_CHANNEL_COUNT];
};

void receiver_init();
void receiver_on_interrupt();
void receiver_update();
receiver_channel_data_t *receiver_get_channel_data();
