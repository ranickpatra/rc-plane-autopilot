#pragma once

#include <stdint.h>

enum systemState_e {
    SYSTEM_STATE_INITIALISING = 0,
    SYSTEM_STATE_CONFIG_LOADED,
    SYSTEM_STATE_SENSORS_READY,
    SYSTEM_STATE_MOTORS_READY,
    SYSTEM_STATE_TRANSPONDER_ENABLED,
    SYSTEM_STATE_READY,
};


extern systemState_e systemState;


void fc_init();
