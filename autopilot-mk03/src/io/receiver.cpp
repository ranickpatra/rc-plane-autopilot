#include "receiver.h"

#define RECEIVER_CHANNEL_1_PIN_BIT (1 << RECEIVER_CHANNEL_1)
#define RECEIVER_CHANNEL_2_PIN_BIT (1 << RECEIVER_CHANNEL_2)
#define RECEIVER_CHANNEL_3_PIN_BIT (1 << RECEIVER_CHANNEL_3)
#define RECEIVER_CHANNEL_4_PIN_BIT (1 << RECEIVER_CHANNEL_4)

#define RECEIVER_CHANNEL_PIN_ALL (RECEIVER_CHANNEL_1_PIN_BIT | RECEIVER_CHANNEL_2_PIN_BIT | RECEIVER_CHANNEL_3_PIN_BIT | RECEIVER_CHANNEL_4_PIN_BIT)

volatile receiver_channel_data_t receiver_input;  // to hold the channel data from receiver
uint8_t receiver_channel_states[RECEIVER_CHANNEL_COUNT] = {0, 0, 0, 0};
unsigned long receiver_timers[RECEIVER_CHANNEL_COUNT] = {0, 0, 0, 0};
long receiver_current_time;

bool receiver_armed = false;

void receiver_init() {
    RECEIVER_DDR &= ~RECEIVER_CHANNEL_PIN_ALL;
    // ISR for receiver input
    PCICR |= (1 << RECEIVER_PCIE);
    RECEIVER_PCMSK |= (1 << RECEIVER_PCINT_CHANNEL_1);
    RECEIVER_PCMSK |= (1 << RECEIVER_PCINT_CHANNEL_2);
    RECEIVER_PCMSK |= (1 << RECEIVER_PCINT_CHANNEL_3);
    RECEIVER_PCMSK |= (1 << RECEIVER_PCINT_CHANNEL_4);
}

void receiver_on_interrupt() {
    receiver_current_time = micros();  // read current time in microseconds

    // Channel 1
    if (RECEIVER_PIN & RECEIVER_CHANNEL_1_PIN_BIT) {
        if (receiver_channel_states[0] == 0) {
            receiver_channel_states[0] = 1;
            receiver_timers[0] = receiver_current_time;
        }
    } else if (receiver_channel_states[0] == 1) {
        receiver_channel_states[0] = 0;
        receiver_input.channel[0] = receiver_current_time - receiver_timers[0];
    }

    // Channel 2
    if (RECEIVER_PIN & RECEIVER_CHANNEL_2_PIN_BIT) {
        if (receiver_channel_states[1] == 0) {
            receiver_channel_states[1] = 1;
            receiver_timers[1] = receiver_current_time;
        }
    } else if (receiver_channel_states[1] == 1) {
        receiver_channel_states[1] = 0;
        receiver_input.channel[1] = receiver_current_time - receiver_timers[1];
    }

    // Channel 3
    if (RECEIVER_PIN & RECEIVER_CHANNEL_3_PIN_BIT) {
        if (receiver_channel_states[2] == 0) {
            receiver_channel_states[2] = 1;
            receiver_timers[2] = receiver_current_time;
        }
    } else if (receiver_channel_states[2] == 1) {
        receiver_channel_states[2] = 0;
        receiver_input.channel[2] = receiver_current_time - receiver_timers[2];
    }

    // Channel 4
    if (RECEIVER_PIN & RECEIVER_CHANNEL_4_PIN_BIT) {
        if (receiver_channel_states[3] == 0) {
            receiver_channel_states[3] = 1;
            receiver_timers[3] = receiver_current_time;
        }
    } else if (receiver_channel_states[3] == 1) {
        receiver_channel_states[3] = 0;
        receiver_input.channel[3] = receiver_current_time - receiver_timers[3];
    }
}

void receiver_update() {
    if (!receiver_armed) {
        if (receiver_input.channel[0] < 1050 && receiver_input.channel[1] < 1050 && receiver_input.channel[2] < 1050 && receiver_input.channel[3] > 1950) {
            receiver_armed = true;
        }

        receiver_input.channel[0] = 1500;
        receiver_input.channel[1] = 1500;
        receiver_input.channel[2] = 1000;
        receiver_input.channel[3] = 1500;
    }
}

receiver_channel_data_t* receiver_get_channel_data() {
    return (receiver_channel_data_t*)&receiver_input;
}