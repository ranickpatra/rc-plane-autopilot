#include "receiver.h"

#include "common/craft.h"

#define RECEIVER_CHANNEL_1_PIN_BIT (1 << RECEIVER_CHANNEL_1)
#define RECEIVER_CHANNEL_2_PIN_BIT (1 << RECEIVER_CHANNEL_2)
#define RECEIVER_CHANNEL_3_PIN_BIT (1 << RECEIVER_CHANNEL_3)
#define RECEIVER_CHANNEL_4_PIN_BIT (1 << RECEIVER_CHANNEL_4)

#define RECEIVER_CHANNEL_PIN_ALL (RECEIVER_CHANNEL_1_PIN_BIT | RECEIVER_CHANNEL_2_PIN_BIT | RECEIVER_CHANNEL_3_PIN_BIT | RECEIVER_CHANNEL_4_PIN_BIT)

volatile uint16_t receiver_input_channel[RECEIVER_CHANNEL_COUNT];  // to hold the channel data from receiver
uint8_t channel_states[RECEIVER_CHANNEL_COUNT] = {0, 0, 0, 0};
unsigned long timers[RECEIVER_CHANNEL_COUNT] = {0, 0, 0, 0};
unsigned long current_time;

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
    current_time = micros();  // read current time in microseconds

    // Channel 1
    if (RECEIVER_PIN & RECEIVER_CHANNEL_1_PIN_BIT) {
        if (channel_states[0] == 0) {
            channel_states[0] = 1;
            timers[0] = current_time;
        }
    } else if (channel_states[0] == 1) {
        channel_states[0] = 0;
        receiver_input_channel[0] = current_time - timers[0];
    }

    // Channel 2
    if (RECEIVER_PIN & RECEIVER_CHANNEL_2_PIN_BIT) {
        if (channel_states[1] == 0) {
            channel_states[1] = 1;
            timers[1] = current_time;
        }
    } else if (channel_states[1] == 1) {
        channel_states[1] = 0;
        receiver_input_channel[1] = current_time - timers[1];
    }

    // Channel 3
    if (RECEIVER_PIN & RECEIVER_CHANNEL_3_PIN_BIT) {
        if (channel_states[2] == 0) {
            channel_states[2] = 1;
            timers[2] = current_time;
        }
    } else if (channel_states[2] == 1) {
        channel_states[2] = 0;
        receiver_input_channel[2] = current_time - timers[2];
    }

    // Channel 4
    if (RECEIVER_PIN & RECEIVER_CHANNEL_4_PIN_BIT) {
        if (channel_states[3] == 0) {
            channel_states[3] = 1;
            timers[3] = current_time;
        }
    } else if (channel_states[3] == 1) {
        channel_states[3] = 0;
        receiver_input_channel[3] = current_time - timers[3];
    }
}