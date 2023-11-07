#include "indiactor.h"

#include "common/craft.h"

#define INDICATOR_PIN_BLUE_BIT (1 << INDICATOR_PIN_BLUE)
#define INDICATOR_PIN_GREEN_BIT (1 << INDICATOR_PIN_GREEN)
#define INDICATOR_PIN_RED_BIT (1 << INDICATOR_PIN_RED)

void indicator_init() {
    INDICATOR_DDR |= (INDICATOR_PIN_BLUE_BIT | INDICATOR_PIN_GREEN_BIT | INDICATOR_PIN_RED_BIT);
    INDICATOR_PORT &= ~(INDICATOR_PIN_BLUE_BIT | INDICATOR_PIN_GREEN_BIT | INDICATOR_PIN_RED_BIT);  // turn off all leds
}

void indicator_blue_on() {
    INDICATOR_PORT |= INDICATOR_PIN_BLUE_BIT;
}

void indicator_blue_off() {
    INDICATOR_PORT &= ~INDICATOR_PIN_BLUE_BIT;
}

void indicator_green_on() {
    INDICATOR_PORT |= INDICATOR_PIN_GREEN_BIT;
}

void indicator_green_off() {
    INDICATOR_PORT &= ~INDICATOR_PIN_GREEN_BIT;
}

void indicator_red_on() {
    INDICATOR_PORT |= INDICATOR_PIN_RED_BIT;
}

void indicator_red_off() {
    INDICATOR_PORT &= ~INDICATOR_PIN_RED_BIT;
}

void indicator_blue_blink() {
    if (INDICATOR_PORT & INDICATOR_PIN_BLUE_BIT) {
        indicator_blue_off();
    } else {
        indicator_blue_on();
    }
}

void indicator_green_blink() {
    if (INDICATOR_PORT & INDICATOR_PIN_GREEN_BIT) {
        indicator_green_off();
    } else {
        indicator_green_on();
    }
}

void indicator_red_blink() {
    if (INDICATOR_PORT & INDICATOR_PIN_RED_BIT) {
        indicator_red_off();
    } else {
        indicator_red_on();
    }
}

void indicator_show_number(uint8_t n) {
    if (n == 1 || n == 3 || n == 5 || n == 7) {
        indicator_red_on();
    } else {
        indicator_red_off();
    }

    if (n == 2 || n == 3 || n == 6 || n == 7) {
        indicator_green_on();
    } else {
        indicator_green_off();
    }

    if (n == 4 || n == 5 || n == 6 || n == 7) {
        indicator_blue_on();
    } else {
        indicator_blue_off();
    }
}