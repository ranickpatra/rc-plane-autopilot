#pragma once

#include <avr/interrupt.h>

#include "common/craft.h"
#include "receiver.h"

ISR(PCINT2_vect) {
#if RECEIVER_PCIE == PCIE2
    receiver_on_interrupt();
#endif
}
