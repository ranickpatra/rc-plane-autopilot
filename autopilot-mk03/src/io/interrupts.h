#pragma once

#include <avr/interrupt.h>

#include "common/craft.h"
#include "receiver.h"
#include "sensors/prop_rpm_sensor.h"

ISR(PCINT2_vect) {
#if RECEIVER_PCIE == PCIE2
    receiver_on_interrupt();
#endif

#if PROP_RPM_SENSOR_PCIE == PCIE2
    propeller_rpm_sensor_on_interrupt();
#endif
}
