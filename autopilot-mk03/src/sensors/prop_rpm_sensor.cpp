#include "prop_rpm_sensor.h"

#include "common/craft.h"
#include "common/filter.h"

#define PROPELLER_RPM_SENSOR_PIN_BIT (1 << PROP_RPM_SENSOR_RPM_SENSOR_PIN)

uint8_t prop_rpm_sensor_state;
uint16_t prop_rpm_sensor_blade_pass_count = 0;
unsigned long propeller_rpm_sensor_prev_update_time = 0;  // in ms
simple_low_pass_filter_t prop_rpm_sensor_propeller_speed; // in revolution per second

void propeller_rpm_sensor_init() {
    PROP_RPM_SENSOR_DDR &= ~PROPELLER_RPM_SENSOR_PIN_BIT;
    // ISR for receiver input
    PCICR |= (1 << PROP_RPM_SENSOR_PCIE);
    PROP_RPM_SENSOR_PCMSK |= (1 << PROP_RPM_SENSOR_PCINT);

    filter_simple_low_pass_filter_init(&prop_rpm_sensor_propeller_speed, 0.3);
}

void propeller_rpm_sensor_on_interrupt() {
    if (PROP_RPM_SENSOR_PORT & PROPELLER_RPM_SENSOR_PIN_BIT) {
        if (prop_rpm_sensor_state == 0) {
            prop_rpm_sensor_state = 1;
        }
    } else if (prop_rpm_sensor_state == 1) {
        prop_rpm_sensor_state = 0;
        prop_rpm_sensor_blade_pass_count++;
    }
}

void propeller_rpm_sensor_update() {
    if (millis() - propeller_rpm_sensor_prev_update_time < 50) return;

    filter_simple_low_pass_filter_update(&prop_rpm_sensor_propeller_speed, prop_rpm_sensor_blade_pass_count / (PROP_BLADE_COUNT * 0.05));
    if (prop_rpm_sensor_propeller_speed.data < 0.01f) prop_rpm_sensor_propeller_speed.data = 0.01f;
    prop_rpm_sensor_blade_pass_count = 0;
    propeller_rpm_sensor_prev_update_time = millis();
}

float propeller_rpm_sensor_get_speed_rps() {
    return prop_rpm_sensor_propeller_speed.data;
}