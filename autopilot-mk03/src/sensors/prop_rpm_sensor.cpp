#include "prop_rpm_sensor.h"

#include "common/craft.h"
#include "common/filter.h"

#define PROPELLER_RPM_SENSOR_PIN_BIT (1 << PROP_RPM_SENSOR_RPM_SENSOR_PIN)

uint8_t propeller_rpm_sensor_state;
uint16_t propeller_blade_pass_count = 0;
// float propeller_speed = 0.01;                             // in revolution per second
unsigned long propeller_rpm_sensor_prev_update_time = 0;  // in ms

simple_low_pass_filter_t propeller_speed;

void propeller_rpm_sensor_init() {
    PROP_RPM_SENSOR_DDR &= ~PROPELLER_RPM_SENSOR_PIN_BIT;
    // ISR for receiver input
    PCICR |= (1 << PROP_RPM_SENSOR_PCIE);
    PROP_RPM_SENSOR_PCMSK |= (1 << PROP_RPM_SENSOR_PCINT);

    simple_low_pass_filter_init(&propeller_speed, 0.3);
}

void propeller_rpm_sensor_on_interrupt() {
    if (PROP_RPM_SENSOR_PORT & PROPELLER_RPM_SENSOR_PIN_BIT) {
        if (propeller_rpm_sensor_state == 0) {
            propeller_rpm_sensor_state = 1;
        }
    } else if (propeller_rpm_sensor_state == 1) {
        propeller_rpm_sensor_state = 0;
        propeller_blade_pass_count++;
    }
}

void propeller_rpm_sensor_update() {
    if (millis() - propeller_rpm_sensor_prev_update_time < 50) return;

    // propeller_speed = propeller_blade_pass_count / (PROP_BLADE_COUNT * 0.04);
    simple_low_pass_filter_update(&propeller_speed, propeller_blade_pass_count / (PROP_BLADE_COUNT * 0.05));
    if (propeller_speed.data < 0.01f) propeller_speed.data = 0.01f;
    propeller_blade_pass_count = 0;
    propeller_rpm_sensor_prev_update_time = millis();
}

float propeller_rpm_sensor_get_speed_rps() {
    return propeller_speed.data;
}