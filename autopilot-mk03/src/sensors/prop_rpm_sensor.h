#pragma once

#include <Arduino.h>

void propeller_rpm_sensor_init();
void propeller_rpm_sensor_on_interrupt();
void propeller_rpm_sensor_update();
float propeller_rpm_sensor_get_speed();