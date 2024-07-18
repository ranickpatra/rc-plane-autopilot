#pragma once

#include "pid.h"
#include "common/axis.h"


float* flight_control_system_get_force_to_fin_angle(pid_data_t pid_force_data[FLIGHT_DYMANICS_INDEX_COUNT]);
float* flight_control_system_get_fin_angle();
