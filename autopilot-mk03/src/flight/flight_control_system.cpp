#include "flight_control_system.h"

#include "common/craft.h"
#include "common/maths.h"
#include "physics.h"
#include "sensors/prop_rpm_sensor.h"

float flight_control_system_prop_speed;
float flight_control_system_fin_angles[FIN_COUNT] = {0.0, 0.0, 0.0, 0.0};

// Convert the control outputs to fin angles considering the force and propeller speed
float* flight_control_system_get_force_to_fin_angle(pid_data_t pid_force_data[FLIGHT_DYMANICS_INDEX_COUNT]) {
    // Get the current propeller speed to calculate the fin angles
    flight_control_system_prop_speed = propeller_rpm_sensor_get_speed_rps();

    // TODO calculate force for each fin in better way
    flight_control_system_fin_angles[FIN1] = pid_force_data[FD_ROLL].sum * 0.5 + pid_force_data[FD_YAW].sum * 0.25;
    flight_control_system_fin_angles[FIN2] = pid_force_data[FD_PITCH].sum * 0.5 + pid_force_data[FD_YAW].sum * 0.25;
    flight_control_system_fin_angles[FIN3] = pid_force_data[FD_ROLL].sum * 0.5 - pid_force_data[FD_YAW].sum * 0.25;
    flight_control_system_fin_angles[FIN4] = pid_force_data[FD_PITCH].sum * 0.5 - pid_force_data[FD_YAW].sum * 0.25;

    flight_control_system_fin_angles[FIN1] =
            physics_get_fin_angle_from_force(flight_control_system_fin_angles[FIN1], flight_control_system_prop_speed);
    flight_control_system_fin_angles[FIN2] =
            -physics_get_fin_angle_from_force(flight_control_system_fin_angles[FIN2], flight_control_system_prop_speed);
    flight_control_system_fin_angles[FIN3] =
            physics_get_fin_angle_from_force(flight_control_system_fin_angles[FIN3], flight_control_system_prop_speed);
    flight_control_system_fin_angles[FIN4] =
            -physics_get_fin_angle_from_force(flight_control_system_fin_angles[FIN4], flight_control_system_prop_speed);

    return flight_control_system_fin_angles;
}

float* flight_control_system_get_fin_angle() {
    return flight_control_system_fin_angles;
}