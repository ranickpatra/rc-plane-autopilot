#pragma once

#include <Arduino.h>

// #define MAX_PROP_WIND_SPEED 20.0  // ms^-1
#define PROPELLER_PITCH (10.67 / 100) // m
#define PROPELLER_SLIP 0.2 // 20%
#define DENSITY_OF_AIR 1.293      // Kgm^-3
// #define AREA_OF_FIN 0.003334      // m^2
// #define AREA_OF_FIN 0.0012
#define AREA_OF_FIN 0.00318
#define FIN_MAX_COEF_OF_LIFT_ANGLE 30.0


// from this we can find the angle of fin based on coefficient of lift
#define FIN_COEFF_LIFT_EQUATION {0.0, 0.03789802668726148, -0.00061862700596408} // c + bx + ax^2


double physics_get_fin_angle_from_force(double force, float prop_speed);
float physics_get_thrust_from_roll_pitch(float roll, float pitch);

