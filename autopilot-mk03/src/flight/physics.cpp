#include "physics.h"

#include "common/maths.h"

/**
 *          2 x force_required
 * --------------------------------------------
 *  density_of_air x area_of_fin x airspeed^2
 *
 *                                  2 x force_required
 * = -----------------------------------------------------------------------------------
 *    density_of_air x area_of_fin x (prop_speed_rps x prop_pitch x (1 - prop_slip))^2
 *
 *                                  2                                       force_required
 * = -----------------------------------------------------------------  x  ------------------
 *    density_of_air x area_of_fin x (prop_pitch x (1 - prop_slip))^2       prop_speed_rps^2
 */

#define PROP_CONST_1 (2 / (DENSITY_OF_AIR * AREA_OF_FIN * (PROPELLER_PITCH * PROPELLER_PITCH) * ((1 - PROPELLER_SLIP) * (1 - PROPELLER_SLIP))))

double fin_lift_coeff_equation[3] = FIN_COEFF_LIFT_EQUATION;
double fin_coeff_of_lift;
double quadratic_roots[2];
uint8_t quadratic_roots_count;

double physics_get_fin_angle_from_force(double force, float prop_speed) {
    // get the coeff of lift required for that propeler speed
    fin_coeff_of_lift = abs(force) * PROP_CONST_1 / (prop_speed * prop_speed);

    if (fin_coeff_of_lift < 0.0) fin_coeff_of_lift = 0;

    // solve the quadratic equation to get the angle for the coefficient of lift
    solve_quadratic(fin_lift_coeff_equation[2], fin_lift_coeff_equation[1], fin_lift_coeff_equation[0], fin_coeff_of_lift, quadratic_roots[0],
                    quadratic_roots[1], quadratic_roots_count);

    switch (quadratic_roots_count) {
        case 2:
            if (quadratic_roots[0] >= 0.0 && quadratic_roots[0] <= FIN_MAX_COEF_OF_LIFT_ANGLE && quadratic_roots[1] >= 0.0 &&
                quadratic_roots[1] <= FIN_MAX_COEF_OF_LIFT_ANGLE) {
                quadratic_roots[0] = min(quadratic_roots[0], quadratic_roots[1]);
            } else if (quadratic_roots[0] >= 0.0 && quadratic_roots[0] <= FIN_MAX_COEF_OF_LIFT_ANGLE) {
            } else if (quadratic_roots[1] >= 0.0 && quadratic_roots[1] <= FIN_MAX_COEF_OF_LIFT_ANGLE) {
                quadratic_roots[0] = quadratic_roots[1];
            } else {
                quadratic_roots[0] = FIN_MAX_COEF_OF_LIFT_ANGLE;
            }
            break;
        case 1:
            if (quadratic_roots[0] < 0.0) {
                quadratic_roots[0] = 0.0;
            } else if (quadratic_roots[0] > FIN_MAX_COEF_OF_LIFT_ANGLE) {
                quadratic_roots[0] = FIN_MAX_COEF_OF_LIFT_ANGLE;
            }
            break;
        case 0:
            quadratic_roots[0] = FIN_MAX_COEF_OF_LIFT_ANGLE;
    }

    return force > 0.0 ? quadratic_roots[0] : -quadratic_roots[0];
}