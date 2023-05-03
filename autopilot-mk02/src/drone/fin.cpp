#include "./fin.h"

Fin::Fin(int8_t min_angle, int8_t max_angle, uint16_t center)
{
    this->min_angle = min_angle;
    this->max_angle = max_angle;
    this->center = center;
}

void Fin::update()
{
    if (this->angle > this->max_angle)
        this->angle = this->max_angle;
    else if (this->angle < this->min_angle)
        this->angle = this->min_angle;
}

uint16_t Fin::get_microseconds()
{
    /**
     *          1000
     * 1 deg = ------ = 5.555556 us
     *          180
     * 
     */

    return (uint16_t)(this->angle * 5.555556 + this->center);
}