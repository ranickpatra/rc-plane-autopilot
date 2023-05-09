#include "./fin.h"

Fin::Fin(double min_angle, double max_angle, uint16_t center)
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

    /**
     *          1000
     * 1 deg = ------ = 5.555556 μs
     *          180
     * 
     */

    this->signal = (uint16_t(this->angle * 5.555556)) + this->center;

    if(this->signal > 2000) this->signal = 2000;
    else if(this->signal < 1000) this->signal = 1000;

}

uint16_t Fin::get_microseconds()
{
    return this->signal;
}