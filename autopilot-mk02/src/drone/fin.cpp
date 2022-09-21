#include "./fin.h"

Fin::Fin(double minAngle, double maxAngle, uint16_t center)
{
    this->minAngle = minAngle;
    this->maxAngle = maxAngle;
    this->center = center;
}

void Fin::update()
{
    if (this->angle > this->maxAngle)
        this->angle = this->maxAngle;
    else if (this->angle < this->minAngle)
        this->angle = this->minAngle;
}

uint16_t Fin::getPWM()
{
    /**
     *          1000
     * 1 deg = ------ = 5.555556 us
     *          180
     * 
     */

    return (uint16_t)(this->angle * 5.555556 + this->center);
}