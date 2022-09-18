#include "./fin.h"

Fin::Fin(uint8_t pin, int16_t minAngle, int16_t maxAngle)
{
    this->pin = pin;
    this->minAngle = minAngle;
    this->maxAngle = maxAngle;
}

void Fin::update() {
    if(this->angle > this->maxAngle) this->angle = this->maxAngle;
    else if(this->angle < this->minAngle) this->angle = this->minAngle;
}