#include "./fin.h"

Fin::Fin(double minAngle, double maxAngle)
{
    this->minAngle = minAngle;
    this->maxAngle = maxAngle;
}

void Fin::update() {
    if(this->angle > this->maxAngle) this->angle = this->maxAngle;
    else if(this->angle < this->minAngle) this->angle = this->minAngle;
}