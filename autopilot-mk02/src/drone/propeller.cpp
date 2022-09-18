#include "./propeller.h"

Propeller::Propeller(uint8_t pin, int16_t minSpeed, int16_t maxSpeed)
{
    this->pin = pin;
    this->minSpeed = minSpeed;
    this->maxSpeed = maxSpeed;
}

void Propeller::update() {
    if(this->speed > this->maxSpeed) this->speed = this->maxSpeed;
    else if(this->speed < this->minSpeed) this->speed = this->minSpeed;
}