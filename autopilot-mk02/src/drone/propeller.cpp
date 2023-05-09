#include "./propeller.h"

void Propeller::update() {
    if(this->speed > 100.0) this->speed = 100.0;
    else if(this->speed < 0.0) this->speed = 0.0;

    // throttle to signal(μs) conversion
    this->signal = (uint16_t)(this->speed * 10) + 1000;

    if(this->signal > 2000) this->signal = 2000;
    else if(this->signal < 1000) this->signal = 1000;
}

uint16_t Propeller::get_microseconds() {
    return this->signal;
}