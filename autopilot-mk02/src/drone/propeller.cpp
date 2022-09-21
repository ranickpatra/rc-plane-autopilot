#include "./propeller.h"

void Propeller::update() {
    if(this->speed > 100.0) this->speed = 100.0;
    else if(this->speed < 0.0) this->speed = 0.0;
}

uint16_t Propeller::getPWM() {
    return (uint16_t)(this->speed * 10) + 1000;
}