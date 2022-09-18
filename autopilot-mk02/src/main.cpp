#include <Arduino.h>
#include "./drone/drone.h"
#include "./drone/propeller.h"
#include "./drone/fin.h"

Propeller propeller = Propeller(5, 0, 1000);
Fin fin1 = Fin(5, -9000, 9000);
Fin fin2 = Fin(5, -9000, 9000);
Fin fin3 = Fin(5, -9000, 9000);
Fin fin4 = Fin(5, -9000, 9000);
Drone drone = Drone(&propeller, &fin1, &fin2, &fin3, &fin4);

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  
  drone.update(); // update drone details
}