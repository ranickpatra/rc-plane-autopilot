#include <Arduino.h>
#include "MPU6050.h"

MPU6050 mpu;


void setup() {

  pinMode(LED_BUILTIN, OUTPUT); // set led builtin output
  digitalWrite(LED_BUILTIN, LOW); // turnoff led

  // initilize mpu6050
  mpu.initialize();
}

void loop() {
  // put your main code here, to run repeatedly:
}