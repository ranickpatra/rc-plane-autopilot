#include <Arduino.h>
#include "MPU6050.h"
#include "helper_3dmath.h"

#define DELTA_T 20000


MPU6050 mpu;

unsigned long int currentTime;


void setup() {

  pinMode(LED_BUILTIN, OUTPUT); // set led builtin output
  digitalWrite(LED_BUILTIN, LOW); // turnoff led

  // initilize mpu6050
  mpu.initialize();
  delay(100); // wait for sometime

  Serial.begin(115200);
  
  currentTime = micros() + DELTA_T;
  for (uint16_t i = 0; i < 2000; i++ ) {

  }
  
}

void loop() {
  

  mpu.readRawData();

}