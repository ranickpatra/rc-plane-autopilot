#include <Arduino.h>
#include "./drone/drone.h"
#include "./drone/propeller.h"
#include "./drone/fin.h"
#include "./receiver/receiver.h"


unsigned long loopTimer;  // loop timer

// Drone drone = Drone();
Receiver receiver;// = Receiver();

void setup() {
  receiver.init();
  // drone.setReceiverChannel(receiver.channel); // set receiver channel for drone
  // setup for putputs
  DDRB  |= B00011111;
  PORTB &= B11100000;

  PORTB |= B00011111;
  delayMicroseconds(1000);
  PORTB &= B11100000;

  loopTimer = micros(); // get current time in μs
}

void loop() {
  
  receiver.update();  // get data from receiver
  // drone.update(); // update drone details
  
  while(micros() - loopTimer < LOOP_TIME ); // wait until loop time passed
  loopTimer = micros(); // get current time in μs

  PORTB |= B00011011;
  // unsigned long thrustTime = receiver.channel[2] + loopTimer;
  unsigned long thrustTime = 1500 + loopTimer;

  while(thrustTime > micros());
  
  PORTB &= B11100100;
  
}