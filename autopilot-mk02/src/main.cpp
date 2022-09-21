#include <Arduino.h>
#include "./drone/drone.h"
#include "./drone/propeller.h"
#include "./drone/fin.h"
#include "./receiver/receiver.h"

unsigned long loopTimer; // loop timer

Drone drone;
Receiver receiver; // = Receiver();

void setup()
{
  receiver.init();
  drone.setReceiverChannel(receiver.channel); // set receiver channel for drone
  // setup for putputs
  DDRB |= B00011111;
  PORTB &= B11100000;

  PORTB |= B00011111;
  delayMicroseconds(1000);
  PORTB &= B11100000;

  loopTimer = micros(); // get current time in μs
}

void loop()
{

  receiver.update(); // get data from receiver
  drone.update();    // update drone details

  while (micros() - loopTimer < LOOP_TIME)
    ;                   // wait until loop time passed
  loopTimer = micros(); // get current time in μs

  PORTB |= B00011011;
  // unsigned long thrustTime = receiver.channel[2] + loopTimer;
  // unsigned long thrustTime = 1500 + loopTimer;

  unsigned long propeller_time = drone.propeller.getPWM() + loopTimer;
  unsigned long fin1_time = drone.fin1.getPWM() + loopTimer;
  unsigned long fin2_time = drone.fin2.getPWM() + loopTimer;
  unsigned long fin3_time = drone.fin3.getPWM() + loopTimer;
  unsigned long fin4_time = drone.fin4.getPWM() + loopTimer;

  unsigned long loopTime;
  while (PORTB & B00011111)
  {
    loopTime = micros();
    if (propeller_time <= loopTime)
      PORTB &= B11111011;
    if (fin1_time <= loopTime)
      PORTB &= B11111110;
    if (fin2_time <= loopTime)
      PORTB &= B11111101;
    if (fin3_time <= loopTime)
      PORTB &= B11110111;
    if (fin4_time <= loopTime)
      PORTB &= B11101111;
  }
}