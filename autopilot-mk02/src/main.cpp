#include <Arduino.h>
#include "./drone/drone.h"
#include "./drone/propeller.h"
#include "./drone/fin.h"
#include "./receiver/receiver.h"


Drone drone;
Receiver receiver; // = Receiver();

// loop timers
// ----------------------------------------------------
unsigned long loop_timer; // loop timer
unsigned long pwm_loop_time; // loop timer pwm control
unsigned long propeller_time;
unsigned long fin1_time;
unsigned long fin2_time;
unsigned long fin3_time;
unsigned long fin4_time;

#ifdef FIN_CHECK
unsigned long fin_prev_time = 0;
uint8_t fin_index = 0;
#endif

// setup
// ----------------------------------------------------
void setup()
{
  // setup for output for servo and propeller
  DDRB |= B00011111;
  PORTB &= B11100000;

  receiver.init();
  drone.set_receiver_channel(receiver.channel); // set receiver channel for drone

  // send a 1000μs pulse to servos and propeller
  PORTB |= B00011111;
  delayMicroseconds(1000);
  PORTB &= B11100000;

  loop_timer = micros(); // get current time in μs
}

// loop
// ----------------------------------------------------
void loop()
{

#ifdef FIN_CHECK
  // servo check
  if ((millis() > (fin_prev_time + 2000)) && (fin_index < 4))
  {
    fin_prev_time = millis();
    switch (fin_index)
    {
    case 0:
      if (drone.fin1.angle == 0)
        drone.fin1.angle = 45;
      else if (drone.fin1.angle > 0)
        drone.fin1.angle = -45;
      else if (drone.fin1.angle < 0)
        drone.fin1.angle = 0;
      break;

    case 1:
      if (drone.fin2.angle == 0)
        drone.fin2.angle = 45;
      else if (drone.fin2.angle > 0)
        drone.fin2.angle = -45;
      else if (drone.fin2.angle < 0)
        drone.fin2.angle = 0;
      break;

    case 2:
      if (drone.fin3.angle == 0)
        drone.fin3.angle = 45;
      else if (drone.fin3.angle > 0)
        drone.fin3.angle = -45;
      else if (drone.fin3.angle < 0)
        drone.fin3.angle = 0;
      break;

    case 3:
      if (drone.fin4.angle == 0)
        drone.fin4.angle = 45;
      else if (drone.fin4.angle > 0)
        drone.fin4.angle = -45;
      else if (drone.fin4.angle < 0)
        drone.fin4.angle = 0;
      break;
    }

    if (!drone.fin1.angle && !drone.fin2.angle && !drone.fin3.angle && !drone.fin4.angle)
      fin_index++;
  }

#else
  receiver.update(); // get data from receiver
  drone.update();    // update drone details
#endif

  while (micros() - loop_timer < LOOP_TIME)
    ;                    // wait until loop time passed
  loop_timer = micros(); // get current time in μs

  PORTB |= B00011111;
  propeller_time = drone.propeller.get_microseconds() + loop_timer;
  fin1_time = drone.fin1.get_microseconds() + loop_timer;
  fin2_time = drone.fin2.get_microseconds() + loop_timer;
  fin3_time = drone.fin3.get_microseconds() + loop_timer;
  fin4_time = drone.fin4.get_microseconds() + loop_timer;

  while (PORTB & B00011111)
  {
    pwm_loop_time = micros();
    if (propeller_time <= pwm_loop_time)
      PORTB &= B11111011;
    if (fin1_time <= pwm_loop_time)
      PORTB &= B11111110;
    if (fin2_time <= pwm_loop_time)
      PORTB &= B11111101;
    if (fin3_time <= pwm_loop_time)
      PORTB &= B11110111;
    if (fin4_time <= pwm_loop_time)
      PORTB &= B11101111;
  }
}