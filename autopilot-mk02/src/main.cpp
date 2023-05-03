#include <Arduino.h>
#include "./drone/drone.h"
#include "./drone/propeller.h"
#include "./drone/fin.h"
#include "./receiver/receiver.h"

Drone drone;
Receiver receiver; // = Receiver();

// loop timers
// ----------------------------------------------------
unsigned long loop_timer;                                               // loop timer
unsigned long pwm_loop_time;                                            // loop timer pwm control
unsigned long propeller_time;                                           // propeller pwm time
unsigned long fin_times[4];                                             // fin pwm time
unsigned long receiver_channels_interrupt_time[RECEIVER_CHANNEL_COUNT]; // receiver channel interrupt time tracker
unsigned long receiver_interrupt_time;                                  // interrupt time for receiver
int last_channel_state[RECEIVER_CHANNEL_COUNT];

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

  // setup input for receiver
  DDRD &= B00001111;

  PCICR |= (1 << PCIE2);    // Set PCIE2 to enable PCMSK2 scan.
  PCMSK2 |= (1 << PCINT20); // Set PCINT20 (digital input 4) to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT21); // Set PCINT21 (digital input 5) to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT22); // Set PCINT22 (digital input 6) to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT23); // Set PCINT23 (digital input 7) to trigger an interrupt on state change.

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
  fin_times[0] = drone.fin1.get_microseconds() + loop_timer;
  fin_times[1] = drone.fin2.get_microseconds() + loop_timer;
  fin_times[2] = drone.fin3.get_microseconds() + loop_timer;
  fin_times[3] = drone.fin4.get_microseconds() + loop_timer;

  while (PORTB & B00011111)
  {
    pwm_loop_time = micros();
    if (propeller_time <= pwm_loop_time)
      PORTB &= B11111011;
    if (fin_times[0] <= pwm_loop_time)
      PORTB &= B11111110;
    if (fin_times[1] <= pwm_loop_time)
      PORTB &= B11111101;
    if (fin_times[2] <= pwm_loop_time)
      PORTB &= B11110111;
    if (fin_times[3] <= pwm_loop_time)
      PORTB &= B11101111;
  }
}

ISR(PCINT2_vect)
{
  receiver_interrupt_time = micros(); // get current time

  // channe 1 =========================================================
  if (PIND & B00010000)    // is pin 4 high?
  {
    if (last_channel_state[0] == 0) // is pin 4 already high?
    {
      last_channel_state[0] = 1;
      receiver_channels_interrupt_time[0] = receiver_interrupt_time;  // set current time
    }
  }
  else if (last_channel_state[0] == 1)  // is pin 4 not high and already changed high to low
  {
    last_channel_state[0] = 0;  // set timer to 0 to remember data already read
    receiver.channel[0] = (uint16_t)(receiver_interrupt_time - receiver_channels_interrupt_time[0]); // get the time difference
  }

  // channe 2 =========================================================
  if (PIND & B00100000)    // is pin 5 high?
  {
    if (last_channel_state[1] == 0) // is pin 5 already high?
    {
      last_channel_state[1] = 1;
      receiver_channels_interrupt_time[1] = receiver_interrupt_time;  // set current time
    }
  }
  else if (last_channel_state[1] == 1)  // is pin 5 not high and already changed high to low
  {
    last_channel_state[1] = 0;  // set timer to 0 to remember data already read
    receiver.channel[1] = (uint16_t)(receiver_interrupt_time - receiver_channels_interrupt_time[1]); // get the time difference
  }

  // channe 3 =========================================================
  if (PIND & B01000000)    // is pin 6 high?
  {
    if (last_channel_state[2] == 0) // is pin 6 already high?
    {
      last_channel_state[2] = 1;
      receiver_channels_interrupt_time[2] = receiver_interrupt_time;  // set current time
    }
  }
  else if (last_channel_state[2] == 1)  // is pin 6 not high and already changed high to low
  {
    last_channel_state[2] = 0;  // set timer to 0 to remember data already read
    receiver.channel[2] = (uint16_t)(receiver_interrupt_time - receiver_channels_interrupt_time[2]); // get the time difference
  }

  // channe 4 =========================================================
  if (PIND & B10000000)    // is pin 7 high?
  {
    if (last_channel_state[3] == 0) // is pin 7 already high?
    {
      last_channel_state[3] = 1;
      receiver_channels_interrupt_time[3] = receiver_interrupt_time;  // set current time
    }
  }
  else if (last_channel_state[3] == 1)  // is pin 7 not high and already changed high to low
  {
    last_channel_state[3] = 0;  // set timer to 0 to remember data already read
    receiver.channel[3] = (uint16_t)(receiver_interrupt_time - receiver_channels_interrupt_time[3]); // get the time difference
  }

  // for (uint8_t i = 0; i < RECEIVER_CHANNEL_COUNT; i++)
  // {
  //   // channe[i]  =========================================================
  //   if (PIND & (1 << (PIND4 + i))) // is pin i high?
  //   {
  //     if (receiver_channels_interrupt_time[i] == 0) // is pin i already high?
  //     {
  //       receiver_channels_interrupt_time[i] = receiver_interrupt_time; // set current time
  //     }
  //   }
  //   else if (receiver_channels_interrupt_time[i] != 0) // is pin i not high and already changed high to low
  //   {
  //     receiver.channel[i] = (uint16_t)(receiver_interrupt_time - receiver_channels_interrupt_time[i]); // get the time difference
  //     receiver_channels_interrupt_time[i] = 0;                                                         // set timer to 0 to remember data already read
  //   }
  // }
}