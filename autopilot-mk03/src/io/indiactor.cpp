#include "indiactor.h"

uint8_t indicator_status = 0;

void indicator_init()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, indicator_status);
}

void indicator_on()
{
  digitalWrite(LED_BUILTIN, HIGH);
}

void indicator_off()
{
  digitalWrite(LED_BUILTIN, LOW);
}

void indicator_blink()
{
  indicator_status = !indicator_status;
  digitalWrite(LED_BUILTIN, indicator_status);
}