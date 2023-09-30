#include <Arduino.h>
#include "config.h"

#ifndef FC_FILTER_H
#define FC_FILTER_H

struct simple_low_pass_filter_t
{
  int32_t data;
  float alpha;
};

struct mean_accumulator_t
{
  int32_t data;
  int32_t count;
};




void simple_low_pass_filter_init(simple_low_pass_filter_t* filter, float alpha);
void simple_low_pass_filter_update(simple_low_pass_filter_t* filter, int32_t data);

void mean_accumulator_init(mean_accumulator_t* filter);
void mean_accumulator_add(mean_accumulator_t* filter, int32_t data);
uint16_t mean_accumulator_calculate(mean_accumulator_t* filter, int16_t default_value);

// kalman filter



#endif