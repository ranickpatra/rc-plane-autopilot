#pragma once

#include <stdint.h>
#include "config.h"
#include "global.h"

struct simple_low_pass_filter_t
{
  float data;
  float alpha;
};

struct simple_low_pass_filter_1d_t
{
  float *data;
  uint8_t size;
  float alpha;
};

struct mean_accumulator_t
{
  int32_t data;
  int32_t count;
};

void simple_low_pass_filter_init(simple_low_pass_filter_t *filter, float alpha);
void simple_low_pass_filter_update(simple_low_pass_filter_t *filter, float data);
void simple_low_pass_filter_init(simple_low_pass_filter_1d_t *filter, uint8_t size, float alpha);
void simple_low_pass_filter_update(simple_low_pass_filter_1d_t *filter, float *data);

void mean_accumulator_init(mean_accumulator_t *filter);
void mean_accumulator_add(mean_accumulator_t *filter, int32_t data);
uint16_t mean_accumulator_calculate(mean_accumulator_t *filter, int16_t default_value);

// kalman filter
void ekf_init(float alpha);
void ekf_update(float *gyro, float *acc);