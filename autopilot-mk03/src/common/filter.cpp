#include "filter.h"

void simple_low_pass_filter_init(simple_low_pass_filter_t *filter, float alpha)
{
  filter->data = 0;
  filter->alpha = alpha;
}

void simple_low_pass_filter_update(simple_low_pass_filter_t *filter, int32_t data)
{
  filter->data = (1 - filter->alpha) * filter->data + filter->alpha * data;
}

void mean_accumulator_init(mean_accumulator_t *filter)
{
  filter->data = 0;
  filter->count = 0;
}

void mean_accumulator_add(mean_accumulator_t *filter, int32_t data)
{
  filter->data += data;
  filter->count++;
}

uint16_t mean_accumulator_calculate(mean_accumulator_t *filter, int16_t default_value)
{
  if (filter->count)
  {
    uint16_t result = filter->data / filter->count;
    mean_accumulator_init(filter);
    return result;
  }
  return default_value;  
}