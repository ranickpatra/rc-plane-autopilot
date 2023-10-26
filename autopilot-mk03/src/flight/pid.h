#pragma once

struct pidData_t
{
  float P;
  float I;
  float D;

  float Sum;
};


struct pidCoefficient_t {
  float Kp;
  float Ki;
  float Kd;
};
