#pragma once


enum axis_e {
    X = 0,
    Y,
    Z
};

#define XYZ_AXIS_COUNT 3

enum flight_dynamics_index_e {
  FD_ROLL=0,
  FD_PITCH,
  FD_YAW
};

#define FLIGHT_DYMANICS_INDEX_COUNT 3;