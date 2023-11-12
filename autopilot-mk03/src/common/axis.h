#pragma once


enum axis_e {
  AXIS_X = 0,
  AXIS_Y,
  AXIS_Z
};

#define XYZ_AXIS_COUNT 3

enum flight_dynamics_index_e {
  FD_ROLL=0,
  FD_PITCH,
  FD_YAW
};

#define FLIGHT_DYMANICS_INDEX_COUNT 3