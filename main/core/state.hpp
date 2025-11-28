#pragma once
#include <stdint.h>

struct State {
  // Position [mm]
  int32_t x_mm, y_mm, z_mm;
  // Velocity [mm/s]
  int32_t vx_mms, vy_mms, vz_mms;
  // Heading [centi-degrees] (0 = +X, +CCW)
  uint16_t heading_cd;

  // Timestamp and sequence
  uint32_t ts_s;
  uint16_t ts_ms;
  uint16_t seq;
};
