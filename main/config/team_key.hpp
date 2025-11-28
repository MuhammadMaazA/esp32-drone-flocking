// main/config/team_key.hpp
#pragma once
#include <stdint.h>
#include "sdkconfig.h"

// Helper to expose the configured 128-bit team key to multiple components.
// Note: This must be in a .cpp file or the function must be defined after sdkconfig.h is included
inline const uint8_t* get_team_key()
{
  static const uint8_t key[16] = {
      CONFIG_TEAM_KEY_0,  CONFIG_TEAM_KEY_1,  CONFIG_TEAM_KEY_2,  CONFIG_TEAM_KEY_3,
      CONFIG_TEAM_KEY_4,  CONFIG_TEAM_KEY_5,  CONFIG_TEAM_KEY_6,  CONFIG_TEAM_KEY_7,
      CONFIG_TEAM_KEY_8,  CONFIG_TEAM_KEY_9,  CONFIG_TEAM_KEY_10, CONFIG_TEAM_KEY_11,
      CONFIG_TEAM_KEY_12, CONFIG_TEAM_KEY_13, CONFIG_TEAM_KEY_14, CONFIG_TEAM_KEY_15};
  return key;
}

