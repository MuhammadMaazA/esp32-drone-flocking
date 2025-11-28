// main/globals.hpp
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "core/state.hpp"
#include "core/neighbors.hpp"

// ---- Types shared between tasks ----
struct CmdVel {            // desired velocity [m/s]
  float vx_mps, vy_mps, vz_mps;
};

// ---- Queues (created in app_main) ----
// PHY -> CTRL (latest self state)
extern QueueHandle_t q_state_for_ctrl;

// PHY -> RADIO/TELEM (latest self state)
extern QueueHandle_t q_state_for_tx;

// CTRL -> PHY (desired velocity)
extern QueueHandle_t q_cmd_vel;

// RADIO -> CTRL (snapshot of neighbors)
extern QueueHandle_t q_neighbors;
