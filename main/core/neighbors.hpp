// main/core/neighbors.hpp
#pragma once
#include <stdint.h>
#include <stddef.h>

// ---- Neighbor record (compact, mm / mm/s to match State) ----
struct NeighborState {
  uint8_t mac[6];
  uint8_t team_id;                // team of this neighbor (for friend/foe distinction)
  int32_t x_mm, y_mm, z_mm;
  int32_t vx_mms, vy_mms, vz_mms;
  uint16_t heading_cd;
  uint32_t last_seen_ms;          // monotonic ms when last seen
};

// ---- Bounded snapshot pushed to Control by Radio ----
#include "config/params.hpp"
// MAX_NEIGHBORS is defined in params.hpp
constexpr size_t MAX_NEIGHBORS_VIEW = MAX_NEIGHBORS;   // max exported in a snapshot

struct NeighborsView {
  uint32_t t_ms;                   // snapshot time (ms)
  uint8_t  count_friends;          // number of friendly neighbors
  uint8_t  count_enemies;          // number of enemy neighbors
  NeighborState friends[MAX_NEIGHBORS_VIEW];
  NeighborState enemies[MAX_NEIGHBORS_VIEW];
};

// ---- Table API (implemented in core/neighbors.cpp) ----
void neighbors_init();
void neighbors_prune(uint32_t now_ms, uint32_t max_age_ms);
void neighbors_upsert(const NeighborState& s);
void neighbors_snapshot(NeighborsView& out, uint32_t now_ms);
void neighbors_upsert_enemy(const NeighborState& s);  // for tracking enemies
