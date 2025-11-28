#include "core/neighbors.hpp"
#include "config/params.hpp"
#include <string.h>

static NeighborState g_tab[MAX_NEIGHBORS];

void neighbors_init(){ memset(g_tab, 0, sizeof(g_tab)); }

static int find_slot(const uint8_t mac[6]){
  for (int i=0;i<(int)MAX_NEIGHBORS;i++){
    if (g_tab[i].last_seen_ms != 0 && memcmp(g_tab[i].mac, mac, 6)==0) return i;
  }
  return -1;
}
static int find_free(){
  for (int i=0;i<(int)MAX_NEIGHBORS;i++){
    if (g_tab[i].last_seen_ms == 0) return i;
  }
  // If full, replace the stalest
  int worst = 0;
  for (int i=1;i<(int)MAX_NEIGHBORS;i++){
    if (g_tab[i].last_seen_ms < g_tab[worst].last_seen_ms) worst = i;
  }
  return worst;
}

void neighbors_upsert(const NeighborState& s){
  int idx = find_slot(s.mac);
  if (idx < 0) idx = find_free();
  g_tab[idx] = s;
}

void neighbors_upsert_enemy(const NeighborState& s){
  // Same implementation - we distinguish by team_id in snapshot
  neighbors_upsert(s);
}

void neighbors_prune(uint32_t now_ms, uint32_t max_age_ms){
  for (size_t i=0;i<MAX_NEIGHBORS;i++){
    if (g_tab[i].last_seen_ms && (now_ms - g_tab[i].last_seen_ms) > max_age_ms){
      memset(&g_tab[i], 0, sizeof(NeighborState));
    }
  }
}

void neighbors_snapshot(NeighborsView& out, uint32_t now_ms){
  out.t_ms = now_ms;
  out.count_friends = 0;
  out.count_enemies = 0;
  for (size_t i=0;i<MAX_NEIGHBORS;i++){
    if (g_tab[i].last_seen_ms){
      if (g_tab[i].team_id == TEAM_ID) {
        // Friend
        if (out.count_friends < MAX_NEIGHBORS_VIEW) {
          out.friends[out.count_friends++] = g_tab[i];
        }
      } else {
        // Enemy
        if (out.count_enemies < MAX_NEIGHBORS_VIEW) {
          out.enemies[out.count_enemies++] = g_tab[i];
        }
      }
    }
  }
}
