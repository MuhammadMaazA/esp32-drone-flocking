#include "task_control.hpp"

#include <math.h>
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "config/params.hpp"
#include "core/state.hpp"
#include "core/neighbors.hpp"
#include "core/metrics.hpp"
#include "globals.hpp"

static const char* TAG = "task_ctrl";

extern QueueHandle_t q_state_for_ctrl;
extern QueueHandle_t q_cmd_vel;
extern QueueHandle_t q_neighbors;

// mm <-> m helpers
static inline float m_from_mm(int32_t mm){ return (float)mm * 0.001f; }
static inline float mps_from_mms(int32_t mms){ return (float)mms * 0.001f; }

// clamp vector to max norm
static inline void clamp_vec3(float& x, float& y, float& z, float max_norm){
  float n = sqrtf(x*x + y*y + z*z);
  if (n > max_norm && n > EPSILON) { float s = max_norm / n; x*=s; y*=s; z*=s; }
}

// weighted blend (scalar)
static inline float blend3(float a, float wa, float b, float wb, float c, float wc){
  return a*wa + b*wb + c*wc;
}

// Compute Reynolds rules given self and neighbors (friends only)
static void compute_flocking_friends(
  const State& self,
  const NeighborState* friends, uint8_t friend_count,
  float& out_vx_mps, float& out_vy_mps, float& out_vz_mps)
{
  // Self in meters
  const float sx = m_from_mm(self.x_mm);
  const float sy = m_from_mm(self.y_mm);
  const float sz = m_from_mm(self.z_mm);
  const float svx = mps_from_mms(self.vx_mms);
  const float svy = mps_from_mms(self.vy_mms);

  // If no friends, just cruise forward along current heading
  if (friend_count == 0) {
    float v = sqrtf(svx*svx + svy*svy);
    if (v < EPSILON) { out_vx_mps = CRUISE_V_MPS; out_vy_mps = 0.0f; }
    else         { out_vx_mps = svx;          out_vy_mps = svy;  }
    out_vz_mps = 0.0f;
    return;
  }

  // Accumulators for Reynolds rules (friends only)
  float coh_cx=0, coh_cy=0, coh_cz=0;   int coh_n=0;
  float aln_vx=0, aln_vy=0, aln_vz=0;   int aln_n=0;
  float sep_fx=0, sep_fy=0, sep_fz=0;   int sep_n=0;

  for (uint8_t i=0; i<friend_count; ++i){
    const auto& nb = friends[i];
    float nx = m_from_mm(nb.x_mm);
    float ny = m_from_mm(nb.y_mm);
    float nz = m_from_mm(nb.z_mm);
    float nvx = mps_from_mms(nb.vx_mms);
    float nvy = mps_from_mms(nb.vy_mms);
    float nvz = mps_from_mms(nb.vz_mms);

    float dx = nx - sx, dy = ny - sy, dz = nz - sz;
    float d2 = dx*dx + dy*dy + dz*dz;
    float d  = sqrtf(std::max(d2, 1e-5f));

    // Separation: if too close, push away ~ 1/d^2
    if (d < R_SEP_M) {
      float inv = 1.0f / std::max(d2, 1e-4f);
      sep_fx -= dx * inv;
      sep_fy -= dy * inv;
      sep_fz -= dz * inv;
      sep_n++;
    }

    // Cohesion / Alignment neighborhoods
    if (d < R_COH_M) {
      coh_cx += nx; coh_cy += ny; coh_cz += nz; coh_n++;
    }
    if (d < R_ALN_M) {
      aln_vx += nvx; aln_vy += nvy; aln_vz += nvz; aln_n++;
    }
  }

  // Cohesion: steer toward centroid
  float coh_vx=0, coh_vy=0, coh_vz=0;
  if (coh_n > 0) {
    float cx = coh_cx / (float)coh_n;
    float cy = coh_cy / (float)coh_n;
    float cz = coh_cz / (float)coh_n;
    coh_vx = (cx - sx);
    coh_vy = (cy - sy);
    coh_vz = (cz - sz);
    clamp_vec3(coh_vx, coh_vy, coh_vz, CRUISE_V_MPS);
  }

  // Alignment: match average neighbor velocity
  float aln_vx_d=0, aln_vy_d=0, aln_vz_d=0;
  if (aln_n > 0) {
    aln_vx_d = aln_vx / (float)aln_n;
    aln_vy_d = aln_vy / (float)aln_n;
    aln_vz_d = aln_vz / (float)aln_n;
  }

  // Separation: is a force; convert to a velocity "nudge"
  float sep_vx = sep_fx, sep_vy = sep_fy, sep_vz = sep_fz;
  clamp_vec3(sep_vx, sep_vy, sep_vz, CRUISE_V_MPS);

  // Blend the three behaviors (weighted sum)
  out_vx_mps = blend3(sep_vx, W_SEP, coh_vx, W_COH, aln_vx_d, W_ALN);
  out_vy_mps = blend3(sep_vy, W_SEP, coh_vy, W_COH, aln_vy_d, W_ALN);
  out_vz_mps = blend3(sep_vz, W_SEP, coh_vz, W_COH, aln_vz_d, W_ALN);
}

// Attack behavior: approach nearest enemy
static void compute_attack(
  const State& self,
  const NeighborState* enemies, uint8_t enemy_count,
  float& out_vx_mps, float& out_vy_mps, float& out_vz_mps)
{
  const float sx = m_from_mm(self.x_mm);
  const float sy = m_from_mm(self.y_mm);
  const float sz = m_from_mm(self.z_mm);

  if (enemy_count == 0) {
    out_vx_mps = out_vy_mps = out_vz_mps = 0.0f;
    return;
  }

  // Find nearest enemy
  float min_d2 = R_ATTACK_M * R_ATTACK_M;
  float target_x = 0.0f, target_y = 0.0f, target_z = 0.0f;
  bool found = false;

  for (uint8_t i=0; i<enemy_count; ++i){
    const auto& e = enemies[i];
    float ex = m_from_mm(e.x_mm);
    float ey = m_from_mm(e.y_mm);
    float ez = m_from_mm(e.z_mm);
    float dx = ex - sx, dy = ey - sy, dz = ez - sz;
    float d2 = dx*dx + dy*dy + dz*dz;
    if (d2 < min_d2) {
      min_d2 = d2;
      target_x = ex;
      target_y = ey;
      target_z = ez;
      found = true;
    }
  }

  if (found) {
    // Approach nearest enemy
    out_vx_mps = (target_x - sx);
    out_vy_mps = (target_y - sy);
    out_vz_mps = (target_z - sz);
    clamp_vec3(out_vx_mps, out_vy_mps, out_vz_mps, CRUISE_V_MPS);
  } else {
    out_vx_mps = out_vy_mps = out_vz_mps = 0.0f;
  }
}

// Defend behavior: avoid enemies
static void compute_defend(
  const State& self,
  const NeighborState* enemies, uint8_t enemy_count,
  float& out_vx_mps, float& out_vy_mps, float& out_vz_mps)
{
  const float sx = m_from_mm(self.x_mm);
  const float sy = m_from_mm(self.y_mm);
  const float sz = m_from_mm(self.z_mm);

  float avoid_fx=0, avoid_fy=0, avoid_fz=0;
  int avoid_n=0;

  for (uint8_t i=0; i<enemy_count; ++i){
    const auto& e = enemies[i];
    float ex = m_from_mm(e.x_mm);
    float ey = m_from_mm(e.y_mm);
    float ez = m_from_mm(e.z_mm);
    float dx = ex - sx, dy = ey - sy, dz = ez - sz;
    float d2 = dx*dx + dy*dy + dz*dz;
    float d = sqrtf(std::max(d2, 1e-5f));

    if (d < R_DEFEND_AVOID_M) {
      // Push away from enemy
      float inv = 1.0f / std::max(d2, 1e-4f);
      avoid_fx -= dx * inv;
      avoid_fy -= dy * inv;
      avoid_fz -= dz * inv;
      avoid_n++;
    }
  }

  if (avoid_n > 0) {
    out_vx_mps = avoid_fx;
    out_vy_mps = avoid_fy;
    out_vz_mps = avoid_fz;
    clamp_vec3(out_vx_mps, out_vy_mps, out_vz_mps, CRUISE_V_MPS);
  } else {
    out_vx_mps = out_vy_mps = out_vz_mps = 0.0f;
  }
}

// Main control computation: combines flocking with attack/defend
static void compute_flocking(
  const State& self,
  const NeighborsView* view,
  float& out_vx_mps, float& out_vy_mps, float& out_vz_mps)
{
  const float svx = mps_from_mms(self.vx_mms);
  const float svy = mps_from_mms(self.vy_mms);

  // Base flocking behavior with friends
  float flock_vx=0, flock_vy=0, flock_vz=0;
  if (view) {
    compute_flocking_friends(self, view->friends, view->count_friends,
                            flock_vx, flock_vy, flock_vz);
  } else {
    float v = sqrtf(svx*svx + svy*svy);
    if (v < EPSILON) { flock_vx = CRUISE_V_MPS; flock_vy = 0.0f; }
    else         { flock_vx = svx;          flock_vy = svy;  }
    flock_vz = 0.0f;
  }

  // Attack or defend behavior based on role
  float combat_vx=0, combat_vy=0, combat_vz=0;
  if (view && view->count_enemies > 0) {
    if (NODE_ROLE == FLAG_ROLE_ATTACKER) {
      compute_attack(self, view->enemies, view->count_enemies,
                     combat_vx, combat_vy, combat_vz);
      // Blend: flocking + attack
      out_vx_mps = flock_vx + combat_vx * W_ATTACK;
      out_vy_mps = flock_vy + combat_vy * W_ATTACK;
      out_vz_mps = flock_vz + combat_vz * W_ATTACK;
    } else {
      // Defender: flocking + avoid enemies
      compute_defend(self, view->enemies, view->count_enemies,
                     combat_vx, combat_vy, combat_vz);
      out_vx_mps = flock_vx + combat_vx * W_DEFEND_AVOID;
      out_vy_mps = flock_vy + combat_vy * W_DEFEND_AVOID;
      out_vz_mps = flock_vz + combat_vz * W_DEFEND_AVOID;
    }
  } else {
    // No enemies, just flocking
    out_vx_mps = flock_vx;
    out_vy_mps = flock_vy;
    out_vz_mps = flock_vz;
  }

  // If result is tiny, keep cruising
  float n = sqrtf(out_vx_mps*out_vx_mps + out_vy_mps*out_vy_mps + out_vz_mps*out_vz_mps);
  if (n < 0.1f*CRUISE_V_MPS) {
    out_vx_mps += svx; out_vy_mps += svy;
  }

  // Don't exceed max speed
  clamp_vec3(out_vx_mps, out_vy_mps, out_vz_mps, MAX_SPEED_MPS);
}

extern "C" void vTaskControl(void* pv)
{
  (void)pv;
  ESP_LOGI(TAG, "Control task start: %d ms period", CTRL_PERIOD_MS);

  TickType_t last = xTaskGetTickCount();

  // Cache the most recent neighbors view
  NeighborsView cached_view{};
  bool have_view = false;
  uint32_t log_counter = 0;  // For periodic logging

  for(;;){
    TickType_t t0 = xTaskGetTickCount();

    // 1) Get latest self state (non-blocking; use last if none)
    State self{};
    bool have_self = (q_state_for_ctrl &&
                      xQueueReceive(q_state_for_ctrl, &self, 0) == pdTRUE);

    if (!have_self) {
      // nothing to do without self state; keep timing
      vTaskDelayUntil(&last, pdMS_TO_TICKS(CTRL_PERIOD_MS));
      continue;
    }

    // 2) Grab newest neighbors snapshot if one arrived (reuse cached_view to avoid stack overflow)
    if (q_neighbors){
      if (xQueueReceive(q_neighbors, &cached_view, 0) == pdTRUE) {
        have_view = true;
      }
    }

    // 3) Compute desired velocity (m/s)
    float dvx=CRUISE_V_MPS, dvy=0.0f, dvz=0.0f;
    compute_flocking(self, have_view ? &cached_view : nullptr, dvx, dvy, dvz);

    // 4) Publish CmdVel (overwrite queue length=1)
    if (q_cmd_vel){
      CmdVel cmd{dvx, dvy, dvz};
      (void)xQueueOverwrite(q_cmd_vel, &cmd);
    }

            // Periodic logging to show flocking behavior (every 1 second = 10 iterations)
            if (++log_counter >= 10) {
              log_counter = 0;
              if (have_view) {
                ESP_LOGI(TAG, "Flocking: friends=%u enemies=%u vel=(%.2f,%.2f,%.2f) pos=(%d,%d,%d)",
                         cached_view.count_friends, cached_view.count_enemies,
                         dvx, dvy, dvz, self.x_mm, self.y_mm, self.z_mm);
                // Log neighbor details
                if (cached_view.count_friends > 0) {
                  ESP_LOGI(TAG, "  Friend[0]: MAC=%02X:%02X:%02X:%02X:%02X:%02X pos=(%d,%d,%d) vel=(%d,%d,%d)",
                           cached_view.friends[0].mac[0], cached_view.friends[0].mac[1], cached_view.friends[0].mac[2],
                           cached_view.friends[0].mac[3], cached_view.friends[0].mac[4], cached_view.friends[0].mac[5],
                           cached_view.friends[0].x_mm, cached_view.friends[0].y_mm, cached_view.friends[0].z_mm,
                           cached_view.friends[0].vx_mms, cached_view.friends[0].vy_mms, cached_view.friends[0].vz_mms);
                }
              } else {
                ESP_LOGI(TAG, "Flocking: no neighbors, cruising vel=(%.2f,%.2f,%.2f) pos=(%d,%d,%d)",
                         dvx, dvy, dvz, self.x_mm, self.y_mm, self.z_mm);
              }
            }

    TickType_t t1 = xTaskGetTickCount();
    metrics_record_ctrl(t0, t1);

    // 5) Exactly 10 Hz
    vTaskDelayUntil(&last, pdMS_TO_TICKS(CTRL_PERIOD_MS));
  }
}
