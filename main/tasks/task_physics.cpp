#include "task_physics.hpp"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "config/params.hpp"
#include "core/state.hpp"
#include "core/metrics.hpp"
#include "globals.hpp"

#include <sys/time.h>

static const char* TAG = "task_phy";

// Helpers
static inline int32_t mm_from_m(float m)      { return static_cast<int32_t>(lrintf(m * 1000.0f)); }
static inline int32_t mms_from_mps(float v)   { return static_cast<int32_t>(lrintf(v * 1000.0f)); }
static inline uint16_t cd_from_rad(float rad) {
  // Normalize to [0, 2π), convert to centi-degrees
  while (rad < 0.0f)         rad += 2.0f * (float)M_PI;
  while (rad >= 2.0f * (float)M_PI) rad -= 2.0f * (float)M_PI;
  float deg = rad * (180.0f / (float)M_PI);
  int cd = (int)lrintf(deg * 100.0f);
  if (cd < 0) cd += 36000;
  return static_cast<uint16_t>(cd % 36000);
}

static inline void get_timestamp(uint32_t& ts_s, uint16_t& ts_ms) {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  ts_s  = (uint32_t)tv.tv_sec;
  ts_ms = (uint16_t)((tv.tv_usec / 1000) % 1000);
}

// Rate limiter: bring current toward target with accel cap
static inline float slew(float current, float target, float max_delta) {
  float d = target - current;
  if (d >  max_delta) d =  max_delta;
  if (d < -max_delta) d = -max_delta;
  return current + d;
}

// Simple wall reflection in a box
static inline void reflect_at_walls(float& p, float& v, float minp, float maxp) {
  if (p < minp) { p = minp + (minp - p); v = fabsf(v); }
  if (p > maxp) { p = maxp - (p - maxp); v = -fabsf(v); }
}

extern QueueHandle_t q_state_for_ctrl;
extern QueueHandle_t q_state_for_tx;
extern QueueHandle_t q_cmd_vel;

extern "C" void vTaskPhysics(void* pv) {
  (void)pv;
  ESP_LOGI(TAG, "Physics task start: %d ms period", PHY_PERIOD_MS);

  // State (meters, m/s)
  float px = 10.0f, py = 10.0f, pz = 1.2f; // start within bounds
  float vx = 0.4f, vy = 0.0f, vz = 0.0f;   // initial drift
  float yaw = 0.0f;                      // inferred from velocity
  unsigned seq = INITIAL_SEQ;

  // Desired velocity (from Control). If no commands, hold current velocity.
  float des_vx = vx, des_vy = vy, des_vz = vz;

  // The queues should be created in app_main; we tolerate nulls for stand-alone testing.
  TickType_t last = xTaskGetTickCount();
  const float dt = PHY_PERIOD_MS / 1000.0f;
  const float max_dv = MAX_ACCEL_MPS2 * dt;

  for (;;) {
    TickType_t t0 = xTaskGetTickCount();

    // 1) Optional non-blocking read of commanded velocity
    if (q_cmd_vel) {
      CmdVel cmd;
      if (xQueueReceive(q_cmd_vel, &cmd, 0) == pdTRUE) {
        des_vx = cmd.vx_mps; des_vy = cmd.vy_mps; des_vz = cmd.vz_mps;
      }
    }

    // 2) Rate-limit acceleration toward commanded velocity
    vx = slew(vx, des_vx, max_dv);
    vy = slew(vy, des_vy, max_dv);
    vz = slew(vz, des_vz, max_dv);

    // 3) Clamp speed
    float speed = sqrtf(vx*vx + vy*vy + vz*vz);
    if (speed > MAX_SPEED_MPS) {
      float scale = MAX_SPEED_MPS / speed;
      vx *= scale; vy *= scale; vz *= scale;
    }

    // 4) Integrate position
    px += vx * dt;
    py += vy * dt;
    pz += vz * dt;

    // 5) Keep inside world box (reflect)
    reflect_at_walls(px, vx, WORLD_X_MIN_M, WORLD_X_MAX_M);
    reflect_at_walls(py, vy, WORLD_Y_MIN_M, WORLD_Y_MAX_M);
    reflect_at_walls(pz, vz, WORLD_Z_MIN_M, WORLD_Z_MAX_M);

    // 6) Heading from planar velocity (+X is 0°, CCW positive)
    if (fabsf(vx) + fabsf(vy) > 1e-4f) {
      yaw = atan2f(vy, vx);
    }
    uint16_t heading_cd = cd_from_rad(yaw);

    // 7) Build State snapshot (integer, mm & mm/s)
    State s{};
    s.x_mm   = mm_from_m(px);
    s.y_mm   = mm_from_m(py);
    s.z_mm   = mm_from_m(pz);
    s.vx_mms = mms_from_mps(vx);
    s.vy_mms = mms_from_mps(vy);
    s.vz_mms = mms_from_mps(vz);
    s.heading_cd = heading_cd;
    get_timestamp(s.ts_s, s.ts_ms);
    s.seq   = static_cast<uint16_t>(seq++);

    // 8) Publish latest state to consumers (overwrite queues of length 1)
    if (q_state_for_ctrl) {
      (void)xQueueOverwrite(q_state_for_ctrl, &s);
    }
    if (q_state_for_tx) {
      (void)xQueueOverwrite(q_state_for_tx, &s);
    }

    TickType_t t1 = xTaskGetTickCount();
    metrics_record_phy(t0, t1);

    // 9) Run exactly at 50 Hz
    vTaskDelayUntil(&last, pdMS_TO_TICKS(PHY_PERIOD_MS));
  }
}
