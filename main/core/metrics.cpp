#include "core/metrics.hpp"

#include "config/params.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <limits.h>

namespace {
struct Metrics {
  const char* name;
  uint32_t target_ms;
  uint32_t samples;
  uint32_t min_ms;
  uint32_t max_ms;
  uint64_t sum_ms;
  TickType_t prev_tick;
  bool prev_valid;
};

Metrics g_phy   {"PHY",   PHY_PERIOD_MS,   0, UINT32_MAX, 0, 0, 0, false};
Metrics g_ctrl  {"CTRL",  CTRL_PERIOD_MS,  0, UINT32_MAX, 0, 0, 0, false};
Metrics g_radio {"RADIO", RADIO_PERIOD_MS, 0, UINT32_MAX, 0, 0, 0, false};
Metrics g_tele  {"TELEM", TELE_PERIOD_MS,  0, UINT32_MAX, 0, 0, 0, false};

constexpr uint32_t LOG_EVERY = 0;  // Disabled - set to 0 to disable metrics logging (watchdog issues)

TickType_t tick_diff(TickType_t now, TickType_t before)
{
  if (now >= before) {
    return now - before;
  }
  return (TickType_t)((UINT32_MAX - before) + now + 1);
}

void record(Metrics& m, TickType_t start, TickType_t /*end*/)
{
  if (!m.prev_valid) {
    m.prev_tick = start;
    m.prev_valid = true;
    return;
  }

  TickType_t dt_ticks = tick_diff(start, m.prev_tick);
  m.prev_tick = start;

  uint32_t dt_ms = dt_ticks * portTICK_PERIOD_MS;
  m.samples++;
  m.sum_ms += dt_ms;
  if (dt_ms < m.min_ms) m.min_ms = dt_ms;
  if (dt_ms > m.max_ms) m.max_ms = dt_ms;

  // Only log if LOG_EVERY > 0 (avoid division by zero when disabled)
  if (LOG_EVERY > 0 && (m.samples % LOG_EVERY == 0)) {
    uint32_t avg = (m.samples > 0) ? (uint32_t)(m.sum_ms / m.samples) : 0;
    // Short log message to minimize UART blocking time (watchdog issue)
    ESP_LOGI("metrics", "%s: tgt=%u min=%u avg=%u max=%u n=%u",
             m.name, m.target_ms, m.min_ms, avg, m.max_ms, m.samples);
    // Yield after logging to let idle task reset watchdog (UART logging is slow)
    taskYIELD();
  }
}
} // namespace

void metrics_record_phy(TickType_t start, TickType_t end)   { record(g_phy, start, end); }
void metrics_record_ctrl(TickType_t start, TickType_t end)  { record(g_ctrl, start, end); }
void metrics_record_radio(TickType_t start, TickType_t end) { record(g_radio, start, end); }
void metrics_record_tele(TickType_t start, TickType_t end)  { record(g_tele, start, end); }

