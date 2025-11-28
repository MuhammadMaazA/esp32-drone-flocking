#include "task_telemetry.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "globals.hpp"
#include "core/state.hpp"
#include "net/mqtt_client.hpp"
#include "config/params.hpp"
#include "core/metrics.hpp"

static const char* TAG = "task_tele";

extern QueueHandle_t q_state_for_tx;

extern "C" void vTaskTelemetry(void* pv) {
  (void)pv;
  ESP_LOGI(TAG, "Telemetry task start: %d ms period", TELE_PERIOD_MS);

  if (!mqtt_init()) {
    ESP_LOGE(TAG, "MQTT init failed; suspending");
    vTaskDelay(portMAX_DELAY);
  }

  State s{};
  TickType_t last = xTaskGetTickCount();
  TickType_t last_heartbeat = xTaskGetTickCount();
  uint32_t publish_fail_count = 0;  // Counter to reduce log spam

  for (;;) {
    TickType_t t0 = xTaskGetTickCount();
    // latest self state
    if (q_state_for_tx) {
      xQueueReceive(q_state_for_tx, &s, 0);
    }
    if (!mqtt_publish_state(s, NODE_LABEL)) {
      publish_fail_count++;
      // Only log every 10 failures to reduce spam during disconnections
      if (publish_fail_count % 10 == 0) {
        ESP_LOGW(TAG, "publish failed (not connected?) - %u failures", publish_fail_count);
      }
    } else {
      // Reset counter on success
      if (publish_fail_count > 0) {
        publish_fail_count = 0;
      }
    }
    
    // Publish heartbeat every 10 seconds
    TickType_t now = xTaskGetTickCount();
    if ((now - last_heartbeat) >= pdMS_TO_TICKS(MQTT_HEARTBEAT_INTERVAL_MS)) {
      mqtt_publish_heartbeat();
      last_heartbeat = now;
    }
    
    TickType_t t1 = xTaskGetTickCount();
    metrics_record_tele(t0, t1);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(TELE_PERIOD_MS));
  }
}
