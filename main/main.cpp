// main.cpp — boot, Wi-Fi connect, queue creation, task starts

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include "esp_http_client.h"

#include <time.h>
#include <cstring>
#include <cstdlib>
#include <sys/time.h>

#include "globals.hpp"
#include "config/params.hpp"

// Tasks
#include "tasks/task_physics.hpp"
#include "tasks/task_control.hpp"
#include "tasks/task_radio.hpp"
#include "tasks/task_telemetry.hpp"

// ---- Queue definitions (one TU only) ----
QueueHandle_t q_state_for_ctrl = nullptr;
QueueHandle_t q_state_for_tx   = nullptr;
QueueHandle_t q_cmd_vel        = nullptr;
QueueHandle_t q_neighbors      = nullptr;

static const char* TAG = "app";

static EventGroupHandle_t s_wifi_event_group = nullptr;
static constexpr EventBits_t WIFI_CONNECTED_BIT = BIT0;
static bool s_logged_wifi_scan = false;

// ---------- Minimal Wi-Fi STA helper ----------
// Prefer setting these in menuconfig (Kconfig):
//  - CONFIG_WIFI_SSID
//  - CONFIG_WIFI_PASSWORD
#ifndef WIFI_SSID
#define WIFI_SSID CONFIG_WIFI_SSID
#endif
#ifndef WIFI_PASS
#define WIFI_PASS CONFIG_WIFI_PASSWORD
#endif

#include "esp_wifi.h"
#include "esp_eap_client.h"
#include "sdkconfig.h"

static void log_wifi_scan_results()
{
  wifi_scan_config_t scan_cfg = {};
  scan_cfg.show_hidden = true;

  ESP_LOGI(TAG, "Scanning for nearby access points...");
  esp_err_t scan_rc = esp_wifi_scan_start(&scan_cfg, true);
  if (scan_rc != ESP_OK) {
    ESP_LOGW(TAG, "Wi-Fi scan failed: %s", esp_err_to_name(scan_rc));
    return;
  }

  uint16_t ap_count = 0;
  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
  if (ap_count == 0) {
    ESP_LOGW(TAG, "Wi-Fi scan complete: no networks found");
    return;
  }

  constexpr uint16_t kMaxReport = 20;
  uint16_t record_count = ap_count;
  if (record_count > kMaxReport) {
    record_count = kMaxReport;
  }

  // Allocate on heap to avoid stack overflow (20 records = ~4KB)
  wifi_ap_record_t* ap_records = static_cast<wifi_ap_record_t*>(
      malloc(sizeof(wifi_ap_record_t) * record_count));
  if (!ap_records) {
    ESP_LOGW(TAG, "Failed to allocate scan results buffer");
    return;
  }
  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&record_count, ap_records));

  ESP_LOGI(TAG, "Found %u access point%s (showing %u):",
           ap_count, ap_count == 1 ? "" : "s", record_count);
  for (uint16_t i = 0; i < record_count; ++i) {
    const wifi_ap_record_t& ap = ap_records[i];
    ESP_LOGI(TAG,
             "  %2u: SSID=\"%s\" RSSI=%d dBm channel=%d auth=%d",
             static_cast<unsigned>(i + 1),
             reinterpret_cast<const char*>(ap.ssid),
             ap.rssi,
             ap.primary,
             ap.authmode);
  }

  if (ap_count > record_count) {
    ESP_LOGI(TAG, "  ... plus %u more", ap_count - record_count);
  }

  free(ap_records);
}

static void wifi_event_handler(void*, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    auto* event = static_cast<wifi_event_sta_disconnected_t*>(event_data);
    if (event) {
      ESP_LOGW(TAG, "Wi-Fi disconnected; reason=%d", event->reason);
    } else {
      ESP_LOGW(TAG, "Wi-Fi disconnected; reason=unknown");
    }
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    auto* event = static_cast<ip_event_got_ip_t*>(event_data);
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    if (s_wifi_event_group) {
      xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
  }
}

static void wifi_start_and_connect()
{
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  s_wifi_event_group = xEventGroupCreate();
  esp_event_handler_instance_t wifi_instance = nullptr;
  esp_event_handler_instance_t ip_instance = nullptr;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      nullptr,
                                                      &wifi_instance));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &wifi_event_handler,
                                                      nullptr,
                                                      &ip_instance));

  wifi_config_t wcfg = {};
  
#ifdef CONFIG_WIFI_MODE_EDUROAM
  // eduroam WPA2 Enterprise configuration
  ESP_LOGI(TAG, "Configuring eduroam (WPA2 Enterprise)");
  strncpy((char*)wcfg.sta.ssid, "eduroam", sizeof(wcfg.sta.ssid) - 1);
  wcfg.sta.ssid[sizeof(wcfg.sta.ssid) - 1] = '\0';
  wcfg.sta.threshold.authmode = WIFI_AUTH_WPA2_ENTERPRISE;
  
  // Set eduroam credentials
  ESP_ERROR_CHECK(esp_eap_client_set_identity(
      (const uint8_t*)CONFIG_EDUROAM_IDENTITY, strlen(CONFIG_EDUROAM_IDENTITY)));
  ESP_ERROR_CHECK(esp_eap_client_set_username(
      (const uint8_t*)CONFIG_EDUROAM_USERNAME, strlen(CONFIG_EDUROAM_USERNAME)));
  ESP_ERROR_CHECK(esp_eap_client_set_password(
      (const uint8_t*)CONFIG_EDUROAM_PASSWORD, strlen(CONFIG_EDUROAM_PASSWORD)));
  
  ESP_ERROR_CHECK(esp_wifi_sta_enterprise_enable());
#else
  // Regular WPA2 configuration
  ESP_LOGI(TAG, "Configuring regular WPA2");
  snprintf((char*)wcfg.sta.ssid,    sizeof(wcfg.sta.ssid),    "%s", WIFI_SSID);
  snprintf((char*)wcfg.sta.password,sizeof(wcfg.sta.password),"%s", WIFI_PASS);
  wcfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
#endif

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
  ESP_ERROR_CHECK(esp_wifi_start());

  if (!s_logged_wifi_scan) {
    log_wifi_scan_results();
    s_logged_wifi_scan = true;
  }

#ifdef CONFIG_WIFI_MODE_EDUROAM
  ESP_LOGI(TAG, "Wi-Fi connecting to eduroam (WPA2 Enterprise) ...");
  ESP_LOGI(TAG, "  Identity: %s", CONFIG_EDUROAM_IDENTITY);
  ESP_LOGI(TAG, "  Username: %s", CONFIG_EDUROAM_USERNAME);
#else
  ESP_LOGI(TAG, "Wi-Fi connecting to SSID=\"%s\" ...", WIFI_SSID);
#endif
  EventBits_t bits = xEventGroupWaitBits(
      s_wifi_event_group,
      WIFI_CONNECTED_BIT,
      pdTRUE,
      pdFALSE,
      pdMS_TO_TICKS(15000));

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "Wi-Fi init done (proceeding; telemetry will retry if needed)");
  } else {
    ESP_LOGW(TAG, "Wi-Fi connect timeout; continuing but network may be unavailable");
  }

  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        ip_instance));
  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        wifi_instance));
  vEventGroupDelete(s_wifi_event_group);
  s_wifi_event_group = nullptr;
}

static bool sync_time_via_http()
{
  ESP_LOGW(TAG, "Falling back to HTTP time sync");
  esp_http_client_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.url = "https://worldtimeapi.org/api/timezone/Etc/UTC";
  cfg.timeout_ms = 5000;
  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (!client) {
    ESP_LOGW(TAG, "Failed to init HTTP client for time sync");
    return false;
  }

  esp_err_t err = esp_http_client_open(client, 0);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "HTTP open failed: %s", esp_err_to_name(err));
    esp_http_client_cleanup(client);
    return false;
  }

  char buffer[512] = {0};
  int total_read = 0;
  while (total_read < (int)sizeof(buffer) - 1) {
    int read = esp_http_client_read(client, buffer + total_read,
                                    sizeof(buffer) - 1 - total_read);
    if (read <= 0) {
      break;
    }
    total_read += read;
  }

  esp_http_client_close(client);
  esp_http_client_cleanup(client);

  if (total_read <= 0) {
    ESP_LOGW(TAG, "No data received from HTTP time source");
    return false;
  }

  buffer[total_read] = '\0';
  const char* unixtime = strstr(buffer, "\"unixtime\":");
  if (!unixtime) {
    ESP_LOGW(TAG, "Failed to parse unixtime from HTTP response");
    return false;
  }

  unixtime += strlen("\"unixtime\":");
  long epoch = strtol(unixtime, nullptr, 10);
  if (epoch <= 0) {
    ESP_LOGW(TAG, "Invalid epoch parsed from HTTP response");
    return false;
  }

  struct timeval tv = {
      .tv_sec = epoch,
      .tv_usec = 0,
  };
  settimeofday(&tv, nullptr);

  time_t now = epoch;
  ESP_LOGI(TAG, "HTTP time sync complete: %ld", (long)now);
  return true;
}

static void sync_time_via_sntp()
{
  ESP_LOGI(TAG, "Starting SNTP time sync");
  esp_log_level_set("sntp", ESP_LOG_DEBUG);
  sntp_servermode_dhcp(true);
  esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, "time.cloudflare.com");
  esp_sntp_setservername(1, "time.google.com");
  esp_sntp_setservername(2, "pool.ntp.org");
  esp_sntp_init();

  const int retry_max = 20;
  for (int retry = 0; retry < retry_max; ++retry) {
    time_t now = 0;
    struct tm timeinfo;
    memset(&timeinfo, 0, sizeof(timeinfo));
    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_year > (2016 - 1900)) {
      ESP_LOGI(TAG, "Time sync complete: %ld", (long)now);
      return;
    }
    ESP_LOGI(TAG, "Waiting for SNTP sync... (%d/%d)", retry + 1, retry_max);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  ESP_LOGW(TAG, "SNTP sync timed out; timestamps may be inaccurate");
  if (!sync_time_via_http()) {
    ESP_LOGW(TAG, "HTTP time sync failed; system time may be wrong");
  }
}

// ---------- app_main ----------
extern "C" void app_main(void)
{
  // NVS is required for Wi-Fi
  esp_err_t nvs_rc = nvs_flash_init();
  if (nvs_rc == ESP_ERR_NVS_NO_FREE_PAGES || nvs_rc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set("task_wdt", ESP_LOG_NONE);  // Completely silence watchdog warnings (they're just noise from slow LoRa)
  ESP_LOGI(TAG, "Booting COMP0221 node");

  // Create queues (length 1 overwrite pattern)
  q_state_for_ctrl = xQueueCreate(1, sizeof(State));
  q_state_for_tx   = xQueueCreate(1, sizeof(State));
  q_cmd_vel        = xQueueCreate(1, sizeof(CmdVel));
  q_neighbors      = xQueueCreate(1, sizeof(NeighborsView));

  configASSERT(q_state_for_ctrl && q_state_for_tx && q_cmd_vel && q_neighbors);

  // Bring up Wi-Fi before starting telemetry (MQTT needs TCP/IP)
  wifi_start_and_connect();
  sync_time_via_sntp();

  // Create tasks (pin to core 1 to keep Wi-Fi on core 0 happier)
  const BaseType_t core = 1;

  xTaskCreatePinnedToCore(vTaskPhysics,   "phy",   TASK_PHYSICS_STACK_SIZE,  nullptr, TASK_PHYSICS_PRIORITY,  nullptr, core);
  xTaskCreatePinnedToCore(vTaskControl,   "ctrl",  TASK_FLOCKING_STACK_SIZE, nullptr, TASK_FLOCKING_PRIORITY, nullptr, core);
  xTaskCreatePinnedToCore(vTaskRadio,     "radio", TASK_RADIO_STACK_SIZE,    nullptr, TASK_RADIO_PRIORITY,    nullptr, core);
  xTaskCreatePinnedToCore(vTaskTelemetry, "tele",  TASK_TELEMETRY_STACK_SIZE, nullptr, TASK_TELEMETRY_PRIORITY, nullptr, core);

  ESP_LOGI(TAG, "Tasks started: PHY(%dHz), CTRL(%dHz), RADIO(%dHz), TELE(%dHz)",
           TASK_PHYSICS_FREQ_HZ, TASK_FLOCKING_FREQ_HZ, TASK_RADIO_FREQ_HZ, TASK_TELEMETRY_FREQ_HZ);
}
