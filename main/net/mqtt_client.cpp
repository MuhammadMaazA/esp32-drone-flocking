#include "net/mqtt_client.hpp"
#include "config/params.hpp"
#include "config/team_key.hpp"
#include "core/packet.hpp"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "esp_event.h"
#include <cJSON.h>
#include <stdio.h>
#include <string.h>

static const char* TAG = "mqtt";
static esp_mqtt_client_handle_t client = nullptr;
static bool connected = false;
static bool mac_ready = false;
static uint8_t node_mac[6]{};
static int heartbeat_counter = 0;

static void ensure_node_mac()
{
  if (!mac_ready) {
    esp_read_mac(node_mac, ESP_MAC_WIFI_STA);
    mac_ready = true;
  }
}

static void mqtt_event_handler(void*, esp_event_base_t, int32_t event_id, void* event_data) {
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  
  switch ((esp_mqtt_event_id_t) event_id) {
    case MQTT_EVENT_CONNECTED:
      connected = true;
      ESP_LOGI(TAG, "MQTT connected");
      // Subscribe to test topic
      esp_mqtt_client_subscribe(client, MQTT_TOPIC_TEST, 1);
      ESP_LOGI(TAG, "Subscribed to topic: %s", MQTT_TOPIC_TEST);
      break;
      
    case MQTT_EVENT_DATA:
      ESP_LOGI(TAG, "MQTT message on topic: %.*s", event->topic_len, event->topic);
      ESP_LOGI(TAG, "Payload: %.*s", event->data_len, event->data);
      break;
      
    case MQTT_EVENT_DISCONNECTED:
      connected = false;
      ESP_LOGW(TAG, "MQTT disconnected");
      break;
      
    default:
      break;
  }
}

bool mqtt_init() {
  ensure_node_mac();
  esp_mqtt_client_config_t cfg = {};
  cfg.broker.address.uri = MQTT_BROKER_URI;
  client = esp_mqtt_client_init(&cfg);
  esp_mqtt_client_register_event(client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID,
                                 mqtt_event_handler, nullptr);
  esp_mqtt_client_start(client);

  uint32_t start = esp_timer_get_time() / 1000ULL;
  while (!connected && ((esp_timer_get_time() / 1000ULL) - start) < 5000) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  return connected;
}

bool mqtt_publish_state(const State& s, const char* node_label) {
  if (!client || !connected) return false;
  (void)node_label; // legacy parameter, no longer published

  ensure_node_mac();
  WireHeader wh{};
  wh.version    = PROTO_VERSION;
  wh.team_id    = TEAM_ID;
  memcpy(wh.node_id, node_mac, sizeof(node_mac));
  wh.seq_number = s.seq;
  wh.ts_s       = s.ts_s;
  wh.ts_ms      = s.ts_ms;
  // Convert signed positions to unsigned (ensure non-negative)
  wh.x_mm       = (uint32_t)(s.x_mm >= 0 ? s.x_mm : 0);
  wh.y_mm       = (uint32_t)(s.y_mm >= 0 ? s.y_mm : 0);
  wh.z_mm       = (uint32_t)(s.z_mm >= 0 ? s.z_mm : 0);
  wh.vx_mm_s    = s.vx_mms;
  wh.vy_mm_s    = s.vy_mms;
  wh.vz_mm_s    = s.vz_mms;
  wh.yaw_cd     = s.heading_cd;  // Already uint16_t

  uint8_t wire[WIRE_TOTAL_LEN];
  const uint8_t* key = get_team_key();
  if (packet_encode(wh, wire, key) != WIRE_TOTAL_LEN) {
    ESP_LOGW(TAG, "Failed to compute CMAC for telemetry");
    return false;
  }
  char mac_tag_hex[9];
  snprintf(mac_tag_hex, sizeof(mac_tag_hex), "%02X%02X%02X%02X",
           wire[WIRE_NO_TAG_LEN + 0],
           wire[WIRE_NO_TAG_LEN + 1],
           wire[WIRE_NO_TAG_LEN + 2],
           wire[WIRE_NO_TAG_LEN + 3]);

  cJSON* root = cJSON_CreateObject();
  cJSON_AddNumberToObject(root, "version", wh.version);
  cJSON_AddNumberToObject(root, "team_id", wh.team_id);

  cJSON* arr = cJSON_CreateArray();
  for (size_t i = 0; i < sizeof(node_mac); ++i) {
    cJSON_AddItemToArray(arr, cJSON_CreateNumber(node_mac[i]));
  }
  cJSON_AddItemToObject(root, "node_id", arr);

  cJSON_AddNumberToObject(root, "seq_number", wh.seq_number);
  cJSON_AddNumberToObject(root, "ts_s", wh.ts_s);
  cJSON_AddNumberToObject(root, "ts_ms", wh.ts_ms);
  cJSON_AddNumberToObject(root, "x_mm", wh.x_mm);
  cJSON_AddNumberToObject(root, "y_mm", wh.y_mm);
  cJSON_AddNumberToObject(root, "z_mm", wh.z_mm);
  cJSON_AddNumberToObject(root, "vx_mm_s", wh.vx_mm_s);
  cJSON_AddNumberToObject(root, "vy_mm_s", wh.vy_mm_s);
  cJSON_AddNumberToObject(root, "vz_mm_s", wh.vz_mm_s);
  cJSON_AddNumberToObject(root, "yaw_cd", wh.yaw_cd);
  cJSON_AddStringToObject(root, "mac_tag", mac_tag_hex);

  char* json = cJSON_PrintUnformatted(root);
  esp_mqtt_client_publish(client, MQTT_TOPIC_STATE, json, 0, 0, 0);
  cJSON_Delete(root);
  free(json);
  return true;
}

bool mqtt_publish_heartbeat() {
  if (!client || !connected) return false;
  
  char msg[64];
  snprintf(msg, sizeof(msg), "Hello from ESP32, count=%d", heartbeat_counter++);
  
  int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC_HEARTBEAT, msg, 0, 1, 0);
  if (msg_id >= 0) {
    ESP_LOGI(TAG, "Published heartbeat: %s (msg_id=%d)", msg, msg_id);
    return true;
  } else {
    ESP_LOGW(TAG, "Failed to publish heartbeat");
    return false;
  }
}

bool mqtt_is_connected() {
  return connected && client != nullptr;
}
