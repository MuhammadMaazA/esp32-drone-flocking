/*
 * Drone Flocking Coursework - COMP0221
 * Real-Time Operating System implementation with Physics, Flocking, Radio, and Telemetry tasks
 */

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_eap_client.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "lwip/inet.h"
#include "mqtt_client.h"
#include "esp_sntp.h"
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>

// Include our custom modules
#include "config.h"
#include "packet.h"
#include "physics.h"
#include "security.h"
#include "neighbor.h"
#include "performance.h"
#include "lora_radio.h"

static const char *TAG = "drone_flocking";

// --- USER SETTINGS (UCL-style) ---
// IMPORTANT: Replace these with your actual credentials before building
#define EDUROAM_SSID     "eduroam"
#define EDUROAM_IDENTITY "your-email@ucl.ac.uk"
#define EDUROAM_USERNAME "your-email@ucl.ac.uk"
#define EDUROAM_PASSWORD "your-password"
// ---------------------------------

// MQTT broker (moved to config.h)

static EventGroupHandle_t s_ev;
#define WIFI_CONNECTED_BIT BIT0

// --- GLOBAL DRONE STATE ---
static DroneState drone_state = {0};
static SemaphoreHandle_t state_mutex = NULL;

// --- NEIGHBOR TABLE ---
static NeighborTable neighbor_table;
static SemaphoreHandle_t neighbor_mutex = NULL;

// --- RADIO OUTPUT QUEUE ---
static QueueHandle_t radio_out_queue = NULL;

// --- RADIO INPUT QUEUE (for received packets) ---
static QueueHandle_t radio_in_queue = NULL;

// --- FLOCKING CONFIGURATION ---
static FlockingWeights flocking_weights = {
    .cohesion_weight = COHESION_WEIGHT,
    .alignment_weight = ALIGNMENT_WEIGHT,
    .separation_weight = SEPARATION_WEIGHT
};

// --- PERFORMANCE MONITORING ---
static TaskPerformance perf_physics;
static TaskPerformance perf_flocking;
static TaskPerformance perf_radio;
static TaskPerformance perf_telemetry;

// ----------------------------------------------------------
// Wi-Fi event handler
// ----------------------------------------------------------
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Disconnected - retrying");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_ev, WIFI_CONNECTED_BIT);
    }
}

// ----------------------------------------------------------
// Connect to eduroam
// ----------------------------------------------------------
static void wifi_connect_eduroam(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t cfg = {0};
    strncpy((char *)cfg.sta.ssid, EDUROAM_SSID, sizeof(cfg.sta.ssid));
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_ENTERPRISE;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));

    // PEAP / MSCHAPv2 credentials
    ESP_ERROR_CHECK(esp_eap_client_set_identity((const uint8_t *)EDUROAM_IDENTITY, strlen(EDUROAM_IDENTITY)));
    ESP_ERROR_CHECK(esp_eap_client_set_username((const uint8_t *)EDUROAM_USERNAME, strlen(EDUROAM_USERNAME)));
    ESP_ERROR_CHECK(esp_eap_client_set_password((const uint8_t *)EDUROAM_PASSWORD, strlen(EDUROAM_PASSWORD)));

    ESP_ERROR_CHECK(esp_wifi_sta_enterprise_enable());
    ESP_ERROR_CHECK(esp_wifi_start());

    s_ev = xEventGroupCreate();
    EventBits_t bits = xEventGroupWaitBits(s_ev, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "Timed out waiting for IP");
    }
}

// ----------------------------------------------------------
// MQTT event handler
// ----------------------------------------------------------
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;

    switch (event->event_id) {

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        // Subscribe to a test topic
        esp_mqtt_client_subscribe(client, "COMP0221/test", 1);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT message on topic: %.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "Payload: %.*s", event->data_len, event->data);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;

    default:
        break;
    }
    return ESP_OK;
}

// Wrapper required by esp_mqtt_client_register_event
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    mqtt_event_handler_cb(event_data);
}

// ----------------------------------------------------------
// Start MQTT client
// ----------------------------------------------------------
static esp_mqtt_client_handle_t start_mqtt(void)
{
    const esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    return client;
}

// ----------------------------------------------------------
// Initialize NTP time synchronization
// ----------------------------------------------------------
static void init_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, NTP_SERVER);
    esp_sntp_init();
    
    // Wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    
    while (timeinfo.tm_year < (2024 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    
    if (retry >= retry_count) {
        ESP_LOGW(TAG, "Failed to sync time with NTP server");
    } else {
        ESP_LOGI(TAG, "Time synchronized with NTP");
    }
}

// ----------------------------------------------------------
// Get current Unix timestamp with milliseconds
// ----------------------------------------------------------
static void get_timestamp(uint32_t *ts_s, uint16_t *ts_ms)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    *ts_s = (uint32_t)tv.tv_sec;
    *ts_ms = (uint16_t)(tv.tv_usec / 1000);
}

// ----------------------------------------------------------
// Initialize drone with random position and velocity
// ----------------------------------------------------------
static void init_drone_state(void)
{
    // Seed random number generator with MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    uint32_t seed = mac[0] | (mac[1] << 8) | (mac[2] << 16) | (mac[3] << 24);
    srand(seed);
    
    // Random position in center 50m cube (25m-75m range)
    drone_state.x = INITIAL_POS_MIN_MM + (rand() % (INITIAL_POS_MAX_MM - INITIAL_POS_MIN_MM));
    drone_state.y = INITIAL_POS_MIN_MM + (rand() % (INITIAL_POS_MAX_MM - INITIAL_POS_MIN_MM));
    drone_state.z = INITIAL_POS_MIN_MM + (rand() % (INITIAL_POS_MAX_MM - INITIAL_POS_MIN_MM));
    
    // Small random initial velocity (-1 to 1 m/s)
    drone_state.vx = INITIAL_VEL_MIN_MM_S + (rand() % (INITIAL_VEL_MAX_MM_S - INITIAL_VEL_MIN_MM_S));
    drone_state.vy = INITIAL_VEL_MIN_MM_S + (rand() % (INITIAL_VEL_MAX_MM_S - INITIAL_VEL_MIN_MM_S));
    drone_state.vz = INITIAL_VEL_MIN_MM_S + (rand() % (INITIAL_VEL_MAX_MM_S - INITIAL_VEL_MIN_MM_S));
    
    // Random initial heading
    drone_state.yaw = rand() % 36000;  // 0-359.99 degrees
    
    ESP_LOGI(TAG, "Drone initialized at position (%d, %d, %d) mm, velocity (%d, %d, %d) mm/s",
             drone_state.x, drone_state.y, drone_state.z,
             drone_state.vx, drone_state.vy, drone_state.vz);
}

// ----------------------------------------------------------
// TASK: Physics (50 Hz, High Priority)
// ----------------------------------------------------------
static void physics_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PHYSICS_PERIOD_MS);
    
    ESP_LOGI(TAG, "Physics task started at %d Hz", PHYSICS_FREQ_HZ);
    
    while (1) {
        uint64_t start_time = perf_start(&perf_physics);
        
        // Update physics (position based on velocity)
        if (xSemaphoreTake(state_mutex, portMAX_DELAY)) {
            update_physics(&drone_state, PHYSICS_DT);
            
            // Make a copy for the queue
            DroneState state_copy = drone_state;
            xSemaphoreGive(state_mutex);
            
            // Send state to radio queue (non-blocking)
            xQueueSend(radio_out_queue, &state_copy, 0);
        }
        
        perf_end(&perf_physics, start_time);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ----------------------------------------------------------
// TASK: Flocking (10 Hz, Medium Priority)
// ----------------------------------------------------------
static void flocking_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(FLOCKING_PERIOD_MS);
    
    ESP_LOGI(TAG, "Flocking task started at %d Hz", FLOCKING_FREQ_HZ);
    
    // Temporary arrays for neighbor data
    Neighbor neighbors_for_flocking[MAX_NEIGHBORS];
    NeighborEntry neighbor_entries[MAX_NEIGHBORS];
    
    while (1) {
        uint64_t start_time = perf_start(&perf_flocking);
        // Process incoming packets from radio_in_queue
        FlockPacket received_packet;
        while (xQueueReceive(radio_in_queue, &received_packet, 0) == pdTRUE) {
            uint32_t current_time_ms = esp_timer_get_time() / 1000;
            
            // Update neighbor table
            if (xSemaphoreTake(neighbor_mutex, pdMS_TO_TICKS(10))) {
                neighbor_table_update(&neighbor_table, &received_packet, current_time_ms);
                xSemaphoreGive(neighbor_mutex);
            }
        }
        
        // Prune stale neighbors
        uint32_t current_time_ms = esp_timer_get_time() / 1000;
        if (xSemaphoreTake(neighbor_mutex, pdMS_TO_TICKS(10))) {
            neighbor_table_prune_stale(&neighbor_table, current_time_ms, NEIGHBOR_TIMEOUT_MS);
            
            // Get all neighbors for flocking calculation
            int count = neighbor_table_get_all(&neighbor_table, neighbor_entries, MAX_NEIGHBORS);
            
            // Convert to Neighbor format for physics module
            for (int i = 0; i < count; i++) {
                neighbors_for_flocking[i].x = neighbor_entries[i].x_mm;
                neighbors_for_flocking[i].y = neighbor_entries[i].y_mm;
                neighbors_for_flocking[i].z = neighbor_entries[i].z_mm;
                neighbors_for_flocking[i].vx = neighbor_entries[i].vx_mm_s;
                neighbors_for_flocking[i].vy = neighbor_entries[i].vy_mm_s;
                neighbors_for_flocking[i].vz = neighbor_entries[i].vz_mm_s;
            }
            
            xSemaphoreGive(neighbor_mutex);
            
            // Calculate flocking velocity
            if (count > 0) {
                if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10))) {
                    Vector3D flocking_velocity = calculate_flocking(drone_state, neighbors_for_flocking, count, &flocking_weights);
                    
                    // Apply flocking velocity (blend with current velocity for smooth motion)
                    drone_state.vx = (int32_t)((1.0f - FLOCKING_BLEND_FACTOR) * drone_state.vx + FLOCKING_BLEND_FACTOR * flocking_velocity.x);
                    drone_state.vy = (int32_t)((1.0f - FLOCKING_BLEND_FACTOR) * drone_state.vy + FLOCKING_BLEND_FACTOR * flocking_velocity.y);
                    drone_state.vz = (int32_t)((1.0f - FLOCKING_BLEND_FACTOR) * drone_state.vz + FLOCKING_BLEND_FACTOR * flocking_velocity.z);
                    
                    xSemaphoreGive(state_mutex);
                }
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ----------------------------------------------------------
// LoRa RX Callback - called when packet is received
// ----------------------------------------------------------
static void lora_rx_handler(const FlockPacket *packet, int rssi, float snr)
{
    if (packet == NULL) {
        return;
    }

    // Get our MAC to avoid processing our own packets
    uint8_t my_mac[6];
    esp_read_mac(my_mac, ESP_MAC_WIFI_STA);
    
    // Ignore our own packets
    if (memcmp(packet->node_id, my_mac, 6) == 0) {
        return;
    }

    // Verify team ID (0 means accept all)
    if (packet->team_id != 0 && packet->team_id != TEAM_ID) {
        ESP_LOGD(TAG, "RX: Ignoring packet from different team (team_id=%u)", packet->team_id);
        return;
    }

    ESP_LOGI(TAG, "RX: seq=%u from %02X:%02X:%02X:%02X:%02X:%02X RSSI=%d SNR=%.1f",
             packet->seq, 
             packet->node_id[0], packet->node_id[1], packet->node_id[2],
             packet->node_id[3], packet->node_id[4], packet->node_id[5],
             rssi, snr);

    // Send to radio_in_queue for processing by flocking task
    xQueueSend(radio_in_queue, packet, 0);
}

// ----------------------------------------------------------
// TASK: Radio (5 Hz, Low Priority)
// ----------------------------------------------------------
static void radio_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(RADIO_PERIOD_MS);
    
    // AES-128 key for packet signing (shared among team drones)
    uint8_t aes_key[16] = AES_KEY_BYTES;
    
    // Get our MAC address
    uint8_t my_mac[6];
    esp_read_mac(my_mac, ESP_MAC_WIFI_STA);
    
    uint16_t sequence_number = 0;
    
    ESP_LOGI(TAG, "Radio task started at %d Hz", RADIO_FREQ_HZ);
    ESP_LOGI(TAG, "Node MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);
    
    while (1) {
        uint64_t start_time = perf_start(&perf_radio);
        
        DroneState state_to_send;
        
        // Receive state from queue (non-blocking)
        if (xQueueReceive(radio_out_queue, &state_to_send, 0) == pdTRUE) {
            // Create and populate packet
            FlockPacket packet = {0};
            packet.version = PROTOCOL_VERSION;
            packet.flags = 0;
            packet.team_id = TEAM_ID;
            
            // Node ID (MAC address)
            memcpy(packet.node_id, my_mac, 6);
            
            packet.seq = sequence_number++;
            
            // Timestamp (Unix time with NTP synchronization)
            get_timestamp(&packet.ts_s, &packet.ts_ms);
            
            // State data
            packet.x_mm = state_to_send.x;
            packet.y_mm = state_to_send.y;
            packet.z_mm = state_to_send.z;
            packet.vx = state_to_send.vx;
            packet.vy = state_to_send.vy;
            packet.vz = state_to_send.vz;
            packet.heading = state_to_send.yaw;
            
            // Serialize packet (Big-Endian)
            uint8_t buffer[64];
            int len = serialize_packet(&packet, buffer, sizeof(buffer));
            
            if (len > 0) {
                // Sign the packet (AES-128-CMAC, excluding the MAC tag itself)
                sign_packet(buffer, len - 4, aes_key, packet.mac_tag);
                
                // Copy the tag back into the buffer
                memcpy(&buffer[len - 4], packet.mac_tag, 4);
                
                // Transmit via LoRa radio
                if (lora_transmit(buffer, len)) {
                    ESP_LOGI(TAG, "TX: seq=%u pos=(%d,%d,%d) vel=(%d,%d,%d) heading=%u",
                             packet.seq, state_to_send.x, state_to_send.y, state_to_send.z,
                             state_to_send.vx, state_to_send.vy, state_to_send.vz, state_to_send.yaw);
                } else {
                    ESP_LOGW(TAG, "TX: Failed to transmit packet");
                }
            }
        }
        
        // Note: LoRa packet reception is handled by lora_rx_handler callback
        // which is called asynchronously when packets arrive
        
        perf_end(&perf_radio, start_time);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ----------------------------------------------------------
// TASK: Telemetry (2 Hz, Low Priority)
// ----------------------------------------------------------
static void telemetry_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);
    
    esp_mqtt_client_handle_t *client = (esp_mqtt_client_handle_t *)pvParameters;
    
    // Get our MAC address for telemetry
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    
    ESP_LOGI(TAG, "Telemetry task started at %d Hz", TELEMETRY_FREQ_HZ);
    
    while (1) {
        uint64_t start_time = perf_start(&perf_telemetry);
        
        // Lock mutex to read state
        if (xSemaphoreTake(state_mutex, portMAX_DELAY)) {
            DroneState state_copy = drone_state;
            xSemaphoreGive(state_mutex);
            
            // Get timestamp
            uint32_t ts_s;
            uint16_t ts_ms;
            get_timestamp(&ts_s, &ts_ms);
            
            // Get neighbor count
            int neighbor_count = 0;
            if (xSemaphoreTake(neighbor_mutex, pdMS_TO_TICKS(10))) {
                neighbor_count = neighbor_table_get_count(&neighbor_table);
                xSemaphoreGive(neighbor_mutex);
            }
            
            // Build JSON payload according to specification
            // Using all fields from the LoRa packet format
            char json_payload[512];
            snprintf(json_payload, sizeof(json_payload),
                     "{\"version\":%d,"
                     "\"flags\":0,"
                     "\"team_id\":%d,"
                     "\"node_id\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
                     "\"seq\":0,"
                     "\"ts_s\":%u,"
                     "\"ts_ms\":%u,"
                     "\"x_mm\":%d,"
                     "\"y_mm\":%d,"
                     "\"z_mm\":%d,"
                     "\"vx_mms\":%d,"
                     "\"vy_mms\":%d,"
                     "\"vz_mms\":%d,"
                     "\"heading_cd\":%u,"
                     "\"neighbors\":%d}",
                     PROTOCOL_VERSION,
                     TEAM_ID,
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                     ts_s, ts_ms,
                     state_copy.x, state_copy.y, state_copy.z,
                     state_copy.vx, state_copy.vy, state_copy.vz,
                     state_copy.yaw,
                     neighbor_count);
            
            // Publish to the correct topic: COMP0221/flock/0/state
            int msg_id = esp_mqtt_client_publish(*client, MQTT_TOPIC, json_payload, 0, MQTT_QOS, MQTT_RETAIN);
            
            if (msg_id >= 0) {
                ESP_LOGI(TAG, "MQTT published: pos=(%d,%d,%d) neighbors=%d",
                         state_copy.x, state_copy.y, state_copy.z, neighbor_count);
            } else {
                ESP_LOGW(TAG, "MQTT publish failed");
            }
        }
        
        perf_end(&perf_telemetry, start_time);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ----------------------------------------------------------
// TASK: Performance Monitoring (periodic logging)
// ----------------------------------------------------------
static void performance_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Performance monitoring task started");
    
    while (1) {
        // Wait for logging interval
        vTaskDelay(pdMS_TO_TICKS(PERF_LOG_INTERVAL_MS));
        
        // Print all task performance statistics
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "   PERFORMANCE STATISTICS REPORT");
        ESP_LOGI(TAG, "========================================");
        perf_print(&perf_physics);
        perf_print(&perf_flocking);
        perf_print(&perf_radio);
        perf_print(&perf_telemetry);
        
        // Print neighbor table statistics
        if (xSemaphoreTake(neighbor_mutex, pdMS_TO_TICKS(10))) {
            ESP_LOGI(TAG, "=== Neighbor Statistics ===");
            ESP_LOGI(TAG, "  Active neighbors: %d", neighbor_table_get_count(&neighbor_table));
            ESP_LOGI(TAG, "  Total packets received: %u", neighbor_table.total_received);
            ESP_LOGI(TAG, "  Total packets dropped: %u", neighbor_table.total_dropped);
            xSemaphoreGive(neighbor_mutex);
        }
        
        // Print LoRa radio statistics
        uint32_t tx_count, rx_count, tx_errors;
        lora_get_stats(&tx_count, &rx_count, &tx_errors);
        ESP_LOGI(TAG, "=== LoRa Radio Statistics ===");
        ESP_LOGI(TAG, "  Packets transmitted: %u", tx_count);
        ESP_LOGI(TAG, "  Packets received: %u", rx_count);
        ESP_LOGI(TAG, "  Transmission errors: %u", tx_errors);
        
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "");
    }
}

// ----------------------------------------------------------
// app_main: Initialize system and create tasks
// ----------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "=== Drone Flocking System - COMP0221 ===");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Connect to eduroam WiFi
    wifi_connect_eduroam();

    // Initialize NTP for time synchronization
    init_sntp();

    // Start MQTT client
    static esp_mqtt_client_handle_t client;
    client = start_mqtt();
    
    // Wait a moment for MQTT to connect
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Initialize LoRa radio
    if (!lora_init()) {
        ESP_LOGW(TAG, "LoRa initialization failed - continuing without radio");
    } else {
        lora_set_rx_callback(lora_rx_handler);
        ESP_LOGI(TAG, "LoRa radio initialized successfully");
    }

    // Initialize neighbor table
    neighbor_table_init(&neighbor_table);

    // Initialize drone with random position and velocity
    init_drone_state();

    // Initialize performance monitoring for each task
    perf_init(&perf_physics, "Physics", PHYSICS_PERIOD_MS * 1000);    // Convert ms to us
    perf_init(&perf_flocking, "Flocking", FLOCKING_PERIOD_MS * 1000);
    perf_init(&perf_radio, "Radio", RADIO_PERIOD_MS * 1000);
    perf_init(&perf_telemetry, "Telemetry", TELEMETRY_PERIOD_MS * 1000);

    // Create synchronization primitives
    state_mutex = xSemaphoreCreateMutex();
    if (state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return;
    }

    neighbor_mutex = xSemaphoreCreateMutex();
    if (neighbor_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create neighbor mutex");
        return;
    }

    // Create radio output queue
    radio_out_queue = xQueueCreate(RADIO_QUEUE_SIZE, sizeof(DroneState));
    if (radio_out_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create radio output queue");
        return;
    }

    // Create radio input queue (for received packets)
    radio_in_queue = xQueueCreate(NEIGHBOR_QUEUE_SIZE, sizeof(FlockPacket));
    if (radio_in_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create radio input queue");
        return;
    }

    ESP_LOGI(TAG, "Creating RTOS tasks...");

    // Create tasks with configured priorities and stack sizes
    // Physics: 50Hz, Highest Priority
    xTaskCreatePinnedToCore(
        physics_task,
        "physics",
        STACK_SIZE_PHYSICS,
        NULL,
        PRIORITY_PHYSICS,
        NULL,
        1  // Core 1
    );

    // Flocking: 10Hz, Medium-High Priority
    xTaskCreatePinnedToCore(
        flocking_task,
        "flocking",
        STACK_SIZE_FLOCKING,
        NULL,
        PRIORITY_FLOCKING,
        NULL,
        1  // Core 1
    );

    // Radio: 5Hz, Medium Priority
    xTaskCreatePinnedToCore(
        radio_task,
        "radio",
        STACK_SIZE_RADIO,
        NULL,
        PRIORITY_RADIO,
        NULL,
        0  // Core 0
    );

    // Telemetry: 2Hz, Low Priority
    xTaskCreatePinnedToCore(
        telemetry_task,
        "telemetry",
        STACK_SIZE_TELEMETRY,
        (void *)&client,
        PRIORITY_TELEMETRY,
        NULL,
        0  // Core 0
    );

    // Performance monitoring task (optional, only if enabled)
    #if ENABLE_PERFORMANCE_LOGGING
    xTaskCreatePinnedToCore(
        performance_task,
        "performance",
        4096,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        0  // Core 0
    );
    #endif

    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "System initialization complete");
    ESP_LOGI(TAG, "Task frequencies: Physics=%dHz, Flocking=%dHz, Radio=%dHz, Telemetry=%dHz",
             PHYSICS_FREQ_HZ, FLOCKING_FREQ_HZ, RADIO_FREQ_HZ, TELEMETRY_FREQ_HZ);
}
