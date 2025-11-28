#include "task_radio.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "config/params.hpp"
#include "globals.hpp"
#include "core/state.hpp"
#include "core/packet.hpp"
#include "core/neighbors.hpp"
#include "core/metrics.hpp"
#include "drivers/radio_lora.hpp"
#include "config/team_key.hpp"
#include "core/cmac.hpp"  // For cmac_tag16 in debug logging
#include <RadioLib.h>

#include <string.h>

static const char* TAG = "task_radio";

extern QueueHandle_t q_state_for_tx;   // from Physics
extern QueueHandle_t q_neighbors;      // to Control

static inline uint32_t now_ms(){ return (uint32_t)(esp_timer_get_time()/1000ULL); }

// Replay attack: buffer to store captured packets
struct ReplayPacket {
  uint8_t data[WIRE_TOTAL_LEN];
  uint32_t captured_at_ms;  // When this packet was captured
  bool valid;  // Is this slot used?
};

static ReplayPacket g_replay_buffer[REPLAY_BUFFER_SIZE];
static size_t g_replay_buffer_index = 0;
static size_t g_replay_buffer_count = 0;

// Store a captured packet in the replay buffer
static void store_packet_for_replay(const uint8_t* packet_data, size_t len) {
  if (!ENABLE_REPLAY_ATTACK || len != WIRE_TOTAL_LEN) return;
  
  // Store in circular buffer
  ReplayPacket& slot = g_replay_buffer[g_replay_buffer_index];
  memcpy(slot.data, packet_data, WIRE_TOTAL_LEN);
  slot.captured_at_ms = now_ms();
  slot.valid = true;
  
  g_replay_buffer_index = (g_replay_buffer_index + 1) % REPLAY_BUFFER_SIZE;
  if (g_replay_buffer_count < REPLAY_BUFFER_SIZE) {
    g_replay_buffer_count++;
  }
}

// Replay a captured packet (optionally with modified timestamp)
static bool replay_captured_packet(const uint8_t* team_key) {
  if (!ENABLE_REPLAY_ATTACK || g_replay_buffer_count == 0) return false;
  
  // Find the oldest valid packet
  size_t oldest_idx = 0;
  uint32_t oldest_time = UINT32_MAX;
  for (size_t i = 0; i < REPLAY_BUFFER_SIZE; i++) {
    if (g_replay_buffer[i].valid && g_replay_buffer[i].captured_at_ms < oldest_time) {
      oldest_time = g_replay_buffer[i].captured_at_ms;
      oldest_idx = i;
    }
  }
  
  if (!g_replay_buffer[oldest_idx].valid) return false;
  
  const ReplayPacket& packet = g_replay_buffer[oldest_idx];
  uint8_t replay_frame[WIRE_TOTAL_LEN];
  
  if (REPLAY_MODIFY_TIMESTAMP) {
    // Decode packet, modify timestamp, re-encode (requires key)
    WireHeader h{};
    if (!packet_decode(packet.data, WIRE_TOTAL_LEN, h, team_key)) {
      ESP_LOGW(TAG, "Replay: failed to decode stored packet");
      return false;
    }
    
    uint32_t old_ts = h.ts_s;  // Save for logging
    
    // Modify timestamp (create ordering confusion)
    if (REPLAY_TIMESTAMP_OFFSET_S < 0) {
      // Subtract seconds (go back in time)
      if (h.ts_s >= (uint32_t)(-REPLAY_TIMESTAMP_OFFSET_S)) {
        h.ts_s += REPLAY_TIMESTAMP_OFFSET_S;  // Note: offset is negative, so this subtracts
      } else {
        h.ts_s = 0;  // Clamp to 0
      }
    } else {
      h.ts_s += REPLAY_TIMESTAMP_OFFSET_S;
    }
    
    // Re-encode with new timestamp (recomputes MAC)
    size_t n = packet_encode(h, replay_frame, team_key);
    if (n != WIRE_TOTAL_LEN) {
      ESP_LOGW(TAG, "Replay: failed to re-encode packet");
      return false;
    }
    
    ESP_LOGI(TAG, "Replay: replaying packet with modified timestamp (old_ts=%u new_ts=%u offset=%d)",
             old_ts, h.ts_s, REPLAY_TIMESTAMP_OFFSET_S);
  } else {
    // Replay exact packet (no modification, works without key)
    memcpy(replay_frame, packet.data, WIRE_TOTAL_LEN);
    ESP_LOGI(TAG, "Replay: replaying exact captured packet (age=%u ms)",
             now_ms() - packet.captured_at_ms);
  }
  
  // Send the replayed packet
  int rc = radio_tx(replay_frame, WIRE_TOTAL_LEN);
  if (rc == RADIOLIB_ERR_NONE) {
    ESP_LOGI(TAG, "Replay: sent replayed packet");
    return true;
  } else {
    ESP_LOGW(TAG, "Replay: TX failed rc=%d", rc);
    return false;
  }
}

// Craft a spoofed attack packet for testing MAC verification and protocol handling
// Returns true if packet was crafted and sent
static bool craft_and_send_attack_packet(const State& real_state, const uint8_t* real_mac, 
                                         const uint8_t* team_key) {
  if (!ENABLE_ATTACK_MODE) return false;

  WireHeader attack_h{};
  
  // Start with real state
  attack_h.version = PROTO_VERSION;
  attack_h.team_id = TEAM_ID;
  memcpy(attack_h.node_id, real_mac, 6);
  attack_h.seq_number = real_state.seq;
  attack_h.ts_s = real_state.ts_s;
  attack_h.ts_ms = real_state.ts_ms;
  // Convert signed positions to unsigned (ensure non-negative)
  attack_h.x_mm = (uint32_t)(real_state.x_mm >= 0 ? real_state.x_mm : 0);
  attack_h.y_mm = (uint32_t)(real_state.y_mm >= 0 ? real_state.y_mm : 0);
  attack_h.z_mm = (uint32_t)(real_state.z_mm >= 0 ? real_state.z_mm : 0);
  attack_h.vx_mm_s = real_state.vx_mms;
  attack_h.vy_mm_s = real_state.vy_mms;
  attack_h.vz_mm_s = real_state.vz_mms;
  attack_h.yaw_cd = real_state.heading_cd;

  // Apply attack modifications
  if (ATTACK_SPOOF_POSITION) {
    attack_h.x_mm = (uint32_t)(ATTACK_SPOOFED_X_MM >= 0 ? ATTACK_SPOOFED_X_MM : 0);
    attack_h.y_mm = (uint32_t)(ATTACK_SPOOFED_Y_MM >= 0 ? ATTACK_SPOOFED_Y_MM : 0);
    attack_h.z_mm = (uint32_t)(ATTACK_SPOOFED_Z_MM >= 0 ? ATTACK_SPOOFED_Z_MM : 0);
  }
  if (ATTACK_SPOOF_TEAM) {
    attack_h.team_id = ATTACK_SPOOFED_TEAM_ID;
  }
  if (ATTACK_WRONG_VERSION) {
    attack_h.version = ATTACK_WRONG_VERSION_VAL;
  }

  uint8_t attack_frame[WIRE_TOTAL_LEN];
  
  // Encode with valid MAC (if we have the key, we can forge valid packets)
  size_t n = packet_encode(attack_h, attack_frame, team_key);
  if (n != WIRE_TOTAL_LEN) {
    ESP_LOGW(TAG, "Attack: failed to encode packet");
    return false;
  }

  // Corrupt MAC tag if requested (test rejection)
  if (ATTACK_INVALID_MAC) {
    // Flip bits in the MAC tag to make it invalid
    attack_frame[WIRE_NO_TAG_LEN] ^= 0xFF;
    attack_frame[WIRE_NO_TAG_LEN + 1] ^= 0xFF;
    attack_frame[WIRE_NO_TAG_LEN + 2] ^= 0xFF;
    attack_frame[WIRE_NO_TAG_LEN + 3] ^= 0xFF;
  }

          // Send the attack packet
          int rc = radio_tx(attack_frame, WIRE_TOTAL_LEN);
          if (rc == RADIOLIB_ERR_NONE) {
            ESP_LOGI(TAG, "Attack: sent spoofed packet (team=%u pos=(%u,%u,%u) version=%u invalid_mac=%s)",
                     attack_h.team_id, attack_h.x_mm, attack_h.y_mm, attack_h.z_mm,
                     attack_h.version, ATTACK_INVALID_MAC ? "yes" : "no");
    return true;
  } else {
    ESP_LOGW(TAG, "Attack: TX failed rc=%d", rc);
    return false;
  }
}

extern "C" void vTaskRadio(void*){
  ESP_LOGI(TAG, "Radio task start: %d ms period", RADIO_PERIOD_MS);
  if(!radio_init()){
    ESP_LOGE(TAG, "radio_init failed"); vTaskDelay(portMAX_DELAY);
  }
  neighbors_init();

  // local cache of self state
  State self{};
  uint8_t node_mac[6]; radio_get_node_mac(node_mac);
  
  // Log MAC address at startup
  ESP_LOGI(TAG, "Node MAC: %02X:%02X:%02X:%02X:%02X:%02X",
           node_mac[0], node_mac[1], node_mac[2], node_mac[3], node_mac[4], node_mac[5]);
  
  if (DISABLE_CMAC_VERIFICATION) {
    ESP_LOGW(TAG, "WARNING: CMAC verification is DISABLED - accepting all packets (testing only)");
  }

  TickType_t last = xTaskGetTickCount();

  const uint8_t* team_key = get_team_key();
  
  // Attack mode: track last attack packet time
  TickType_t last_attack_time = 0;
  
  // Replay attack: initialize buffer and track last replay time
  if (ENABLE_REPLAY_ATTACK) {
    memset(g_replay_buffer, 0, sizeof(g_replay_buffer));
    g_replay_buffer_index = 0;
    g_replay_buffer_count = 0;
    ESP_LOGI(TAG, "Replay attack enabled: buffer_size=%u interval=%u ms modify_ts=%s",
             REPLAY_BUFFER_SIZE, REPLAY_INTERVAL_MS, REPLAY_MODIFY_TIMESTAMP ? "yes" : "no");
  }
  TickType_t last_replay_time = 0;
  
  // Statistics counters
  static uint32_t rx_timeout_count = 0;
  static uint32_t rx_success_count = 0;
  static uint32_t tx_count = 0;
  static uint32_t log_counter = 0;

  for(;;){
    TickType_t t0 = xTaskGetTickCount();
    // ----- TX: build + send -----
    // Pull the latest self state if available
    if (q_state_for_tx){
      (void)xQueueReceive(q_state_for_tx, &self, 0);
    }
    WireHeader h{};
    h.version  = PROTO_VERSION;  // Protocol version
    h.team_id   = TEAM_ID;
    memcpy(h.node_id, node_mac, 6);
    h.seq_number = self.seq;
    h.ts_s      = self.ts_s;
    h.ts_ms     = self.ts_ms;
    // Convert signed positions to unsigned (ensure non-negative)
    h.x_mm      = (uint32_t)(self.x_mm >= 0 ? self.x_mm : 0);
    h.y_mm      = (uint32_t)(self.y_mm >= 0 ? self.y_mm : 0);
    h.z_mm      = (uint32_t)(self.z_mm >= 0 ? self.z_mm : 0);
    h.vx_mm_s   = self.vx_mms;
    h.vy_mm_s   = self.vy_mms;
    h.vz_mm_s   = self.vz_mms;
    h.yaw_cd    = self.heading_cd;  // Already uint16_t

    uint8_t frame[WIRE_TOTAL_LEN];
    size_t n = packet_encode(h, frame, team_key);
    if (n == WIRE_TOTAL_LEN){
      int rc = radio_tx(frame, n);
      if (rc == RADIOLIB_ERR_NONE){
        tx_count++;
        // Log TX more frequently to see sending activity (every 5 packets = ~1.5 seconds)
        if (tx_count % 5 == 0) {
          // Extract MAC tag (last 4 bytes of frame)
          const uint8_t* mac_tag = &frame[WIRE_TOTAL_LEN - 4];
          ESP_LOGI(TAG, "TX: seq=%u pos=(%u,%u,%u) vel=(%d,%d,%d) team=%u MAC_tag=%02X%02X%02X%02X",
                   h.seq_number, h.x_mm, h.y_mm, h.z_mm, h.vx_mm_s, h.vy_mm_s, h.vz_mm_s, h.team_id,
                   mac_tag[0], mac_tag[1], mac_tag[2], mac_tag[3]);
        }
      } else {
        ESP_LOGW(TAG, "TX fail rc=%d", rc);
      }
      // Yield after TX to let idle task reset watchdog
      taskYIELD();
    }

    // ----- Attack Mode: Send spoofed packets periodically -----
    if (ENABLE_ATTACK_MODE) {
      TickType_t now_ticks = xTaskGetTickCount();
      TickType_t attack_interval_ticks = pdMS_TO_TICKS(ATTACK_INTERVAL_MS);
      if (now_ticks - last_attack_time >= attack_interval_ticks) {
        craft_and_send_attack_packet(self, node_mac, team_key);
        last_attack_time = now_ticks;
        taskYIELD();  // Yield after attack packet
      }
    }

    // ----- Replay Attack: Replay captured packets periodically -----
    if (ENABLE_REPLAY_ATTACK) {
      TickType_t now_ticks = xTaskGetTickCount();
      TickType_t replay_interval_ticks = pdMS_TO_TICKS(REPLAY_INTERVAL_MS);
      if (now_ticks - last_replay_time >= replay_interval_ticks) {
        replay_captured_packet(team_key);
        last_replay_time = now_ticks;
        taskYIELD();  // Yield after replay packet
      }
    }

    // ----- RX: poll a few times within this period -----
    uint8_t rxbuf[WIRE_TOTAL_LEN];
    // Use reasonable receive window: 4 attempts with 20ms timeout each = 80ms total
    // This gives enough time to receive packets while still yielding frequently
    for (int i=0;i<4;i++){
      // Yield before RX to allow IDLE task to run and reset watchdog
      taskYIELD();
      int got = radio_rx(rxbuf, sizeof(rxbuf), RADIO_RX_TIMEOUT_MS);
      if (got <= 0) {
        if (got == 0) rx_timeout_count++;
        continue; // 0 timeout, <0 error
      }
      
      // Log raw received packet for debugging
      if (rx_success_count == 0 || (rx_success_count % 10 == 0)) {
        ESP_LOGI(TAG, "RX raw packet: len=%d (expected %zu)", got, WIRE_TOTAL_LEN);
        if (got > 0 && got <= 48) {
          ESP_LOGI(TAG, "RX first 16 bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                   rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4], rxbuf[5], rxbuf[6], rxbuf[7],
                   rxbuf[8], rxbuf[9], rxbuf[10], rxbuf[11], rxbuf[12], rxbuf[13], rxbuf[14], rxbuf[15]);
        }
      }
      
      rx_success_count++;

      WireHeader r{};
      if (!packet_decode(rxbuf, (size_t)got, r, team_key)) {
        // Try to diagnose the issue
        if (got == WIRE_TOTAL_LEN) {
          // 46-byte format: compute CMAC over first 42 bytes
          uint8_t tag16[16];
          if (cmac_tag16(team_key, rxbuf, WIRE_NO_TAG_LEN, tag16)) {
            const uint8_t* computed_tag = tag16 + 12;
            const uint8_t* received_tag = rxbuf + WIRE_NO_TAG_LEN;
            ESP_LOGW(TAG, "RX decode failed (46-byte): CMAC mismatch");
            ESP_LOGW(TAG, "  Computed: %02X%02X%02X%02X", computed_tag[0], computed_tag[1], computed_tag[2], computed_tag[3]);
            ESP_LOGW(TAG, "  Received: %02X%02X%02X%02X", received_tag[0], received_tag[1], received_tag[2], received_tag[3]);
          } else {
            ESP_LOGW(TAG, "RX decode failed (46-byte): CMAC computation error");
          }
        } else if (got == 47) {
          // 47-byte format: compute CMAC over first 43 bytes
          uint8_t tag16[16];
          if (cmac_tag16(team_key, rxbuf, 43, tag16)) {
            const uint8_t* computed_tag = tag16 + 12;
            const uint8_t* received_tag = rxbuf + 43;
            ESP_LOGW(TAG, "RX decode failed (47-byte): CMAC mismatch");
            ESP_LOGW(TAG, "  Computed: %02X%02X%02X%02X", computed_tag[0], computed_tag[1], computed_tag[2], computed_tag[3]);
            ESP_LOGW(TAG, "  Received: %02X%02X%02X%02X", received_tag[0], received_tag[1], received_tag[2], received_tag[3]);
          } else {
            ESP_LOGW(TAG, "RX decode failed (47-byte): CMAC computation error");
          }
        } else if (got == 48) {
          // 48-byte format: compute CMAC over first 44 bytes
          uint8_t tag16[16];
          if (cmac_tag16(team_key, rxbuf, 44, tag16)) {
            const uint8_t* computed_tag = tag16 + 12;
            const uint8_t* received_tag = rxbuf + 44;
            ESP_LOGW(TAG, "RX decode failed (48-byte): CMAC mismatch");
            ESP_LOGW(TAG, "  Computed: %02X%02X%02X%02X", computed_tag[0], computed_tag[1], computed_tag[2], computed_tag[3]);
            ESP_LOGW(TAG, "  Received: %02X%02X%02X%02X", received_tag[0], received_tag[1], received_tag[2], received_tag[3]);
          } else {
            ESP_LOGW(TAG, "RX decode failed (48-byte): CMAC computation error");
          }
        } else {
          ESP_LOGW(TAG, "RX decode failed: len=%d (expected 46 bytes)", got);
        }
        continue;
      }
      
      // Check for suspicious version (might indicate attack)
      if (r.version != PROTO_VERSION) {
        ESP_LOGW(TAG, "RX protocol confusion: unknown version=%u (expected %u) from %02X:%02X:%02X:%02X:%02X:%02X",
                 r.version, PROTO_VERSION, r.node_id[0], r.node_id[1], r.node_id[2], 
                 r.node_id[3], r.node_id[4], r.node_id[5]);
        // Still process it (might be valid, just log warning)
      }
      
      // Ignore own frames
      if (memcmp(r.node_id, node_mac, 6)==0) continue;

      // Store packet for replay attack (before processing)
      if (ENABLE_REPLAY_ATTACK) {
        store_packet_for_replay(rxbuf, (size_t)got);
      }

      // Track both friends and enemies (distinguished by team_id)
      NeighborState nb{};
      memcpy(nb.mac, r.node_id, 6);
      nb.team_id = r.team_id;  // store team_id to distinguish friend/foe
      // Convert unsigned positions back to signed for NeighborState
      nb.x_mm = (int32_t)r.x_mm;
      nb.y_mm = (int32_t)r.y_mm;
      nb.z_mm = (int32_t)r.z_mm;
      nb.vx_mms = r.vx_mm_s;
      nb.vy_mms = r.vy_mm_s;
      nb.vz_mms = r.vz_mm_s;
      nb.heading_cd = r.yaw_cd;  // Already uint16_t in spec format
      nb.last_seen_ms = now_ms();
      neighbors_upsert(nb);
      
      // Log received packet - show raw bytes if position is zero (debugging 47-byte format)
      const char* type = (r.team_id == TEAM_ID) ? "FRIEND" : "ENEMY";
      if (got == 47 && r.x_mm == 0 && r.y_mm == 0 && r.z_mm == 0 && rx_success_count % 5 == 0) {
        // Debug: show raw position/velocity bytes for 47-byte packets with zero position
        ESP_LOGI(TAG, "RX: %s seq=%u pos=(%u,%u,%u) vel=(%d,%d,%d) from %02X:%02X:%02X:%02X:%02X:%02X",
                 type, r.seq_number, r.x_mm, r.y_mm, r.z_mm, r.vx_mm_s, r.vy_mm_s, r.vz_mm_s,
                 r.node_id[0], r.node_id[1], r.node_id[2], r.node_id[3], r.node_id[4], r.node_id[5]);
        ESP_LOGI(TAG, "  Raw pos/vel bytes [17-40]: x=%02X%02X%02X%02X y=%02X%02X%02X%02X z=%02X%02X%02X%02X vx=%02X%02X%02X%02X vy=%02X%02X%02X%02X vz=%02X%02X%02X%02X",
                 rxbuf[17], rxbuf[18], rxbuf[19], rxbuf[20],  // x_mm
                 rxbuf[21], rxbuf[22], rxbuf[23], rxbuf[24],  // y_mm
                 rxbuf[25], rxbuf[26], rxbuf[27], rxbuf[28],  // z_mm
                 rxbuf[29], rxbuf[30], rxbuf[31], rxbuf[32],  // vx_mm_s
                 rxbuf[33], rxbuf[34], rxbuf[35], rxbuf[36],  // vy_mm_s
                 rxbuf[37], rxbuf[38], rxbuf[39], rxbuf[40]); // vz_mm_s
      } else if (rx_success_count % 5 == 0) {
        ESP_LOGI(TAG, "RX: %s seq=%u pos=(%u,%u,%u) vel=(%d,%d,%d) from %02X:%02X:%02X:%02X:%02X:%02X",
                 type, r.seq_number, r.x_mm, r.y_mm, r.z_mm, r.vx_mm_s, r.vy_mm_s, r.vz_mm_s,
                 r.node_id[0], r.node_id[1], r.node_id[2], r.node_id[3], r.node_id[4], r.node_id[5]);
      }
    }

    // Prune & publish snapshot to Control
    neighbors_prune(now_ms(), NEIGHBOR_MAX_AGE_MS);
    if (q_neighbors){
      NeighborsView view{};
      neighbors_snapshot(view, now_ms());
      (void)xQueueOverwrite(q_neighbors, &view);
      
      // Log neighbor table contents periodically (every 5 seconds = ~12 iterations)
      if (++log_counter >= 12) {
        log_counter = 0;
        ESP_LOGI(TAG, "=== Radio Status ===");
        ESP_LOGI(TAG, "TX: %u packets sent | RX: %u received, %u timeouts",
                 tx_count, rx_success_count, rx_timeout_count);
        ESP_LOGI(TAG, "Neighbor table: friends=%u enemies=%u", view.count_friends, view.count_enemies);
        for (uint8_t i = 0; i < view.count_friends; i++) {
          const auto& f = view.friends[i];
          uint32_t age_ms = now_ms() - f.last_seen_ms;
          ESP_LOGI(TAG, "  Friend[%u]: MAC=%02X:%02X:%02X:%02X:%02X:%02X pos=(%d,%d,%d) age=%u ms",
                   i, f.mac[0], f.mac[1], f.mac[2], f.mac[3], f.mac[4], f.mac[5],
                   f.x_mm, f.y_mm, f.z_mm, age_ms);
        }
        for (uint8_t i = 0; i < view.count_enemies; i++) {
          const auto& e = view.enemies[i];
          uint32_t age_ms = now_ms() - e.last_seen_ms;
          ESP_LOGI(TAG, "  Enemy[%u]: MAC=%02X:%02X:%02X:%02X:%02X:%02X team=%u pos=(%d,%d,%d) age=%u ms",
                   i, e.mac[0], e.mac[1], e.mac[2], e.mac[3], e.mac[4], e.mac[5],
                   e.team_id, e.x_mm, e.y_mm, e.z_mm, age_ms);
        }
      }
      
      // Yield after snapshot to let idle task reset watchdog
      taskYIELD();
    }

    TickType_t t1 = xTaskGetTickCount();
    metrics_record_radio(t0, t1);

    vTaskDelayUntil(&last, pdMS_TO_TICKS(RADIO_PERIOD_MS));
  }
}
