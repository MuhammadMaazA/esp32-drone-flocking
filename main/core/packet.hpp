#pragma once
#include <stddef.h>
#include <stdint.h>
#include "core/state.hpp"
#include <stddef.h>  // for offsetof

// Packet structure matching coursework spec (SHA-256 hash: 6abd70d5283e46de)
// Total size: 46 bytes (42 bytes data + 4 bytes MAC tag)
struct WireHeader {
  uint8_t  version;      // Protocol version
  uint8_t  team_id;      // Team identifier
  uint8_t  node_id[6];   // ESP MAC address
  uint16_t seq_number;   // Sequence number
  uint32_t ts_s;         // Unix seconds (NTP-synced)
  uint16_t ts_ms;        // Milliseconds remainder (0..999)
  uint32_t x_mm;         // X position in millimeters
  uint32_t y_mm;         // Y position in millimeters
  uint32_t z_mm;         // Z position in millimeters
  int32_t  vx_mm_s;      // X velocity in millimeters per second
  int32_t  vy_mm_s;      // Y velocity in millimeters per second
  int32_t  vz_mm_s;      // Z velocity in millimeters per second
  uint16_t yaw_cd;       // Yaw heading in centi-degrees (0..35999)
  uint8_t  mac_tag[4];   // 4-byte truncated CMAC tag
} __attribute__((packed));

// Use offsetof to find where mac_tag starts (as suggested in spec)
constexpr size_t WIRE_NO_TAG_LEN = offsetof(WireHeader, mac_tag);  // Bytes before mac_tag (for CMAC input)
constexpr size_t WIRE_TAG_LEN    = 4;
constexpr size_t WIRE_TOTAL_LEN  = sizeof(WireHeader);

// Encode struct + compute and append 4B CMAC tag
// returns length written (WIRE_TOTAL_LEN) or 0 on error
size_t packet_encode(const WireHeader& h, uint8_t out[WIRE_TOTAL_LEN],
                     const uint8_t key16[16]);

// Decode & CMAC verify; returns true on success; fills out 'h'
bool packet_decode(const uint8_t* in, size_t len, WireHeader& h,
                   const uint8_t key16[16]);
