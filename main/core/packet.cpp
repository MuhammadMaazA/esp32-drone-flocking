#include "core/packet.hpp"
#include <string.h>
#include "core/cmac.hpp"
#include "config/params.hpp"

size_t packet_encode(const WireHeader& h, uint8_t out[WIRE_TOTAL_LEN],
                     const uint8_t key16[16]){
  // Copy struct directly to buffer (little-endian, no conversion needed)
  memcpy(out, &h, sizeof(WireHeader));
  
  // Compute CMAC over data (excluding mac_tag)
  uint8_t tag16[16];
  if(!cmac_tag16(key16, out, WIRE_NO_TAG_LEN, tag16)) return 0;
  
  // Write the 4-byte truncated MAC tag (use LAST 4 bytes of 16-byte CMAC)
  memcpy(out + WIRE_NO_TAG_LEN, tag16 + 12, 4);  // bytes 12-15 of tag16
  
  return WIRE_TOTAL_LEN;
}

bool packet_decode(const uint8_t* in, size_t len, WireHeader& h,
                   const uint8_t key16[16]){
  // Support 46-byte (current format) packets
  if (len == WIRE_TOTAL_LEN) {
    // Verify CMAC before decoding
    if (!DISABLE_CMAC_VERIFICATION) {
      uint8_t tag16[16];
      if(!cmac_tag16(key16, in, WIRE_NO_TAG_LEN, tag16)) return false;
      // Compare using LAST 4 bytes of 16-byte CMAC
      if (memcmp(tag16 + 12, in + WIRE_NO_TAG_LEN, 4) != 0) return false;
    }
    
    // Copy buffer directly to struct (little-endian, no conversion needed)
    memcpy(&h, in, sizeof(WireHeader));
    return true;
  }
  
  return false;  // Wrong packet size
}
