#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include <stddef.h>

// FlockPacket structure (Big-Endian / Network Byte Order)
// Total size: 1 + 1 + 1 + 6 + 2 + 4 + 2 + 4*7 + 4 = 49 bytes
typedef struct __attribute__((packed)) {
    uint8_t version;        // Protocol version
    uint8_t flags;          // Status flags
    uint8_t team_id;        // Team identifier
    uint8_t node_id[6];     // Node MAC address
    uint16_t seq;           // Sequence number (Big-Endian)
    uint32_t ts_s;          // Timestamp seconds (Big-Endian)
    uint16_t ts_ms;         // Timestamp milliseconds (Big-Endian)
    int32_t x_mm;           // X position in mm (Big-Endian)
    int32_t y_mm;           // Y position in mm (Big-Endian)
    int32_t z_mm;           // Z position in mm (Big-Endian)
    int32_t vx;             // X velocity in mm/s (Big-Endian)
    int32_t vy;             // Y velocity in mm/s (Big-Endian)
    int32_t vz;             // Z velocity in mm/s (Big-Endian)
    uint16_t heading;       // Heading in centi-degrees (Big-Endian)
    uint8_t mac_tag[4];     // AES-CMAC authentication tag
} FlockPacket;

/**
 * Serialize a FlockPacket into a byte buffer (Big-Endian)
 * @param packet Pointer to the packet structure
 * @param buffer Output buffer (must be at least sizeof(FlockPacket) bytes)
 * @param buffer_len Size of the output buffer
 * @return Number of bytes written, or -1 on error
 */
int serialize_packet(const FlockPacket *packet, uint8_t *buffer, size_t buffer_len);

/**
 * Deserialize a byte buffer into a FlockPacket (Big-Endian)
 * @param buffer Input buffer containing serialized packet
 * @param buffer_len Size of the input buffer
 * @param packet Output packet structure
 * @return 0 on success, -1 on error
 */
int deserialize_packet(const uint8_t *buffer, size_t buffer_len, FlockPacket *packet);

#endif // PACKET_H
