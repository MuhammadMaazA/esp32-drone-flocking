#include "packet.h"
#include <string.h>
#include <arpa/inet.h>  // For htonl, htons, ntohl, ntohs

int serialize_packet(const FlockPacket *packet, uint8_t *buffer, size_t buffer_len)
{
    if (packet == NULL || buffer == NULL) {
        return -1;
    }

    if (buffer_len < sizeof(FlockPacket)) {
        return -1;
    }

    size_t offset = 0;

    // Version (1 byte)
    buffer[offset++] = packet->version;

    // Flags (1 byte)
    buffer[offset++] = packet->flags;

    // Team ID (1 byte)
    buffer[offset++] = packet->team_id;

    // Node ID (6 bytes) - already in correct byte order
    memcpy(&buffer[offset], packet->node_id, 6);
    offset += 6;

    // Sequence number (2 bytes) - convert to Big-Endian
    uint16_t seq_be = htons(packet->seq);
    memcpy(&buffer[offset], &seq_be, 2);
    offset += 2;

    // Timestamp seconds (4 bytes) - convert to Big-Endian
    uint32_t ts_s_be = htonl(packet->ts_s);
    memcpy(&buffer[offset], &ts_s_be, 4);
    offset += 4;

    // Timestamp milliseconds (2 bytes) - convert to Big-Endian
    uint16_t ts_ms_be = htons(packet->ts_ms);
    memcpy(&buffer[offset], &ts_ms_be, 2);
    offset += 2;

    // X position (4 bytes) - convert to Big-Endian
    int32_t x_mm_be = htonl(packet->x_mm);
    memcpy(&buffer[offset], &x_mm_be, 4);
    offset += 4;

    // Y position (4 bytes) - convert to Big-Endian
    int32_t y_mm_be = htonl(packet->y_mm);
    memcpy(&buffer[offset], &y_mm_be, 4);
    offset += 4;

    // Z position (4 bytes) - convert to Big-Endian
    int32_t z_mm_be = htonl(packet->z_mm);
    memcpy(&buffer[offset], &z_mm_be, 4);
    offset += 4;

    // VX velocity (4 bytes) - convert to Big-Endian
    int32_t vx_be = htonl(packet->vx);
    memcpy(&buffer[offset], &vx_be, 4);
    offset += 4;

    // VY velocity (4 bytes) - convert to Big-Endian
    int32_t vy_be = htonl(packet->vy);
    memcpy(&buffer[offset], &vy_be, 4);
    offset += 4;

    // VZ velocity (4 bytes) - convert to Big-Endian
    int32_t vz_be = htonl(packet->vz);
    memcpy(&buffer[offset], &vz_be, 4);
    offset += 4;

    // Heading (2 bytes) - convert to Big-Endian
    uint16_t heading_be = htons(packet->heading);
    memcpy(&buffer[offset], &heading_be, 2);
    offset += 2;

    // MAC tag (4 bytes) - already in correct byte order
    memcpy(&buffer[offset], packet->mac_tag, 4);
    offset += 4;

    return offset;
}

int deserialize_packet(const uint8_t *buffer, size_t buffer_len, FlockPacket *packet)
{
    if (buffer == NULL || packet == NULL) {
        return -1;
    }

    if (buffer_len < sizeof(FlockPacket)) {
        return -1;
    }

    size_t offset = 0;

    // Version (1 byte)
    packet->version = buffer[offset++];

    // Flags (1 byte)
    packet->flags = buffer[offset++];

    // Team ID (1 byte)
    packet->team_id = buffer[offset++];

    // Node ID (6 bytes)
    memcpy(packet->node_id, &buffer[offset], 6);
    offset += 6;

    // Sequence number (2 bytes) - convert from Big-Endian
    uint16_t seq_be;
    memcpy(&seq_be, &buffer[offset], 2);
    packet->seq = ntohs(seq_be);
    offset += 2;

    // Timestamp seconds (4 bytes) - convert from Big-Endian
    uint32_t ts_s_be;
    memcpy(&ts_s_be, &buffer[offset], 4);
    packet->ts_s = ntohl(ts_s_be);
    offset += 4;

    // Timestamp milliseconds (2 bytes) - convert from Big-Endian
    uint16_t ts_ms_be;
    memcpy(&ts_ms_be, &buffer[offset], 2);
    packet->ts_ms = ntohs(ts_ms_be);
    offset += 2;

    // X position (4 bytes) - convert from Big-Endian
    int32_t x_mm_be;
    memcpy(&x_mm_be, &buffer[offset], 4);
    packet->x_mm = ntohl(x_mm_be);
    offset += 4;

    // Y position (4 bytes) - convert from Big-Endian
    int32_t y_mm_be;
    memcpy(&y_mm_be, &buffer[offset], 4);
    packet->y_mm = ntohl(y_mm_be);
    offset += 4;

    // Z position (4 bytes) - convert from Big-Endian
    int32_t z_mm_be;
    memcpy(&z_mm_be, &buffer[offset], 4);
    packet->z_mm = ntohl(z_mm_be);
    offset += 4;

    // VX velocity (4 bytes) - convert from Big-Endian
    int32_t vx_be;
    memcpy(&vx_be, &buffer[offset], 4);
    packet->vx = ntohl(vx_be);
    offset += 4;

    // VY velocity (4 bytes) - convert from Big-Endian
    int32_t vy_be;
    memcpy(&vy_be, &buffer[offset], 4);
    packet->vy = ntohl(vy_be);
    offset += 4;

    // VZ velocity (4 bytes) - convert from Big-Endian
    int32_t vz_be;
    memcpy(&vz_be, &buffer[offset], 4);
    packet->vz = ntohl(vz_be);
    offset += 4;

    // Heading (2 bytes) - convert from Big-Endian
    uint16_t heading_be;
    memcpy(&heading_be, &buffer[offset], 2);
    packet->heading = ntohs(heading_be);
    offset += 2;

    // MAC tag (4 bytes)
    memcpy(packet->mac_tag, &buffer[offset], 4);
    offset += 4;

    return 0;
}
