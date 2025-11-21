#ifndef NEIGHBOR_H
#define NEIGHBOR_H

#include <stdint.h>
#include <stdbool.h>
#include "packet.h"

/**
 * Neighbor entry with state and metadata
 */
typedef struct {
    uint8_t node_id[6];     // Node MAC address
    int32_t x_mm;           // Position X (mm)
    int32_t y_mm;           // Position Y (mm)
    int32_t z_mm;           // Position Z (mm)
    int32_t vx_mm_s;        // Velocity X (mm/s)
    int32_t vy_mm_s;        // Velocity Y (mm/s)
    int32_t vz_mm_s;        // Velocity Z (mm/s)
    uint16_t heading_cd;    // Heading (centi-degrees)
    uint16_t seq;           // Last sequence number
    uint32_t last_seen_ms;  // Timestamp of last update (milliseconds)
    bool active;            // Is this entry valid?
} NeighborEntry;

/**
 * Neighbor table manager
 */
typedef struct {
    NeighborEntry entries[20];  // MAX_NEIGHBORS
    int count;                   // Active neighbor count
    uint32_t total_received;     // Total packets received
    uint32_t total_dropped;      // Total packets dropped (invalid)
} NeighborTable;

/**
 * Initialize the neighbor table
 * @param table Pointer to neighbor table
 */
void neighbor_table_init(NeighborTable *table);

/**
 * Update or add a neighbor from a received packet
 * @param table Pointer to neighbor table
 * @param packet Pointer to received packet
 * @param current_time_ms Current system time in milliseconds
 * @return true if neighbor was updated/added, false otherwise
 */
bool neighbor_table_update(NeighborTable *table, const FlockPacket *packet, uint32_t current_time_ms);

/**
 * Remove stale neighbors (not seen within timeout)
 * @param table Pointer to neighbor table
 * @param current_time_ms Current system time in milliseconds
 * @param timeout_ms Timeout threshold in milliseconds
 * @return Number of neighbors removed
 */
int neighbor_table_prune_stale(NeighborTable *table, uint32_t current_time_ms, uint32_t timeout_ms);

/**
 * Get count of active neighbors
 * @param table Pointer to neighbor table
 * @return Number of active neighbors
 */
int neighbor_table_get_count(const NeighborTable *table);

/**
 * Check if a node ID exists in the table
 * @param table Pointer to neighbor table
 * @param node_id Node MAC address (6 bytes)
 * @return Pointer to entry if found, NULL otherwise
 */
NeighborEntry* neighbor_table_find(NeighborTable *table, const uint8_t *node_id);

/**
 * Get all active neighbors (for flocking calculations)
 * @param table Pointer to neighbor table
 * @param out_neighbors Output array for neighbor data
 * @param max_count Maximum number of neighbors to copy
 * @return Number of neighbors copied
 */
int neighbor_table_get_all(const NeighborTable *table, NeighborEntry *out_neighbors, int max_count);

#endif // NEIGHBOR_H

