#include "neighbor.h"
#include "config.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "neighbor";

void neighbor_table_init(NeighborTable *table)
{
    if (table == NULL) {
        return;
    }

    memset(table, 0, sizeof(NeighborTable));
    
    // Mark all entries as inactive
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        table->entries[i].active = false;
    }
    
    table->count = 0;
    table->total_received = 0;
    table->total_dropped = 0;
    
    ESP_LOGI(TAG, "Neighbor table initialized (max=%d)", MAX_NEIGHBORS);
}

bool neighbor_table_update(NeighborTable *table, const FlockPacket *packet, uint32_t current_time_ms)
{
    if (table == NULL || packet == NULL) {
        return false;
    }

    table->total_received++;

    // Find existing entry or get a free slot
    NeighborEntry *entry = NULL;
    int free_slot = -1;

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (table->entries[i].active && 
            memcmp(table->entries[i].node_id, packet->node_id, 6) == 0) {
            // Found existing neighbor
            entry = &table->entries[i];
            
            // Check for out-of-order packets (optional)
            if (packet->seq <= entry->seq && (entry->seq - packet->seq) < 30000) {
                // Old or duplicate packet (allow for sequence wrap-around)
                ESP_LOGD(TAG, "Ignoring old/duplicate packet (seq %u <= %u)", 
                         packet->seq, entry->seq);
                table->total_dropped++;
                return false;
            }
            break;
        } else if (!table->entries[i].active && free_slot == -1) {
            // Remember first free slot
            free_slot = i;
        }
    }

    // If not found and we have space, create new entry
    if (entry == NULL) {
        if (free_slot >= 0) {
            entry = &table->entries[free_slot];
            entry->active = true;
            table->count++;
            ESP_LOGI(TAG, "New neighbor added: %02X:%02X:%02X:%02X:%02X:%02X (count=%d)",
                     packet->node_id[0], packet->node_id[1], packet->node_id[2],
                     packet->node_id[3], packet->node_id[4], packet->node_id[5],
                     table->count);
        } else {
            ESP_LOGW(TAG, "Neighbor table full, dropping packet");
            table->total_dropped++;
            return false;
        }
    }

    // Update entry with packet data
    memcpy(entry->node_id, packet->node_id, 6);
    entry->x_mm = packet->x_mm;
    entry->y_mm = packet->y_mm;
    entry->z_mm = packet->z_mm;
    entry->vx_mm_s = packet->vx;
    entry->vy_mm_s = packet->vy;
    entry->vz_mm_s = packet->vz;
    entry->heading_cd = packet->heading;
    entry->seq = packet->seq;
    entry->last_seen_ms = current_time_ms;

    return true;
}

int neighbor_table_prune_stale(NeighborTable *table, uint32_t current_time_ms, uint32_t timeout_ms)
{
    if (table == NULL) {
        return 0;
    }

    int removed_count = 0;

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (table->entries[i].active) {
            uint32_t age_ms = current_time_ms - table->entries[i].last_seen_ms;
            
            if (age_ms > timeout_ms) {
                ESP_LOGI(TAG, "Removing stale neighbor: %02X:%02X:%02X:%02X:%02X:%02X (age=%ums)",
                         table->entries[i].node_id[0], table->entries[i].node_id[1],
                         table->entries[i].node_id[2], table->entries[i].node_id[3],
                         table->entries[i].node_id[4], table->entries[i].node_id[5],
                         age_ms);
                
                table->entries[i].active = false;
                table->count--;
                removed_count++;
            }
        }
    }

    return removed_count;
}

int neighbor_table_get_count(const NeighborTable *table)
{
    if (table == NULL) {
        return 0;
    }
    return table->count;
}

NeighborEntry* neighbor_table_find(NeighborTable *table, const uint8_t *node_id)
{
    if (table == NULL || node_id == NULL) {
        return NULL;
    }

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (table->entries[i].active && 
            memcmp(table->entries[i].node_id, node_id, 6) == 0) {
            return &table->entries[i];
        }
    }

    return NULL;
}

int neighbor_table_get_all(const NeighborTable *table, NeighborEntry *out_neighbors, int max_count)
{
    if (table == NULL || out_neighbors == NULL || max_count <= 0) {
        return 0;
    }

    int copied = 0;

    for (int i = 0; i < MAX_NEIGHBORS && copied < max_count; i++) {
        if (table->entries[i].active) {
            memcpy(&out_neighbors[copied], &table->entries[i], sizeof(NeighborEntry));
            copied++;
        }
    }

    return copied;
}

