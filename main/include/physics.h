#ifndef PHYSICS_H
#define PHYSICS_H

#include <stdint.h>

// Vector3D structure for 3D calculations
typedef struct {
    float x;
    float y;
    float z;
} Vector3D;

// Neighbor information for flocking
typedef struct {
    int32_t x;      // mm
    int32_t y;      // mm
    int32_t z;      // mm
    int32_t vx;     // mm/s
    int32_t vy;     // mm/s
    int32_t vz;     // mm/s
} Neighbor;

// DroneState structure (should match main.c)
typedef struct {
    int32_t x;          // mm
    int32_t y;          // mm
    int32_t z;          // mm
    int32_t vx;         // mm/s
    int32_t vy;         // mm/s
    int32_t vz;         // mm/s
    uint16_t yaw;       // centi-degrees
} DroneState;

// Flocking weights (tunable)
typedef struct {
    float cohesion_weight;
    float alignment_weight;
    float separation_weight;
} FlockingWeights;

/**
 * Update drone physics based on current velocity
 * @param state Pointer to drone state
 * @param dt Time delta in seconds
 */
void update_physics(DroneState *state, float dt);

/**
 * Calculate flocking velocity adjustments using Reynolds rules
 * @param self Current drone state
 * @param neighbors Array of neighbor drones
 * @param count Number of neighbors
 * @param weights Flocking algorithm weights
 * @return Desired velocity adjustment vector
 */
Vector3D calculate_flocking(DroneState self, Neighbor *neighbors, int count, FlockingWeights *weights);

#endif // PHYSICS_H
