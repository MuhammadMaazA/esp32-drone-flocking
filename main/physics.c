#include "physics.h"
#include <math.h>
#include <string.h>

// Constants
#define MAX_SPEED_MM_S 5000.0f
#define MIN_SEPARATION_MM 2000.0f
#define BOUNDARY_MIN_MM 0
#define BOUNDARY_MAX_MM 100000  // 100m = 100,000mm

// Default flocking weights
static FlockingWeights default_weights = {
    .cohesion_weight = 1.0f,
    .alignment_weight = 1.0f,
    .separation_weight = 1.5f
};

/**
 * Clamp a value between min and max
 */
static inline float clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * Calculate magnitude of a 3D vector
 */
static inline float vector_magnitude(Vector3D v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * Normalize a 3D vector
 */
static inline Vector3D vector_normalize(Vector3D v)
{
    float mag = vector_magnitude(v);
    if (mag < 0.001f) {
        return (Vector3D){0.0f, 0.0f, 0.0f};
    }
    return (Vector3D){v.x / mag, v.y / mag, v.z / mag};
}

/**
 * Limit vector magnitude to max_magnitude
 */
static inline Vector3D vector_limit(Vector3D v, float max_magnitude)
{
    float mag = vector_magnitude(v);
    if (mag > max_magnitude && mag > 0.001f) {
        float scale = max_magnitude / mag;
        return (Vector3D){v.x * scale, v.y * scale, v.z * scale};
    }
    return v;
}

void update_physics(DroneState *state, float dt)
{
    if (state == NULL || dt <= 0.0f) {
        return;
    }

    // Update position based on velocity: pos += vel * dt
    state->x += (int32_t)(state->vx * dt);
    state->y += (int32_t)(state->vy * dt);
    state->z += (int32_t)(state->vz * dt);

    // Boundary checking - keep within 100m box
    if (state->x < BOUNDARY_MIN_MM) {
        state->x = BOUNDARY_MIN_MM;
        state->vx = 0;
    } else if (state->x > BOUNDARY_MAX_MM) {
        state->x = BOUNDARY_MAX_MM;
        state->vx = 0;
    }

    if (state->y < BOUNDARY_MIN_MM) {
        state->y = BOUNDARY_MIN_MM;
        state->vy = 0;
    } else if (state->y > BOUNDARY_MAX_MM) {
        state->y = BOUNDARY_MAX_MM;
        state->vy = 0;
    }

    if (state->z < BOUNDARY_MIN_MM) {
        state->z = BOUNDARY_MIN_MM;
        state->vz = 0;
    } else if (state->z > BOUNDARY_MAX_MM) {
        state->z = BOUNDARY_MAX_MM;
        state->vz = 0;
    }

    // Update heading based on velocity direction (if moving)
    if (state->vx != 0 || state->vy != 0) {
        float heading_rad = atan2f((float)state->vy, (float)state->vx);
        float heading_deg = heading_rad * 180.0f / M_PI;
        if (heading_deg < 0) heading_deg += 360.0f;
        state->yaw = (uint16_t)(heading_deg * 100.0f); // Convert to centi-degrees
    }
}

Vector3D calculate_flocking(DroneState self, Neighbor *neighbors, int count, FlockingWeights *weights)
{
    if (neighbors == NULL || count <= 0) {
        return (Vector3D){0.0f, 0.0f, 0.0f};
    }

    // Use default weights if none provided
    if (weights == NULL) {
        weights = &default_weights;
    }

    Vector3D cohesion = {0.0f, 0.0f, 0.0f};
    Vector3D alignment = {0.0f, 0.0f, 0.0f};
    Vector3D separation = {0.0f, 0.0f, 0.0f};

    int cohesion_count = 0;
    int alignment_count = 0;
    int separation_count = 0;

    // Process each neighbor
    for (int i = 0; i < count; i++) {
        Neighbor *n = &neighbors[i];

        // Calculate distance to neighbor
        float dx = (float)(n->x - self.x);
        float dy = (float)(n->y - self.y);
        float dz = (float)(n->z - self.z);
        float distance = sqrtf(dx * dx + dy * dy + dz * dz);

        // COHESION: Steer toward average position of neighbors
        cohesion.x += dx;
        cohesion.y += dy;
        cohesion.z += dz;
        cohesion_count++;

        // ALIGNMENT: Steer toward average velocity of neighbors
        alignment.x += (float)n->vx;
        alignment.y += (float)n->vy;
        alignment.z += (float)n->vz;
        alignment_count++;

        // SEPARATION: Steer away if too close
        if (distance < MIN_SEPARATION_MM && distance > 0.1f) {
            // Inverse distance weighting - closer neighbors have more influence
            float repulsion_strength = (MIN_SEPARATION_MM - distance) / distance;
            separation.x -= dx * repulsion_strength;
            separation.y -= dy * repulsion_strength;
            separation.z -= dz * repulsion_strength;
            separation_count++;
        }
    }

    // Average the cohesion force
    if (cohesion_count > 0) {
        cohesion.x /= (float)cohesion_count;
        cohesion.y /= (float)cohesion_count;
        cohesion.z /= (float)cohesion_count;
        cohesion = vector_normalize(cohesion);
        cohesion.x *= weights->cohesion_weight;
        cohesion.y *= weights->cohesion_weight;
        cohesion.z *= weights->cohesion_weight;
    }

    // Average the alignment force
    if (alignment_count > 0) {
        alignment.x /= (float)alignment_count;
        alignment.y /= (float)alignment_count;
        alignment.z /= (float)alignment_count;
        
        // Subtract current velocity to get desired change
        alignment.x -= (float)self.vx;
        alignment.y -= (float)self.vy;
        alignment.z -= (float)self.vz;
        
        alignment = vector_normalize(alignment);
        alignment.x *= weights->alignment_weight;
        alignment.y *= weights->alignment_weight;
        alignment.z *= weights->alignment_weight;
    }

    // Average the separation force
    if (separation_count > 0) {
        separation.x /= (float)separation_count;
        separation.y /= (float)separation_count;
        separation.z /= (float)separation_count;
        separation = vector_normalize(separation);
        separation.x *= weights->separation_weight;
        separation.y *= weights->separation_weight;
        separation.z *= weights->separation_weight;
    }

    // Combine all forces
    Vector3D result;
    result.x = cohesion.x + alignment.x + separation.x;
    result.y = cohesion.y + alignment.y + separation.y;
    result.z = cohesion.z + alignment.z + separation.z;

    // Scale to velocity and limit max speed
    result.x *= MAX_SPEED_MM_S;
    result.y *= MAX_SPEED_MM_S;
    result.z *= MAX_SPEED_MM_S;
    result = vector_limit(result, MAX_SPEED_MM_S);

    return result;
}
