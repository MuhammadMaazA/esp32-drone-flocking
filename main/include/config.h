#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// ==================================================================
// TEAM CONFIGURATION
// ==================================================================
#define TEAM_ID 0  // 0 = everyone (as per specification)
#define PROTOCOL_VERSION 1

// ==================================================================
// TASK FREQUENCIES (Hz)
// ==================================================================
#define PHYSICS_FREQ_HZ 50
#define FLOCKING_FREQ_HZ 10
#define RADIO_FREQ_HZ 5
#define TELEMETRY_FREQ_HZ 2

// Derived periods in milliseconds
#define PHYSICS_PERIOD_MS (1000 / PHYSICS_FREQ_HZ)
#define FLOCKING_PERIOD_MS (1000 / FLOCKING_FREQ_HZ)
#define RADIO_PERIOD_MS (1000 / RADIO_FREQ_HZ)
#define TELEMETRY_PERIOD_MS (1000 / TELEMETRY_FREQ_HZ)

// Derived time delta for physics integration
#define PHYSICS_DT (1.0f / PHYSICS_FREQ_HZ)

// ==================================================================
// DRONE PHYSICS PARAMETERS
// ==================================================================
// Maximum velocities (mm/s) - realistic for multirotor drones
#define MAX_VELOCITY_XY_MM_S 5000   // 5 m/s horizontal (realistic for consumer drones)
#define MAX_VELOCITY_Z_MM_S 3000    // 3 m/s vertical (realistic ascent/descent rate)

// Simulation boundaries (100m x 100m x 100m as per specification)
#define BOUNDARY_MIN_MM 0
#define BOUNDARY_MAX_MM 100000  // 100,000 mm = 100 m

// Initial position range (random spawn in center 50m cube)
#define INITIAL_POS_MIN_MM 25000    // 25m from origin
#define INITIAL_POS_MAX_MM 75000    // 75m from origin

// Initial velocity range (small random starting velocities)
#define INITIAL_VEL_MIN_MM_S -1000  // -1 m/s
#define INITIAL_VEL_MAX_MM_S 1000   // 1 m/s

// ==================================================================
// FLOCKING PARAMETERS
// ==================================================================
// Reynolds flocking weights (tunable for stable flocking behavior)
#define COHESION_WEIGHT 1.0f       // Attraction to flock center
#define ALIGNMENT_WEIGHT 1.0f      // Match neighbor velocities
#define SEPARATION_WEIGHT 1.5f     // Avoid collisions (higher priority)

// Flocking distances
#define MIN_SEPARATION_MM 2000     // 2m minimum separation
#define NEIGHBOR_PERCEPTION_RADIUS_MM 20000  // 20m perception radius

// Flocking velocity blending (smooth control)
#define FLOCKING_BLEND_FACTOR 0.1f  // 10% new velocity, 90% old (smooth)

// ==================================================================
// NEIGHBOR TABLE CONFIGURATION
// ==================================================================
#define MAX_NEIGHBORS 20            // Maximum tracked neighbors
#define NEIGHBOR_TIMEOUT_MS 3000    // 3 seconds without update = stale

// ==================================================================
// MQTT CONFIGURATION
// ==================================================================
#define MQTT_BROKER_URI "mqtt://engf0001.cs.ucl.ac.uk:1883"
#define MQTT_TOPIC "COMP0221/flock/0/state"
#define MQTT_QOS 1
#define MQTT_RETAIN 0

// ==================================================================
// SECURITY CONFIGURATION
// ==================================================================
// AES-128 key (16 bytes) - CHANGE THIS FOR YOUR TEAM!
// This is a placeholder - use your own secret key
#define AES_KEY_BYTES { \
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, \
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F  \
}

// ==================================================================
// FREERTOS TASK PRIORITIES
// ==================================================================
// Higher number = higher priority
// Physics needs highest priority for real-time deadline
#define PRIORITY_PHYSICS (configMAX_PRIORITIES - 1)  // Highest
#define PRIORITY_FLOCKING (tskIDLE_PRIORITY + 3)     // Medium-high
#define PRIORITY_RADIO (tskIDLE_PRIORITY + 2)        // Medium
#define PRIORITY_TELEMETRY (tskIDLE_PRIORITY + 1)    // Low

// ==================================================================
// FREERTOS TASK STACK SIZES
// ==================================================================
#define STACK_SIZE_PHYSICS 4096
#define STACK_SIZE_FLOCKING 4096
#define STACK_SIZE_RADIO 8192      // Larger for LoRa operations
#define STACK_SIZE_TELEMETRY 4096

// ==================================================================
// QUEUE CONFIGURATION
// ==================================================================
#define RADIO_QUEUE_SIZE 10
#define NEIGHBOR_QUEUE_SIZE 20

// ==================================================================
// LORA CONFIGURATION (if using LoRa)
// ==================================================================
// SX127x Pin Configuration (adjust for your hardware)
#define LORA_SPI_HOST SPI2_HOST
#define LORA_PIN_MOSI 23
#define LORA_PIN_MISO 19
#define LORA_PIN_SCK 18
#define LORA_PIN_CS 5
#define LORA_PIN_RST 14
#define LORA_PIN_DIO0 26

// LoRa RF Parameters
#define LORA_FREQUENCY 915000000   // 915 MHz (adjust for your region)
#define LORA_BANDWIDTH 125000      // 125 kHz
#define LORA_SPREADING_FACTOR 7    // SF7 (good balance of range/speed)
#define LORA_CODING_RATE 5         // 4/5
#define LORA_TX_POWER 14           // 14 dBm (adjust as needed)
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYNC_WORD 0x12

// ==================================================================
// NTP CONFIGURATION
// ==================================================================
#define NTP_SERVER "pool.ntp.org"
#define NTP_TIMEZONE "GMT0"        // UTC timezone

// ==================================================================
// PERFORMANCE MONITORING
// ==================================================================
#define ENABLE_PERFORMANCE_LOGGING 1
#define PERF_LOG_INTERVAL_MS 10000  // Log every 10 seconds

// ==================================================================
// DEBUG CONFIGURATION
// ==================================================================
#define DEBUG_PHYSICS 0
#define DEBUG_FLOCKING 0
#define DEBUG_RADIO 1
#define DEBUG_TELEMETRY 1

#endif // CONFIG_H

