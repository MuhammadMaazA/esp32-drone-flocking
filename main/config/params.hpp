#pragma once

/* LoRa Radio Configuration - Matching OFFICIAL main.cpp */
static constexpr float LORA_FREQUENCY           = 868.1f;  // MHz (UK/EU)
static constexpr int8_t  LORA_TX_POWER          = 14;      // dBm
static constexpr uint8_t LORA_SPREADING_FACTOR  = 9;       // 7-12
static constexpr float LORA_BANDWIDTH           = 250.0f;  // kHz (OFFICIAL uses 125.0, not 250.0!)
static constexpr uint8_t LORA_CODING_RATE       = 7;       // 5-8 (4/5 to 4/8) - 7 = 4/8
static constexpr uint16_t LORA_PREAMBLE_LENGTH  = 10;      // symbols
static constexpr uint8_t LORA_SYNC_WORD         = 0x12;
static constexpr bool    LORA_CRC_ON            = true;

/* LoRa Pin Configuration (TTGO LoRa32 v2.1) */
static constexpr int LORA_SCK_PIN    = 5;
static constexpr int LORA_MISO_PIN   = 19;
static constexpr int LORA_MOSI_PIN   = 27;
static constexpr int LORA_CS_PIN     = 18;
static constexpr int LORA_RST_PIN    = 23;    // Reset pin
static constexpr int LORA_DIO0_PIN   = 26;
static constexpr int LORA_DIO1_PIN   = 33;    // DIO1 (required by RadioLib)

/* Task Frequencies (Hz) - How often each task runs */
static constexpr int TASK_PHYSICS_FREQ_HZ     = 50;  // Physics simulation: 50 updates/sec
static constexpr int TASK_FLOCKING_FREQ_HZ    = 10;  // Flocking control: 10 updates/sec
static constexpr int TASK_RADIO_FREQ_HZ       = 3;   // LoRa transmissions: 3 per sec
static constexpr int TASK_TELEMETRY_FREQ_HZ   = 2;   // MQTT telemetry: 2 per sec

/* Task Periods (milliseconds) */
static constexpr int TASK_PHYSICS_PERIOD_MS      = (1000 / TASK_PHYSICS_FREQ_HZ);     // 20 ms
static constexpr int TASK_FLOCKING_PERIOD_MS     = (1000 / TASK_FLOCKING_FREQ_HZ);    // 100 ms
static constexpr int TASK_RADIO_PERIOD_MS        = (1000 / TASK_RADIO_FREQ_HZ);       // 333 ms
static constexpr int TASK_TELEMETRY_PERIOD_MS    = (1000 / TASK_TELEMETRY_FREQ_HZ);   // 500 ms

/* Task Priorities (higher number = higher priority) */
static constexpr int TASK_PHYSICS_PRIORITY        = 5;
static constexpr int TASK_RADIO_PRIORITY          = 4;    // Increased: Radio needs high priority due to blocking LoRa operations
static constexpr int TASK_FLOCKING_PRIORITY       = 3;
static constexpr int TASK_TELEMETRY_PRIORITY      = 2;

/* Task Stack Sizes (bytes) */
static constexpr int TASK_PHYSICS_STACK_SIZE      = 4096;
static constexpr int TASK_FLOCKING_STACK_SIZE     = 12288;    // Increased: NeighborsView is ~4KB (50 friends + 50 enemies), need space for multiple copies
static constexpr int TASK_RADIO_STACK_SIZE        = 8192;     // Increased: NeighborsView snapshot allocation (~4KB) + logging buffers
static constexpr int TASK_TELEMETRY_STACK_SIZE    = 4096;

/* Queue Sizes */
static constexpr int QUEUE_STATE_SIZE             = 1;    // Using xQueueOverwrite() - only need size 1 (keeps latest state)
static constexpr int QUEUE_VELOCITY_CMD_SIZE      = 10;   // Increased for flocking commands
static constexpr int QUEUE_RADIO_RX_SIZE          = 20;   // Increased for RX buffer

/* Protocol Configuration */
static constexpr uint8_t  PROTOCOL_VERSION       = 1;
static constexpr uint8_t  PROTO_VERSION          = PROTOCOL_VERSION;  // Alias for compatibility
static constexpr uint8_t  PROTO_FLAGS            = 0;     // set as needed
static constexpr uint8_t  TEAM_ID                = 0;     // 0 = everyone

// Packet format selection: 0 = 47-byte format (uint16_t heading), 1 = 48-byte format (int32_t heading)
static constexpr int USE_48_BYTE_FORMAT          = 1;     // Set to 1 to use 48-byte format (compatible with other teams)

#if USE_48_BYTE_FORMAT
    static constexpr int PACKET_SIZE              = 48;    // 48-byte format (int32_t heading)
#else
    static constexpr int PACKET_SIZE              = 47;    // 47-byte format (uint16_t heading)
#endif

/* Physical Simulation Parameters */
static constexpr int SPACE_SIZE_MM                = 100000;  // 100 meters in millimeters
static constexpr int MAX_VELOCITY_MMS             = 5000;     // 5 m/s max velocity (reasonable for multirotor)
static constexpr int MAX_YAW_RATE_CDS             = 18000;   // 180 deg/sec in centidegrees/sec

// Motion limits (converted from mm/s to m/s for compatibility)
static constexpr float MAX_SPEED_MPS   = MAX_VELOCITY_MMS / 1000.0f;   // clamp speed
static constexpr float MAX_ACCEL_MPS2  = 3.0f;   // rate limit when tracking cmd vel

// "World" bounds (simple box; reflect at edges) — keep within 100 m cube
static constexpr float WORLD_X_MIN_M =   0.0f;
static constexpr float WORLD_X_MAX_M = 100.0f;
static constexpr float WORLD_Y_MIN_M =   0.0f;
static constexpr float WORLD_Y_MAX_M = 100.0f;
static constexpr float WORLD_Z_MIN_M =   0.0f;
static constexpr float WORLD_Z_MAX_M =  30.0f;

// Sequence number start
static constexpr unsigned INITIAL_SEQ = 1;

/* Flocking Algorithm Parameters - Reynolds' Three Rules */
// Weights: Higher values = stronger influence
static constexpr float FLOCKING_COHESION_WEIGHT     = 1.0f;   // Pull toward flock center
static constexpr float FLOCKING_SEPARATION_WEIGHT   = 1.5f;   // Avoid collisions
static constexpr float FLOCKING_ALIGNMENT_WEIGHT    = 1.0f;   // Match flock velocity

// Distance thresholds
static constexpr int FLOCKING_SEPARATION_DIST_MM    = 2000;   // 2m: Minimum safe distance
static constexpr int FLOCKING_NEIGHBOR_RANGE_MM     = 20000;  // 20m: Max neighbor detection

// Converted to meters for calculations
static constexpr float SEPARATION_RADIUS_M  = FLOCKING_SEPARATION_DIST_MM / 1000.0f;  
static constexpr float COHESION_RADIUS_M    = FLOCKING_NEIGHBOR_RANGE_MM / 1000.0f;   
static constexpr float ALIGNMENT_RADIUS_M   = FLOCKING_NEIGHBOR_RANGE_MM / 1000.0f;    

// Short aliases for internal use
static constexpr float R_SEP_M  = SEPARATION_RADIUS_M;
static constexpr float R_COH_M  = COHESION_RADIUS_M;
static constexpr float R_ALN_M  = ALIGNMENT_RADIUS_M;

// Weight aliases
static constexpr float W_SEP    = FLOCKING_SEPARATION_WEIGHT;
static constexpr float W_COH    = FLOCKING_COHESION_WEIGHT;
static constexpr float W_ALN    = FLOCKING_ALIGNMENT_WEIGHT;

// Small cruise speed if alone [m/s]
static constexpr float CRUISE_V_MPS = 0.4f;

// Numerical guards (renamed from EPS to avoid conflict with ESP-IDF macro)
static constexpr float EPSILON = 1e-5f;

/* Neighbor Table Configuration */
static constexpr size_t MAX_NEIGHBORS             = 50;
static constexpr int NEIGHBOR_TIMEOUT_MS          = 5000;    // 5 seconds
static constexpr uint32_t NEIGHBOR_MAX_AGE_MS     = NEIGHBOR_TIMEOUT_MS;  // Alias for compatibility

/* Security Configuration */
static constexpr int MAC_TAG_SIZE                 = 4;       // 4-byte truncated CMAC

/* Timing Measurement */
static constexpr int TIMING_ENABLE                = 1;       // Set to 0 to disable timing instrumentation

// --- Radio (LoRa) - Compatibility aliases ---
static constexpr int   PHY_PERIOD_MS              = TASK_PHYSICS_PERIOD_MS;
static constexpr int   CTRL_PERIOD_MS             = TASK_FLOCKING_PERIOD_MS;
static constexpr int   RADIO_PERIOD_MS            = TASK_RADIO_PERIOD_MS;
static constexpr int   TELE_PERIOD_MS             = TASK_TELEMETRY_PERIOD_MS;
static constexpr float LORA_FREQ_MHZ              = LORA_FREQUENCY;
static constexpr float LORA_BW_KHZ                = LORA_BANDWIDTH;
static constexpr uint8_t LORA_SF                  = LORA_SPREADING_FACTOR;
static constexpr uint8_t LORA_CR                  = LORA_CODING_RATE;
static constexpr uint8_t LORA_SYNCWORD            = LORA_SYNC_WORD;
static constexpr uint16_t LORA_PREAMBLE           = LORA_PREAMBLE_LENGTH;
static constexpr int8_t  LORA_POWER_DBM           = LORA_TX_POWER;
static constexpr uint32_t RADIO_RX_TIMEOUT_MS     = 20;    // per poll

// --- Attack/Defend behavior ---
// Flag bits: bit 0 = role (0=defender, 1=attacker)
static constexpr uint8_t  FLAG_ROLE_ATTACKER = 0x01;
static constexpr uint8_t  FLAG_ROLE_DEFENDER = 0x00;

// Set node role (attacker or defender)
static constexpr uint8_t  NODE_ROLE = FLAG_ROLE_DEFENDER;  // 0=defender, 1=attacker

// Attack behavior: approach nearest enemy within this range [m]
static constexpr float R_ATTACK_M = 10.0f;
static constexpr float W_ATTACK = 2.0f;  // weight for attack behavior

// Defend behavior: avoid enemies within this range [m]
static constexpr float R_DEFEND_AVOID_M = 5.0f;
static constexpr float W_DEFEND_AVOID = 1.5f;  // weight for defensive avoidance

// --- Adversarial Testing (Attack Mode) ---
// Enable packet spoofing and replay attacks for testing MAC verification and protocol handling
// 
// HOW TO USE SPOOFING ATTACKS:
// 1. Set ENABLE_ATTACK_MODE = true to enable attack packet generation
// 2. Configure which attack types to use (ATTACK_INVALID_MAC, ATTACK_SPOOF_POSITION, etc.)
// 3. Rebuild and flash - attack packets will be sent every ATTACK_INTERVAL_MS
// 4. Observe logs: "Attack: sent spoofed packet" and "RX invalid CMAC" (when MAC is invalid)
// 5. Observe behavior: spoofed positions/teams should affect flocking if MAC is valid
//
// HOW TO USE REPLAY ATTACKS:
// 1. Set ENABLE_REPLAY_ATTACK = true to enable packet capture and replay
// 2. Configure REPLAY_MODIFY_TIMESTAMP to test timestamp ordering confusion
// 3. Rebuild and flash - packets will be captured on RX and replayed every REPLAY_INTERVAL_MS
// 4. Observe logs: "Replay: replaying packet" - shows captured packets being replayed
// 5. With timestamp modification: creates ordering confusion (old packets appear new)
// 6. Without timestamp modification: exact replay (works without knowing sender's key)
//
// NOTE: This is for testing only. Ensure you have ethics plan approval before using.
static constexpr bool ENABLE_ATTACK_MODE = false;  // Set to true to enable attacks
static constexpr uint32_t ATTACK_INTERVAL_MS = 2000;  // Send attack packet every 2 seconds

// Attack types (can be combined)
static constexpr bool ATTACK_INVALID_MAC = true;      // Send packet with corrupted MAC tag
static constexpr bool ATTACK_SPOOF_POSITION = true;   // Inject false position (far away)
static constexpr bool ATTACK_SPOOF_TEAM = true;        // Spoof different team_id (create fake enemy)
static constexpr bool ATTACK_WRONG_VERSION = true;    // Send packet with wrong protocol version
static constexpr bool ATTACK_WRONG_FLAGS = true;       // Send packet with unknown flags

// Spoofed values for attack packets
static constexpr uint8_t  ATTACK_SPOOFED_TEAM_ID = 99;  // Fake team ID
static constexpr int32_t  ATTACK_SPOOFED_X_MM = 50000;  // Fake position (50m)
static constexpr int32_t  ATTACK_SPOOFED_Y_MM = 50000;
static constexpr int32_t  ATTACK_SPOOFED_Z_MM = 5000;
static constexpr uint8_t  ATTACK_WRONG_VERSION_VAL = 255;  // Invalid version
static constexpr uint8_t  ATTACK_WRONG_FLAGS_VAL = 0xFF;    // Unknown flags

// Replay attack configuration
static constexpr bool ENABLE_REPLAY_ATTACK = false;  // Set to true to enable replay attacks

// --- CMAC Authentication Configuration ---
// WARNING: Setting DISABLE_CMAC_VERIFICATION to true disables CMAC verification, allowing packets from devices
// with different team keys to be accepted. This is UNSAFE and should only be used for
// testing/debugging. Never use this in production or for actual security testing.
static constexpr bool DISABLE_CMAC_VERIFICATION = false;  // CMAC verification ENABLED - packets must have valid MAC tag
static constexpr uint32_t REPLAY_INTERVAL_MS = 3000;  // Replay captured packet every 3 seconds
static constexpr size_t REPLAY_BUFFER_SIZE = 5;  // Store last 5 captured packets
static constexpr bool REPLAY_MODIFY_TIMESTAMP = true;  // Modify timestamp when replaying (creates ordering confusion)
static constexpr int32_t REPLAY_TIMESTAMP_OFFSET_S = -10;  // Replay with timestamp 10 seconds in the past

// --- Telemetry (MQTT) ---
// TELE_PERIOD_MS is defined above as alias to TASK_TELEMETRY_PERIOD_MS
static constexpr const char* MQTT_BROKER_URI = "mqtt://test.mosquitto.org";
static constexpr const char* MQTT_TOPIC_STATE = "COMP0221/flock/0/state";
static constexpr const char* MQTT_TOPIC_HEARTBEAT = "COMP0221/heartbeat";  // Heartbeat topic
static constexpr const char* MQTT_TOPIC_TEST = "COMP0221/test";            // Subscription topic
static constexpr uint32_t MQTT_HEARTBEAT_INTERVAL_MS = 10000;              // 10 seconds
static constexpr const char* NODE_LABEL = "NodeA";    // change per board

