# ESP32 Drone Swarm with Flocking Behavior

A distributed multi-agent system for ESP32 microcontrollers implementing Reynolds' flocking behavior with attack/defense capabilities using LoRa radio communication.

## Features

- **Flocking Behavior**: Implements Reynolds' three rules (separation, cohesion, alignment)
- **Team-Based Combat**: Attack and defense behaviors between different teams
- **LoRa Communication**: Long-range radio communication for neighbor detection
- **Security**: CMAC authentication to prevent packet spoofing
- **Real-Time Telemetry**: MQTT publishing for monitoring and visualization
- **Configurable Parameters**: Tune behavior weights and ranges

## Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Quick Start](#quick-start)
- [System Architecture](#system-architecture)
- [Configuration Guide](#configuration-guide)
- [Testing & Deployment](#testing--deployment)
- [How It Works](#how-it-works)
- [Troubleshooting](#troubleshooting)
- [Parameter Tuning](#parameter-tuning)
- [Documentation](#documentation)

---

## Hardware Requirements

- **ESP32 boards** with LoRa radio (e.g., TTGO v2.1 LoRa32)
- At least 2 ESP32 devices for testing
- USB cables for flashing
- Wi-Fi network with internet access (mobile hotspot works great!)

## Quick Start

### 1. Install ESP-IDF

Make sure you have ESP-IDF v5.x installed. Set up the environment:

**Windows:**
```powershell
. $HOME\esp\v5.5.1\esp-idf\export.ps1
```

**Linux/Mac:**
```bash
. $HOME/esp/esp-idf/export.sh
```

### 2. Configure Wi-Fi (One-Time Setup)

**Option A: Using menuconfig (Recommended)**

```bash
idf.py menuconfig
```

Navigate to:
- **Wi-Fi Configuration** → Set `WIFI_SSID` and `WIFI_PASSWORD`
- **Team Configuration** → Set all 16 bytes of team key (same for all ESP32s)

**Option B: Hardcode in main.cpp**

Edit `main/main.cpp` (around lines 37-42):

```cpp
#define WIFI_SSID "YourWiFiName"
#define WIFI_PASS "YourWiFiPassword"
```

**Note**: Use 2.4GHz networks only (ESP32 doesn't support 5GHz)

### 3. Configure Each ESP32

Edit `main/config/params.hpp` for each device:

**ESP32 #1 (Team 1, Defender):**
```cpp
static constexpr uint8_t  TEAM_ID       = 1;
static constexpr uint8_t  NODE_ROLE     = FLAG_ROLE_DEFENDER;
static constexpr const char* NODE_LABEL = "Team1-Defender";
```

**ESP32 #2 (Team 2, Attacker):**
```cpp
static constexpr uint8_t  TEAM_ID       = 2;  // Different team = enemy
static constexpr uint8_t  NODE_ROLE     = FLAG_ROLE_ATTACKER;
static constexpr const char* NODE_LABEL = "Team2-Attacker";
```

### 4. Build and Flash

For each ESP32:

```bash
# Clean build directory (first time only)
idf.py fullclean

# Build, flash, and monitor
idf.py build flash monitor
```

**Important**: You must rebuild after changing `params.hpp` values!

---

## System Architecture

### Overview

The system uses **FreeRTOS** with four concurrent tasks:

```
┌─────────────────────────────────────────────────────────┐
│                    ESP32 Node                            │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐ │
│  │   Physics    │──▶│   Control    │──▶│    Radio     │ │
│  │  Task (50Hz) │   │  Task (10Hz) │   │ Task (2.5Hz) │ │
│  └──────────────┘   └──────────────┘   └──────────────┘ │
│         │                  │                  │          │
│         ▼                  ▼                  ▼          │
│  ┌──────────────┐   ┌──────────────┐   ┌─────────────┐ │
│  │  Position    │   │  Neighbors   │   │  LoRa TX/RX │ │
│  │  Velocity    │   │  Table       │   │  + CMAC     │ │
│  │  Heading     │   │  (Friends/   │   │  Auth       │ │
│  │              │   │   Enemies)   │   │             │ │
│  └──────────────┘   └──────────────┘   └─────────────┘ │
│                                                           │
│  ┌──────────────┐                                        │
│  │  Telemetry   │                                        │
│  │  Task (2Hz)  │                                        │
│  │  MQTT→Cloud  │                                        │
│  └──────────────┘                                        │
└─────────────────────────────────────────────────────────┘
```

### Task Responsibilities

1. **Physics Task (50 Hz)**: Simulates drone physics, updates position/velocity
2. **Control Task (10 Hz)**: Computes flocking + attack/defense behaviors
3. **Radio Task (2.5 Hz)**: LoRa communication, neighbor detection, CMAC verification
4. **Telemetry Task (2 Hz)**: Publishes state to MQTT for visualization

### Communication Channels

**LoRa Radio (Primary)**
- ESP32-to-ESP32 communication
- Neighbor discovery and tracking
- Position/velocity broadcasting
- CMAC-authenticated packets

**Wi-Fi (Secondary)**
- SNTP time synchronization
- MQTT telemetry publishing
- Monitoring and visualization only

**Important**: ESP32s communicate via LoRa, NOT Wi-Fi!

---

## How It Works

### Flocking Behavior (Reynolds' Rules)

The system implements three classic flocking behaviors that apply to **friendly neighbors** (same `TEAM_ID`):

1. **Separation** (lines 78-85 in `task_control.cpp`)
   - Avoid getting too close to friends
   - Range: `R_SEP_M = 0.7m`

2. **Cohesion** (lines 88-90, 96-106)
   - Move toward the center of nearby friends
   - Range: `R_COH_M = 3.0m`

3. **Alignment** (lines 91-93, 108-114)
   - Match velocity/direction of nearby friends
   - Range: `R_ALN_M = 3.0m`

### Attack/Defense Behaviors

**Attacker Role** (`FLAG_ROLE_ATTACKER`):
- Approaches nearest enemy within 10m
- Blends flocking with attack behavior
- Weight: `W_ATTACK = 2.0`

**Defender Role** (`FLAG_ROLE_DEFENDER`):
- Avoids enemies within 5m
- Maintains formation with friendly team
- Weight: `W_DEFEND_AVOID = 1.5`

### Security: CMAC Authentication

Every packet includes a 4-byte CMAC tag computed using AES-128-CMAC:

✅ **Valid packet** (correct MAC): Accepted → Added to neighbor table
❌ **Invalid packet** (wrong MAC): Rejected → Logged as "attack detected"

This prevents:
- Spoofed packets (without your team key)
- Modified packets (MAC won't match)
- Unauthorized nodes from joining

### Complete Data Flow

```
1. Radio Task receives LoRa packet
2. Verifies CMAC authentication
3. Decodes position, velocity, team_id
4. Stores in Neighbor Table
5. Classifies as friend (same team) or enemy (different team)
6. Control Task reads neighbor snapshot
7. Computes flocking (for friends) + attack/defend (for enemies)
8. Sends desired velocity to Physics Task
9. Physics Task updates position
10. Radio Task broadcasts new state
```

---

## Configuration Guide

### Team Configuration

| Setting | Location | Purpose | Must Match? |
|---------|----------|---------|-------------|
| Team Key | `idf.py menuconfig` → Team Configuration | Packet authentication | ✅ YES (all ESP32s) |
| Team ID | `main/config/params.hpp` | Friend/foe identification | ❌ NO (different per team) |
| Node Role | `main/config/params.hpp` | Attacker or Defender | ❌ NO (per ESP32) |
| Node Label | `main/config/params.hpp` | Identification | ❌ NO (per ESP32) |

### Flocking Parameters

Edit `main/config/params.hpp` (lines 25-32):

```cpp
static constexpr float R_SEP_M  = 0.7f;   // Separation radius (meters)
static constexpr float R_COH_M  = 3.0f;   // Cohesion radius
static constexpr float R_ALN_M  = 3.0f;   // Alignment radius
static constexpr float W_SEP    = 1.5f;   // Separation weight
static constexpr float W_COH    = 1.0f;   // Cohesion weight
static constexpr float W_ALN    = 1.0f;   // Alignment weight
```

### Attack/Defense Parameters

Edit `main/config/params.hpp` (lines 67-73):

```cpp
static constexpr float R_ATTACK_M        = 10.0f;  // Attack range
static constexpr float W_ATTACK          = 2.0f;   // Attack weight
static constexpr float R_DEFEND_AVOID_M  = 5.0f;   // Defense range
static constexpr float W_DEFEND_AVOID    = 1.5f;   // Defense weight
```

---

## Testing & Deployment

### Minimum Test (2 ESP32s)

**ESP32 #1:**
- `TEAM_ID = 1`
- `NODE_ROLE = FLAG_ROLE_DEFENDER`
- Build and flash

**ESP32 #2:**
- `TEAM_ID = 2` (different team = enemy)
- `NODE_ROLE = FLAG_ROLE_ATTACKER`
- Build and flash

**Expected Behavior:**
- ESP32 #2 approaches ESP32 #1
- ESP32 #1 avoids ESP32 #2

### Better Test (3 or more ESP32s)

**ESP32 #1:**
- `TEAM_ID = 1`, `NODE_ROLE = FLAG_ROLE_DEFENDER`

**ESP32 #2:**
- `TEAM_ID = 2`, `NODE_ROLE = FLAG_ROLE_ATTACKER`

**ESP32 #3:**
- `TEAM_ID = 1`, `NODE_ROLE = FLAG_ROLE_ATTACKER`

**Expected Behavior:**
- ESP32 #1 and #3 flock together (same team)
- ESP32 #2 approaches enemies (#1 or #3)
- ESP32 #1 avoids #2 (defender)
- ESP32 #3 approaches #2 (attacker)

### Monitoring

**Serial Monitor:**
```bash
idf.py monitor
```

**MQTT Telemetry:**
```bash
mosquitto_sub -h test.mosquitto.org -t "COMP0221/flock/0/state"
```

**Expected Logs:**
```
I (478) app: Booting ESP32 Drone Swarm node
I (500) app: Wi-Fi connecting to SSID="YourWiFi" ...
I (1200) app: Wi-Fi init done
I (1201) app: Starting SNTP time sync
I (2000) app: Time sync complete
I (2001) app: Tasks started: PHY(50Hz), CTRL(10Hz), RADIO(~2.5Hz), TELE(2Hz)
```

---

## Troubleshooting

### No Neighbors Detected

**Check:**
- ESP32s within LoRa range (~100-500m)
- Same team key configured (menuconfig)
- Same LoRa frequency (`LORA_FREQ_MHZ` in params.hpp)
- Serial logs for "RX invalid CMAC" (wrong team key)

### Wi-Fi Connection Issues

**Check:**
- Network is 2.4GHz (not 5GHz)
- SSID and password correct (case-sensitive!)
- Mobile hotspot enabled with mobile data
- Wait 5-10 seconds for connection

### Build Errors

**Solutions:**
- Run `idf.py fullclean` if weird errors
- Verify `params.hpp` saved before building
- Check ESP-IDF environment is sourced
- Delete and recreate `build/` directory

### Attack/Defend Not Working

**Check:**
- Different `TEAM_ID` for enemies
- `NODE_ROLE` set correctly
- Enemies within range (10m for attack, 5m for defend)
- Serial logs show neighbor detection

---

## Parameter Tuning

### Increase Flocking Tightness

```cpp
R_COH_M = 5.0f;    // Larger cohesion radius
W_COH   = 1.5f;    // Higher cohesion weight
```

### Make Attack More Aggressive

```cpp
R_ATTACK_M = 15.0f;  // Longer attack range
W_ATTACK   = 3.0f;   // Stronger attack weight
```

### Prevent Collisions

```cpp
R_SEP_M = 1.0f;    // Larger separation radius
W_SEP   = 2.0f;    // Higher separation weight
```

### Speed Limits

```cpp
MAX_SPEED_MPS   = 2.0f;   // Maximum speed (m/s)
MAX_ACCEL_MPS2  = 3.0f;   // Maximum acceleration
```

---

## Project Structure

```
esp32_drone_swarm/
├── main/
│   ├── main.cpp              # Entry point, Wi-Fi, SNTP setup
│   ├── config/
│   │   ├── params.hpp        # All tunable parameters
│   │   └── team_key.hpp      # Team key configuration
│   ├── core/
│   │   ├── cmac.cpp/hpp      # CMAC authentication
│   │   ├── neighbors.cpp/hpp # Neighbor table management
│   │   ├── packet.cpp/hpp    # Packet encoding/decoding
│   │   └── state.hpp         # State structures
│   ├── drivers/
│   │   └── radio_lora.*      # LoRa radio driver
│   ├── tasks/
│   │   ├── task_physics.*    # Physics simulation (50Hz)
│   │   ├── task_control.*    # Flocking + attack/defend (10Hz)
│   │   ├── task_radio.*      # LoRa TX/RX (2.5Hz)
│   │   └── task_telemetry.*  # MQTT publishing (2Hz)
│   └── net/
│       └── mqtt_client.*     # MQTT client
├── CMakeLists.txt            # Project configuration
├── sdkconfig                 # ESP-IDF configuration
└── README.md                 # This file
```

---

## Advanced Features

### Attack Modes (For Testing)

Edit `params.hpp` to enable:

```cpp
static constexpr bool ENABLE_ATTACK_MODE   = true;  // Send spoofed packets
static constexpr bool ENABLE_REPLAY_ATTACK = true;  // Replay captured packets
```

**Warning**: Only enable for security testing!

### Custom Behaviors

Modify `task_control.cpp` to implement custom behaviors:

```cpp
// Add your custom behavior
Vec3f custom_behavior(const State& self, const NeighborsView& neighbors) {
    Vec3f force = {0, 0, 0};
    // Your logic here
    return force;
}
```

---

---

## Documentation

Detailed documentation is available in the `docs/` folder:

- **[ARCHITECTURE.md](docs/ARCHITECTURE.md)** - System architecture, data flow, and internals
- **[SETUP.md](docs/SETUP.md)** - Detailed setup and installation guide
- **[PARAMETERS.md](docs/PARAMETERS.md)** - Complete parameter tuning guide

---

## Project Structure

```
esp32-drone-flocking/
├── docs/               # Detailed documentation
├── main/
│   ├── config/        # Configuration parameters
│   ├── core/          # Core system logic (CMAC, neighbors, packets)
│   ├── drivers/       # Hardware drivers (LoRa radio)
│   ├── net/           # Network communication (MQTT)
│   ├── tasks/         # FreeRTOS tasks (physics, control, radio, telemetry)
│   └── main.cpp       # Entry point
├── CMakeLists.txt     # Project configuration
├── sdkconfig          # ESP-IDF configuration
├── .gitignore         # Git ignore rules
├── LICENSE            # MIT License
└── README.md          # This file
```

---

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Author

Muhammad Maaz Azhar - [GitHub](https://github.com/MuhammadMaazA)

---

## Quick Reference

| Command | Purpose |
|---------|---------|
| `idf.py menuconfig` | Configure Wi-Fi and team key |
| `idf.py build` | Build project |
| `idf.py flash` | Flash to ESP32 |
| `idf.py monitor` | View serial output |
| `idf.py build flash monitor` | Build, flash, and monitor |
| `idf.py fullclean` | Clean build directory |

**Remember**: 
- Same team key for all ESP32s (menuconfig)
- Different team IDs for enemies (params.hpp)
- Rebuild after changing params.hpp!

---

## Support

For issues or questions, check:
1. Serial monitor logs
2. MQTT telemetry data
3. ESP-IDF documentation
4. LoRa radio specifications
