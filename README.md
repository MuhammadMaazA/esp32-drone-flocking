# Drone Flocking System - COMP0221 Coursework

A real-time simulated drone swarm implementation on ESP32 using FreeRTOS, with LoRa communication and MQTT visualization.

## 🎯 Overview

This project implements a distributed drone flocking system where each ESP32 device simulates a drone that:
- Updates physics at 50 Hz (position, velocity, heading)
- Computes flocking behavior at 10 Hz using Reynolds rules
- Exchanges neighbor state via LoRa at 5 Hz
- Publishes telemetry to MQTT at 2 Hz
- Monitors performance (latency, jitter, deadline misses)

## 📋 Features

### ✅ Core Requirements
- [x] **4 FreeRTOS tasks** with real-time scheduling
- [x] **3D Physics simulation** (position, velocity, yaw)
- [x] **Reynolds flocking** (cohesion, alignment, separation)
- [x] **LoRa packet format** with big-endian encoding
- [x] **AES-128-CMAC** authentication
- [x] **MQTT telemetry** to broker
- [x] **Performance monitoring** (latency, jitter, availability)
- [x] **Neighbor table** with timeout management
- [x] **NTP time synchronization**

### 📦 Modules

```
main/
├── main.c              # FreeRTOS tasks and system initialization
├── config.h            # Centralized configuration parameters
├── packet.c/h          # LoRa packet serialization (big-endian)
├── physics.c/h         # Drone physics and flocking algorithms
├── security.c/h        # AES-128-CMAC packet signing/verification
├── neighbor.c/h        # Neighbor table management with timeouts
├── performance.c/h     # Task performance monitoring
└── lora_radio.c/h      # LoRa radio interface (stub/driver)
```

## 🚀 Quick Start

### Prerequisites
- ESP-IDF v5.0 or later
- ESP32 development board
- eduroam WiFi credentials (or modify for your network)
- (Optional) LoRa module (SX1276/SX1278)

### Build and Flash

```bash
# Configure project
idf.py menuconfig

# Build
idf.py build

# Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

### Configuration

Edit `main/main.c` lines 32-36 to set your WiFi credentials:
```c
#define EDUROAM_SSID     "eduroam"
#define EDUROAM_IDENTITY "your-email@ucl.ac.uk"
#define EDUROAM_USERNAME "your-email@ucl.ac.uk"
#define EDUROAM_PASSWORD "your-password"
```

Edit `main/include/config.h` to tune parameters:
- Task frequencies (Hz)
- Flocking weights (cohesion, alignment, separation)
- Drone physics constraints (max velocity, boundaries)
- Team ID (0 = everyone)
- Security key (16 bytes)

## 📡 LoRa Integration

The system includes a LoRa interface stub in `lora_radio.c`. To integrate actual LoRa hardware:

### Hardware Setup
1. Connect SX1276/SX1278 module to ESP32:
   - MOSI → GPIO 23
   - MISO → GPIO 19
   - SCK → GPIO 18
   - CS → GPIO 5
   - RST → GPIO 14
   - DIO0 → GPIO 26

2. Adjust pins in `config.h` if needed

### Driver Integration
1. Add a LoRa driver library to your components (e.g., [ESP32-LoRa](https://github.com/Inteform/esp32-lora-library))
2. Update `lora_radio.c`:
   - Replace `lora_init()` with actual initialization
   - Implement `lora_transmit()` using driver's send function
   - Set up RX interrupt handler to call `lora_rx_handler()`
3. Update `CMakeLists.txt` to link the driver library

### Testing Without Hardware
The system runs without LoRa hardware - it will:
- Log "TX stub" messages instead of transmitting
- Not receive packets from other nodes
- Still publish telemetry to MQTT for single-drone testing

## 📊 MQTT Telemetry

### Broker
- **URI**: `mqtt://engf0001.cs.ucl.ac.uk:1883`
- **Topic**: `COMP0221/flock/0/state`
- **QoS**: 1

### Payload Format (JSON)
```json
{
  "version": 1,
  "flags": 0,
  "team_id": 0,
  "node_id": "AA:BB:CC:DD:EE:FF",
  "seq": 0,
  "ts_s": 1234567890,
  "ts_ms": 123,
  "x_mm": 50000,
  "y_mm": 50000,
  "z_mm": 50000,
  "vx_mms": 100,
  "vy_mms": 200,
  "vz_mms": 0,
  "heading_cd": 9000,
  "neighbors": 3
}
```

### Visualization
Subscribe to the topic using any MQTT client:
```bash
mosquitto_sub -h engf0001.cs.ucl.ac.uk -t "COMP0221/flock/0/state" -v
```

## 🔒 Security

### AES-128-CMAC
- **Algorithm**: AES-128-CMAC over packet bytes (excluding MAC tag)
- **Tag**: Truncated to 4 bytes (32 bits)
- **Key**: Configurable in `config.h` (16 bytes)

### Trade-offs
- **4-byte tag**: ~2^32 possible values
  - Security: Resistant to casual spoofing, not cryptographically strong
  - Airtime: Only 4 bytes overhead per packet
  - Verification: Fast on ESP32 (~1ms)

### Changing the Team Key
Edit `config.h`:
```c
#define AES_KEY_BYTES { \
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, \
    0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF  \
}
```

## 📈 Performance Monitoring

The system tracks performance metrics for all tasks:
- **Execution time**: min, max, average
- **Period**: min, max, average (for jitter calculation)
- **Jitter**: max_period - min_period
- **Deadline misses**: periods exceeding target by >10%
- **CPU utilization**: (avg_exec_time / target_period) × 100%

Performance logs are printed every 10 seconds (configurable in `config.h`).

### Example Output
```
=== Performance Stats: Physics ===
  Executions: 5000
  Target period: 20000 us (50.00 Hz)
  Execution time: avg=245 us, min=230 us, max=280 us
  Period: avg=20005 us, min=19998 us, max=20012 us
  Jitter: 14 us (0.07%)
  Deadline misses: 0 (0.00%)
  CPU utilization: 1.23%
```

## 🎮 Flocking Parameters

Tune in `config.h`:
```c
#define COHESION_WEIGHT 1.0f      // Attraction to flock center
#define ALIGNMENT_WEIGHT 1.0f     // Match neighbor velocities
#define SEPARATION_WEIGHT 1.5f    // Avoid collisions (higher = stronger)
#define MIN_SEPARATION_MM 2000    // 2m minimum separation
```

### Behavior Tips
- **Increase separation weight** to avoid collisions
- **Increase cohesion weight** for tighter flocks
- **Increase alignment weight** for synchronized movement
- Test in simulation (e.g., https://eater.net/boids) before deployment

## 🧪 Testing

### Single Drone Test
1. Flash one ESP32
2. Monitor serial output for task timing
3. Subscribe to MQTT topic to see telemetry
4. Verify NTP synchronization

### Multi-Drone Test
1. Flash multiple ESP32s (same team_id)
2. Power on within LoRa range
3. Watch neighbor counts increase
4. Observe flocking behavior in MQTT stream

### Performance Test
1. Enable `ENABLE_PERFORMANCE_LOGGING` in `config.h`
2. Run for several minutes
3. Check logs for jitter and deadline misses
4. Adjust task priorities if needed

## 🛡️ Adversarial Testing

Before conducting adversarial tests:
1. Submit ethics plan via Moodle
2. Coordinate with target team
3. Log all attack details

### Attack Examples
- **Replay attack**: Capture and resend old packets
- **Spoofing**: Forge packets with incorrect MAC
- **Timestamp confusion**: Modify timestamps
- **Flooding**: High-rate transmission (respect duty cycle!)

### Defense Mechanisms
- MAC verification (in `security.c`)
- Sequence number tracking (in `neighbor.c`)
- Timeout pruning (3 second default)
- Team ID filtering

## 📝 Deliverables Checklist

- [ ] **Private GitHub repository** with full codebase
- [ ] **Technical report** (8 pages max)
  - [ ] System architecture diagram
  - [ ] Task timing analysis
  - [ ] Flocking behavior results
  - [ ] Performance measurements
  - [ ] Security evaluation
  - [ ] Adversarial test results
- [ ] **Blog post** (300-500 words) on Moodle
- [ ] **Ethics plan** (submitted before Week 6)

## 🐛 Troubleshooting

### WiFi Connection Issues
- Verify eduroam credentials
- Check network availability
- Try alternative WiFi (modify `wifi_connect_eduroam()`)

### NTP Sync Fails
- Check internet connectivity
- Modify `NTP_SERVER` in `config.h`
- Increase retry count in `init_sntp()`

### MQTT Not Publishing
- Verify broker address
- Check topic name matches specification
- Ensure WiFi connected before MQTT start
- Monitor MQTT events in `mqtt_event_handler_cb()`

### Flocking Not Working
- Check neighbor count (should be > 0)
- Verify LoRa transmission/reception
- Tune flocking weights
- Check neighbor timeout (3s default)

### High Jitter / Deadline Misses
- Reduce WiFi activity
- Adjust task priorities
- Check for blocking operations in tasks
- Increase heap size in menuconfig

## 📚 References

1. Reynolds, C. W. (1987). "Flocks, Herds and Schools: A Distributed Behavioral Model"
2. Semtech AN1200.13: "LoRa Modem Designer's Guide"
3. ESP-IDF Programming Guide: https://docs.espressif.com/projects/esp-idf/

## 👥 Authors

COMP0221 Coursework - UCL  
Academic Year: 2025-2026

## 📄 License

This is coursework for COMP0221. Do not redistribute or copy.
