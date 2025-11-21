# Quick Start Guide - 5 Minutes to Running System

## ⚡ Prerequisites
- ESP-IDF v5.0+ installed and configured
- ESP32 board connected via USB
- UCL eduroam credentials

## 🚀 3-Step Setup

### Step 1: Configure Credentials (1 min)
Edit `main/main.c` lines 32-36:
```c
#define EDUROAM_IDENTITY "YOUR-EMAIL@ucl.ac.uk"
#define EDUROAM_USERNAME "YOUR-EMAIL@ucl.ac.uk"
#define EDUROAM_PASSWORD "YOUR-PASSWORD"
```

### Step 2: Build & Flash (2 min)
```bash
cd eduroam_mqtt_test
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### Step 3: Verify (2 min)
Watch for these log messages:
```
✅ WiFi connected: Got IP
✅ Time synchronized with NTP
✅ MQTT connected
✅ Drone initialized at position...
✅ All tasks created successfully
✅ MQTT published: pos=...
```

## 📊 View Your Drone

### Option 1: MQTT Client
```bash
mosquitto_sub -h engf0001.cs.ucl.ac.uk -t "COMP0221/flock/0/state" -v
```

### Option 2: Serial Monitor
Your position/velocity updates appear in ESP32 serial output every 500ms.

## 🎮 What's Happening?

```
┌─────────────────────────────────────────────────┐
│  Physics Task (50 Hz)                           │
│  └─> Updates position based on velocity         │
│                                                  │
│  Flocking Task (10 Hz)                          │
│  └─> Adjusts velocity based on neighbors        │
│                                                  │
│  Radio Task (5 Hz)                              │
│  └─> Sends position to neighbors via LoRa*      │
│                                                  │
│  Telemetry Task (2 Hz)                          │
│  └─> Publishes state to MQTT                    │
└─────────────────────────────────────────────────┘

* LoRa uses stub - see LORA_INTEGRATION_GUIDE.md
```

## 🔧 Common First-Time Issues

### WiFi won't connect
```c
// Try changing timeout in main.c:
xEventGroupWaitBits(s_ev, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, 
                    pdMS_TO_TICKS(60000));  // Increase to 60s
```

### MQTT won't publish
- Check broker address in `config.h`
- Verify topic name: `COMP0221/flock/0/state`
- Ensure WiFi connected before MQTT

### Build errors
```bash
# Clean and rebuild
idf.py fullclean
idf.py build
```

## 🎯 Next Steps

1. ✅ **System Running?** → Read `README.md` for details
2. 🔧 **Add LoRa?** → Follow `LORA_INTEGRATION_GUIDE.md`
3. 📊 **Performance?** → Check logs every 10 seconds
4. 🎓 **Report?** → See `IMPLEMENTATION_STATUS.md`

## 📝 Quick Configuration Changes

### Change task frequencies (in `config.h`):
```c
#define PHYSICS_FREQ_HZ 50     // Physics update rate
#define FLOCKING_FREQ_HZ 10    // Flocking calculation rate
#define RADIO_FREQ_HZ 5        // Radio transmission rate
#define TELEMETRY_FREQ_HZ 2    // MQTT publish rate
```

### Change flocking behavior (in `config.h`):
```c
#define COHESION_WEIGHT 1.0f      // Attraction (increase for tighter flock)
#define ALIGNMENT_WEIGHT 1.0f     // Velocity matching
#define SEPARATION_WEIGHT 1.5f    // Collision avoidance (increase to avoid collisions)
```

### Change team ID (in `config.h`):
```c
#define TEAM_ID 0  // 0 = everyone, or set your team number
```

## 🎉 Success Indicators

You're ready for testing when you see:
- ✅ WiFi IP address obtained
- ✅ NTP time synchronized
- ✅ MQTT publishing every 500ms
- ✅ Performance logs every 10s
- ✅ No deadline misses in performance stats

## 📞 Need Help?

1. Check ESP-IDF logs: `idf.py monitor`
2. Review troubleshooting in `README.md`
3. Verify configuration in `config.h`
4. Check hardware connections (for LoRa)

---

**That's it!** You now have a simulated drone running with real-time physics, flocking, and MQTT telemetry. 🚁

