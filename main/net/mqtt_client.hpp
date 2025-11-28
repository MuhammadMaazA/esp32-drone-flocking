#pragma once
#include <stdint.h>
#include <stddef.h>
#include "core/state.hpp"

// Initialise MQTT client (blocking until connected)
bool mqtt_init();

// Publish a JSON telemetry message
bool mqtt_publish_state(const State& s, const char* node_label);

// Publish heartbeat message (for testing/cooperation)
bool mqtt_publish_heartbeat();

// Check if MQTT is connected
bool mqtt_is_connected();
