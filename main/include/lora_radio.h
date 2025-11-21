#ifndef LORA_RADIO_H
#define LORA_RADIO_H

#include <stdint.h>
#include <stdbool.h>
#include "packet.h"

/**
 * LoRa radio callback for received packets
 * @param packet Pointer to received packet
 * @param rssi RSSI value (signal strength)
 * @param snr SNR value (signal-to-noise ratio)
 */
typedef void (*lora_rx_callback_t)(const FlockPacket *packet, int rssi, float snr);

/**
 * Initialize LoRa radio
 * @return true on success, false on failure
 */
bool lora_init(void);

/**
 * Set receive callback for incoming packets
 * @param callback Callback function to handle received packets
 */
void lora_set_rx_callback(lora_rx_callback_t callback);

/**
 * Transmit a packet via LoRa
 * @param buffer Pointer to packet buffer
 * @param length Length of packet in bytes
 * @return true on success, false on failure
 */
bool lora_transmit(const uint8_t *buffer, size_t length);

/**
 * Check if LoRa radio is ready to transmit
 * @return true if ready, false otherwise
 */
bool lora_is_ready(void);

/**
 * Get LoRa radio statistics
 * @param tx_count Output: Number of packets transmitted
 * @param rx_count Output: Number of packets received
 * @param tx_errors Output: Number of transmission errors
 */
void lora_get_stats(uint32_t *tx_count, uint32_t *rx_count, uint32_t *tx_errors);

/**
 * Calculate approximate airtime for a packet
 * @param payload_length Length of payload in bytes
 * @return Airtime in milliseconds
 */
uint32_t lora_calculate_airtime_ms(size_t payload_length);

#endif // LORA_RADIO_H

