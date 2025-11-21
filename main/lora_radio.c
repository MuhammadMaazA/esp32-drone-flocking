#include "lora_radio.h"
#include "config.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "lora";

// LoRa statistics
static uint32_t tx_count = 0;
static uint32_t rx_count = 0;
static uint32_t tx_errors = 0;

// Receive callback
static lora_rx_callback_t rx_callback = NULL;

/**
 * Initialize LoRa radio
 * 
 * NOTE: This is a STUB implementation!
 * 
 * To integrate with actual LoRa hardware (e.g., SX1276/SX1278):
 * 1. Include the appropriate driver library
 * 2. Initialize SPI bus with pins from config.h
 * 3. Configure LoRa parameters (frequency, SF, BW, CR, etc.)
 * 4. Set up interrupt handling for DIO0 (RX done)
 * 5. Enable receive mode
 * 
 * Example using ESP32-LoRa library:
 *   lora_init_spi(LORA_PIN_MOSI, LORA_PIN_MISO, LORA_PIN_SCK, LORA_PIN_CS);
 *   lora_set_frequency(LORA_FREQUENCY);
 *   lora_set_spreading_factor(LORA_SPREADING_FACTOR);
 *   lora_set_bandwidth(LORA_BANDWIDTH);
 *   lora_set_coding_rate(LORA_CODING_RATE);
 *   lora_set_tx_power(LORA_TX_POWER);
 *   lora_enable_crc();
 */
bool lora_init(void)
{
    ESP_LOGI(TAG, "=== LoRa Radio Initialization (STUB) ===");
    ESP_LOGW(TAG, "LoRa hardware not configured - using stub implementation");
    ESP_LOGI(TAG, "To enable LoRa, integrate SX127x driver and update lora_radio.c");
    ESP_LOGI(TAG, "Configuration from config.h:");
    ESP_LOGI(TAG, "  Frequency: %u Hz", LORA_FREQUENCY);
    ESP_LOGI(TAG, "  Spreading Factor: %d", LORA_SPREADING_FACTOR);
    ESP_LOGI(TAG, "  Bandwidth: %u Hz", LORA_BANDWIDTH);
    ESP_LOGI(TAG, "  Coding Rate: 4/%d", LORA_CODING_RATE);
    ESP_LOGI(TAG, "  TX Power: %d dBm", LORA_TX_POWER);
    
    // Calculate and display expected airtime for our packets
    uint32_t airtime = lora_calculate_airtime_ms(sizeof(FlockPacket));
    ESP_LOGI(TAG, "  Packet airtime: ~%u ms", airtime);
    ESP_LOGI(TAG, "  Max packets/min at %dHz: %.1f (airtime ~%.1f%% of period)",
             RADIO_FREQ_HZ,
             (60.0f * RADIO_FREQ_HZ),
             (100.0f * airtime * RADIO_FREQ_HZ) / 1000.0f);
    
    // TODO: Initialize actual LoRa hardware here
    // For now, return true to allow system to run without hardware
    return true;
}

void lora_set_rx_callback(lora_rx_callback_t callback)
{
    rx_callback = callback;
    ESP_LOGI(TAG, "RX callback registered");
}

/**
 * Transmit a packet via LoRa
 * 
 * NOTE: This is a STUB implementation!
 * 
 * To integrate with actual LoRa hardware:
 * 1. Check if radio is idle/ready
 * 2. Write packet to FIFO
 * 3. Start transmission
 * 4. Wait for TX done or use interrupt
 * 
 * Example:
 *   if (!lora_is_transmitting()) {
 *       lora_write_fifo(buffer, length);
 *       lora_send_packet();
 *   }
 */
bool lora_transmit(const uint8_t *buffer, size_t length)
{
    if (buffer == NULL || length == 0) {
        return false;
    }

    // TODO: Implement actual LoRa transmission
    // For now, just log and increment counter
    ESP_LOGD(TAG, "TX stub: %u bytes (seq in buffer)", length);
    tx_count++;
    
    return true;
}

bool lora_is_ready(void)
{
    // TODO: Check actual LoRa hardware status
    // For stub, always return true
    return true;
}

void lora_get_stats(uint32_t *tx_count_out, uint32_t *rx_count_out, uint32_t *tx_errors_out)
{
    if (tx_count_out) *tx_count_out = tx_count;
    if (rx_count_out) *rx_count_out = rx_count;
    if (tx_errors_out) *tx_errors_out = tx_errors;
}

/**
 * Calculate LoRa airtime using formula from Semtech AN1200.13
 * 
 * Symbol duration: Ts = (2^SF) / BW
 * Preamble time: Tpreamble = (Npreamble + 4.25) * Ts
 * Payload symbols: Npayload = 8 + max(ceil[(8*PL - 4*SF + 28 + 16*CRC - 20*IH) / (4*(SF - 2*DE))] * CR, 0)
 * Payload time: Tpayload = Npayload * Ts
 * Total time: Tpacket = Tpreamble + Tpayload
 */
uint32_t lora_calculate_airtime_ms(size_t payload_length)
{
    // Parameters from config.h
    int sf = LORA_SPREADING_FACTOR;
    int bw = LORA_BANDWIDTH;
    int cr = LORA_CODING_RATE;  // 4/5 -> cr = 5
    int preamble = LORA_PREAMBLE_LENGTH;
    
    // Fixed parameters
    int crc = 1;  // CRC enabled
    int ih = 0;   // Explicit header
    int de = (sf >= 11) ? 1 : 0;  // Low data rate optimization for SF11/12
    
    // Symbol duration in milliseconds
    float ts = (powf(2.0f, (float)sf) / (float)bw) * 1000.0f;
    
    // Preamble time
    float t_preamble = ((float)preamble + 4.25f) * ts;
    
    // Payload symbol count
    float numerator = 8.0f * (float)payload_length - 4.0f * (float)sf + 28.0f + 16.0f * (float)crc - 20.0f * (float)ih;
    float denominator = 4.0f * ((float)sf - 2.0f * (float)de);
    float payload_symbols = 8.0f + ceilf(numerator / denominator) * (float)cr;
    if (payload_symbols < 8.0f) payload_symbols = 8.0f;
    
    // Payload time
    float t_payload = payload_symbols * ts;
    
    // Total airtime
    float t_packet = t_preamble + t_payload;
    
    return (uint32_t)ceilf(t_packet);
}

