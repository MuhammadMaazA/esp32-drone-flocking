# LoRa Hardware Integration Guide

This guide explains how to integrate actual LoRa hardware (SX1276/SX1278) with the drone flocking system.

## 📌 Overview

The system currently uses a stub implementation in `lora_radio.c`. This guide shows how to replace it with actual LoRa hardware support.

## 🔌 Hardware Connections

### SX1276/SX1278 Pinout

| LoRa Pin | ESP32 GPIO | Function |
|----------|------------|----------|
| MOSI     | 23         | SPI Data Out |
| MISO     | 19         | SPI Data In |
| SCK      | 18         | SPI Clock |
| CS       | 5          | Chip Select |
| RST      | 14         | Reset |
| DIO0     | 26         | RX/TX Done Interrupt |

**Note**: These pins are configured in `config.h` and can be changed.

## 📚 Recommended Libraries

### Option 1: ESP32-LoRa (Sandeep Mistry)
```bash
cd components
git clone https://github.com/Inteform/esp32-lora-library.git lora
```

### Option 2: ESP-IDF LoRa Component
```bash
cd components
git clone https://github.com/nopnop2002/esp-idf-sx127x.git sx127x
```

### Option 3: RadioLib
```bash
cd components
git clone https://github.com/jgromes/RadioLib.git
```

## 🛠️ Step-by-Step Integration

### Step 1: Add Library as Component

Create `components/lora/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "lora.c"
    INCLUDE_DIRS "include"
    REQUIRES driver spi_flash
)
```

### Step 2: Update main/CMakeLists.txt

```cmake
idf_component_register(
    SRCS "main.c" "packet.c" "physics.c" "security.c" 
         "neighbor.c" "performance.c" "lora_radio.c"
    INCLUDE_DIRS "." "include"
    REQUIRES wpa_supplicant esp_wifi nvs_flash esp_netif 
             mqtt mbedtls lwip lora  # Add lora here
)
```

### Step 3: Implement lora_init() in lora_radio.c

Replace the stub implementation:

```c
#include "lora.h"  // Include your LoRa driver header

bool lora_init(void)
{
    ESP_LOGI(TAG, "=== LoRa Radio Initialization ===");
    
    // Initialize SPI bus
    spi_bus_config_t bus_config = {
        .mosi_io_num = LORA_PIN_MOSI,
        .miso_io_num = LORA_PIN_MISO,
        .sclk_io_num = LORA_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    
    esp_err_t ret = spi_bus_initialize(LORA_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialization failed");
        return false;
    }
    
    // Initialize LoRa module
    lora_init_pins(LORA_PIN_CS, LORA_PIN_RST, LORA_PIN_DIO0);
    
    if (lora_init() != 0) {
        ESP_LOGE(TAG, "LoRa initialization failed");
        return false;
    }
    
    // Configure LoRa parameters
    lora_set_frequency(LORA_FREQUENCY);
    lora_set_spreading_factor(LORA_SPREADING_FACTOR);
    lora_set_bandwidth(LORA_BANDWIDTH);
    lora_set_coding_rate(LORA_CODING_RATE);
    lora_set_tx_power(LORA_TX_POWER);
    lora_set_preamble_length(LORA_PREAMBLE_LENGTH);
    lora_set_sync_word(LORA_SYNC_WORD);
    lora_enable_crc();
    
    // Start receiving
    lora_receive();
    
    ESP_LOGI(TAG, "LoRa initialized: %u Hz, SF%d, BW=%u Hz",
             LORA_FREQUENCY, LORA_SPREADING_FACTOR, LORA_BANDWIDTH);
    
    return true;
}
```

### Step 4: Implement lora_transmit()

```c
bool lora_transmit(const uint8_t *buffer, size_t length)
{
    if (buffer == NULL || length == 0) {
        return false;
    }

    // Check if LoRa is ready
    if (!lora_idle()) {
        ESP_LOGW(TAG, "LoRa busy, dropping packet");
        tx_errors++;
        return false;
    }

    // Begin packet
    lora_send_packet(buffer, length);
    
    // Wait for TX done (with timeout)
    int timeout = 100;  // 100ms
    while (!lora_packet_sent() && timeout-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    if (timeout <= 0) {
        ESP_LOGW(TAG, "TX timeout");
        tx_errors++;
        return false;
    }
    
    tx_count++;
    
    // Return to receive mode
    lora_receive();
    
    return true;
}
```

### Step 5: Create RX Task

Add a task to poll for incoming packets:

```c
static void lora_rx_task(void *pvParameters)
{
    uint8_t buffer[256];
    FlockPacket packet;
    uint8_t aes_key[16] = AES_KEY_BYTES;
    
    ESP_LOGI(TAG, "LoRa RX task started");
    
    while (1) {
        // Check for received packet
        int packet_size = lora_receive_packet(buffer, sizeof(buffer));
        
        if (packet_size > 0) {
            rx_count++;
            
            // Get RSSI and SNR
            int rssi = lora_packet_rssi();
            float snr = lora_packet_snr();
            
            // Deserialize packet
            if (deserialize_packet(buffer, packet_size, &packet) != 0) {
                ESP_LOGW(TAG, "Failed to deserialize packet");
                continue;
            }
            
            // Verify MAC signature
            if (!verify_packet(buffer, packet_size - 4, aes_key, packet.mac_tag)) {
                ESP_LOGW(TAG, "Invalid MAC signature, dropping packet");
                continue;
            }
            
            // Call the RX callback (defined in main.c)
            if (rx_callback) {
                rx_callback(&packet, rssi, snr);
            }
        }
        
        // Small delay to avoid busy-wait
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

Then start this task in `lora_init()`:

```c
xTaskCreatePinnedToCore(
    lora_rx_task,
    "lora_rx",
    4096,
    NULL,
    tskIDLE_PRIORITY + 2,
    NULL,
    0
);
```

## 🎛️ LoRa Parameter Selection

### Spreading Factor (SF7-SF12)
- **SF7**: Fastest, shortest range (~2km)
- **SF10**: Good balance (~5km)
- **SF12**: Slowest, longest range (~15km)

**Recommendation**: SF7 for dense swarms, SF9-10 for outdoor

### Bandwidth (125/250/500 kHz)
- **125 kHz**: Longest range, slowest
- **500 kHz**: Shortest range, fastest

**Recommendation**: 125 kHz for long range, 250 kHz for faster updates

### Coding Rate (4/5 to 4/8)
- **4/5**: Less overhead, less error correction
- **4/8**: More overhead, more error correction

**Recommendation**: 4/5 for good link quality

### Frequency
- **Europe**: 868 MHz
- **North America**: 915 MHz
- **Asia**: 433 MHz

**Check local regulations!**

## 📊 Airtime Calculations

At **5 Hz** transmission rate (200ms period), packet airtime should be < 20% of period.

| SF | BW (kHz) | Airtime (ms) | % of Period | Max Hz |
|----|----------|--------------|-------------|--------|
| 7  | 125      | ~40          | 20%         | 5.0    |
| 7  | 250      | ~20          | 10%         | 10.0   |
| 9  | 125      | ~160         | 80%         | 1.25   |
| 10 | 125      | ~320         | 160%        | 0.63   |

**Recommendation**: SF7, BW=250 kHz allows up to 10 Hz transmission

## 🧪 Testing

### 1. Single Node TX Test
```c
// In lora_init(), add test:
uint8_t test_data[] = {0x01, 0x02, 0x03};
lora_transmit(test_data, sizeof(test_data));
ESP_LOGI(TAG, "Test packet transmitted");
```

### 2. Loopback Test
Set up one node as transmitter, one as receiver, verify reception.

### 3. Multi-Node Test
Flash multiple nodes and verify neighbor discovery.

## 🐛 Common Issues

### Issue: LoRa init fails
**Solutions**:
- Check wiring (especially CS, RST)
- Verify SPI bus initialization
- Check module power (3.3V, sufficient current)

### Issue: No packets received
**Solutions**:
- Verify frequency matches (check local regulations)
- Check antenna connection
- Verify both nodes use same SF/BW/CR
- Check sync word matches

### Issue: Packet corruption
**Solutions**:
- Enable CRC (`lora_enable_crc()`)
- Check for interference
- Reduce TX power if too close
- Verify endianness (our packets use big-endian)

### Issue: High packet loss
**Solutions**:
- Increase spreading factor
- Reduce distance between nodes
- Check for obstacles
- Verify antenna orientation

## 📈 Performance Optimization

1. **Use DMA for SPI**: Faster transfers, less CPU usage
2. **Interrupt-driven RX**: No polling overhead
3. **Adjust task priorities**: Balance with other tasks
4. **Buffer management**: Use queues to avoid blocking

## 🔒 Security Considerations

1. **MAC verification**: Always verify packets before processing
2. **Replay protection**: Check sequence numbers
3. **Timestamp validation**: Reject old packets
4. **Team ID filtering**: Ignore packets from other teams

## 📝 Final Checklist

- [ ] LoRa library added to `components/`
- [ ] `lora_init()` implemented and tested
- [ ] `lora_transmit()` sends packets successfully
- [ ] RX task receives and processes packets
- [ ] MAC verification enabled
- [ ] Airtime calculations verified
- [ ] Multi-node test passed
- [ ] Performance metrics look good

## 📚 Additional Resources

- [LoRa Basics](https://lora-developers.semtech.com/documentation/tech-papers-and-guides/lora-and-lorawan/)
- [SX127x Datasheet](https://www.semtech.com/products/wireless-rf/lora-core/sx1276)
- [Airtime Calculator](https://www.loratools.nl/#/airtime)
- [ESP-IDF SPI Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html)

