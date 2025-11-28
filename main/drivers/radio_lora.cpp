#include "drivers/radio_lora.hpp"
#include "config/params.hpp"
#include "EspHal.h"
#include <RadioLib.h>
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// TTGO v2.1 pins - using definitions from params.hpp
static EspHal* hal = new EspHal(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN);
static SX1276 lora = new Module(hal, LORA_CS_PIN, LORA_DIO0_PIN, LORA_RST_PIN, LORA_DIO1_PIN);

bool radio_init(){
  // SX1276 begin() signature: freq, bw, sf, cr, syncWord, power, preambleLength, gain
  // Note: CRC is set separately, not in begin()
  int rc = lora.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                      LORA_SYNCWORD, LORA_POWER_DBM, LORA_PREAMBLE, 0);
  if (rc != RADIOLIB_ERR_NONE) {
    ESP_LOGE("radio", "lora.begin() failed with error code: %d", rc);
    return false;
  }
  // Set CRC separately (if needed - RadioLib may set it automatically based on begin params)
  if (LORA_CRC_ON) {
    rc = lora.setCRC(2);  // 2 = CRC enabled
    if (rc != RADIOLIB_ERR_NONE) {
      ESP_LOGW("radio", "lora.setCRC() failed with error code: %d", rc);
    }
  }
  lora.setOutputPower(LORA_POWER_DBM);
  ESP_LOGI("radio", "Radio initialized: freq=%.1f MHz, bw=%.1f kHz, sf=%u, cr=%u, power=%d dBm",
           LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_POWER_DBM);
  return true;
}

int radio_tx(const uint8_t* buf, size_t len){
  return lora.transmit((uint8_t*)buf, len);
}

int radio_rx(uint8_t* buf, size_t cap, uint32_t timeout_ms){
  // Use blocking lora.receive() but with short timeouts in a loop to allow frequent yields
  // This gives us the blocking behavior while preventing watchdog timeouts
  TickType_t start_ticks = xTaskGetTickCount();
  TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  
  while ((xTaskGetTickCount() - start_ticks) < timeout_ticks){
    // Use very short timeout (1ms) per receive call to allow frequent yields
    // This ensures the IDLE task can run and reset the watchdog
    RadioLibTime_t short_timeout_us = 1000; // 1ms in microseconds (reduced from 2ms)
    
    int state = lora.receive(buf, cap, short_timeout_us);
    
    if (state == RADIOLIB_ERR_NONE) {
      // Packet received successfully
      size_t rx_len = lora.getPacketLength();
      if (rx_len > cap) rx_len = cap;  // Safety check
      return (int)rx_len;
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      // Short timeout expired, check if we still have time and yield
      // Yield frequently to allow IDLE task to run and reset watchdog
      taskYIELD();
      continue; // Try again if we still have time
    } else {
      // Some other error occurred
      ESP_LOGD("radio", "Receive failed, code %d", state);
      return -1;
    }
  }
  return 0; // Overall timeout
}

float radio_rssi(){ return lora.getRSSI(); }
float radio_snr() { return lora.getSNR();  }

void radio_get_node_mac(uint8_t out6[6]){
  esp_read_mac(out6, ESP_MAC_WIFI_STA);
}
