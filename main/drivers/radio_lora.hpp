#pragma once
#include <stddef.h>
#include <stdint.h>

bool  radio_init();                                     // init SX1276
int   radio_tx(const uint8_t* buf, size_t len);         // RADIOLIB_ERR_*
int   radio_rx(uint8_t* buf, size_t cap, uint32_t ms);  // returns bytes or <0
float radio_rssi();
float radio_snr();
void  radio_get_node_mac(uint8_t out6[6]);              // ESP MAC
