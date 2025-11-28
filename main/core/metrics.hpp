#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"

void metrics_record_phy(TickType_t start, TickType_t end);
void metrics_record_ctrl(TickType_t start, TickType_t end);
void metrics_record_radio(TickType_t start, TickType_t end);
void metrics_record_tele(TickType_t start, TickType_t end);
