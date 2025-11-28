#pragma once
#include <stddef.h>
#include <stdint.h>

// Compute AES-CMAC(128) tag (16B). Returns true on success.
bool cmac_tag16(const uint8_t key16[16], const uint8_t* msg, size_t len,
                uint8_t out16[16]);
