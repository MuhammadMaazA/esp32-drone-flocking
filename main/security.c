#include "security.h"
#include <string.h>
#include "mbedtls/cipher.h"
#include "esp_log.h"

static const char *TAG = "security";

void sign_packet(uint8_t *buffer, size_t len, uint8_t *key, uint8_t *tag_out)
{
    if (buffer == NULL || key == NULL || tag_out == NULL || len == 0) {
        ESP_LOGE(TAG, "Invalid parameters for sign_packet");
        return;
    }

    mbedtls_cipher_context_t ctx;
    mbedtls_cipher_init(&ctx);

    // Setup AES-128-CMAC
    const mbedtls_cipher_info_t *cipher_info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
    if (cipher_info == NULL) {
        ESP_LOGE(TAG, "Failed to get cipher info");
        mbedtls_cipher_free(&ctx);
        return;
    }

    if (mbedtls_cipher_setup(&ctx, cipher_info) != 0) {
        ESP_LOGE(TAG, "Failed to setup cipher");
        mbedtls_cipher_free(&ctx);
        return;
    }

    if (mbedtls_cipher_cmac_starts(&ctx, key, 128) != 0) {
        ESP_LOGE(TAG, "Failed to start CMAC");
        mbedtls_cipher_free(&ctx);
        return;
    }

    // Update with packet data
    if (mbedtls_cipher_cmac_update(&ctx, buffer, len) != 0) {
        ESP_LOGE(TAG, "Failed to update CMAC");
        mbedtls_cipher_free(&ctx);
        return;
    }

    // Finish and get full CMAC (16 bytes)
    uint8_t full_tag[16];
    if (mbedtls_cipher_cmac_finish(&ctx, full_tag) != 0) {
        ESP_LOGE(TAG, "Failed to finish CMAC");
        mbedtls_cipher_free(&ctx);
        return;
    }

    // Truncate to first 4 bytes
    memcpy(tag_out, full_tag, 4);

    mbedtls_cipher_free(&ctx);
}

int verify_packet(uint8_t *buffer, size_t len, uint8_t *key, uint8_t *tag_in)
{
    if (buffer == NULL || key == NULL || tag_in == NULL || len == 0) {
        ESP_LOGE(TAG, "Invalid parameters for verify_packet");
        return 0;
    }

    // Calculate expected tag
    uint8_t expected_tag[4];
    sign_packet(buffer, len, key, expected_tag);

    // Compare tags
    if (memcmp(expected_tag, tag_in, 4) == 0) {
        return 1; // Valid
    }

    return 0; // Invalid
}
