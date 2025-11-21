#ifndef SECURITY_H
#define SECURITY_H

#include <stdint.h>
#include <stddef.h>

/**
 * Sign a packet using AES-128-CMAC
 * @param buffer Pointer to packet buffer (excluding the 4-byte tag)
 * @param len Length of the buffer to sign
 * @param key 16-byte AES key
 * @param tag_out Output buffer for 4-byte truncated CMAC tag
 */
void sign_packet(uint8_t *buffer, size_t len, uint8_t *key, uint8_t *tag_out);

/**
 * Verify a packet signature using AES-128-CMAC
 * @param buffer Pointer to packet buffer (excluding the 4-byte tag)
 * @param len Length of the buffer to verify
 * @param key 16-byte AES key
 * @param tag_in 4-byte CMAC tag to verify against
 * @return 1 if valid, 0 if invalid
 */
int verify_packet(uint8_t *buffer, size_t len, uint8_t *key, uint8_t *tag_in);

#endif // SECURITY_H
