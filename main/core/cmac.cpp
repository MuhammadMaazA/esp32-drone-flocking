#include "core/cmac.hpp"
#include <mbedtls/cipher.h>
#include <mbedtls/cmac.h>

bool cmac_tag16(const uint8_t key16[16], const uint8_t* msg, size_t len,
                uint8_t out16[16]){
  const mbedtls_cipher_info_t* ci =
      mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
  if(!ci) return false;

  mbedtls_cipher_context_t ctx;
  mbedtls_cipher_init(&ctx);
  if(mbedtls_cipher_setup(&ctx, ci) != 0){
    mbedtls_cipher_free(&ctx);
    return false;
  }
  
  // Use CMAC API: starts -> update -> finish
  if(mbedtls_cipher_cmac_starts(&ctx, key16, 128) != 0){
    mbedtls_cipher_free(&ctx);
    return false;
  }
  if(mbedtls_cipher_cmac_update(&ctx, msg, len) != 0){
    mbedtls_cipher_free(&ctx);
    return false;
  }
  int rc = mbedtls_cipher_cmac_finish(&ctx, out16);
  mbedtls_cipher_free(&ctx);
  return rc == 0;
}
