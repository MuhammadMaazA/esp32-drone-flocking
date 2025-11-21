idf_component_register(
    SRCS "main.c" "packet.c" "physics.c" "security.c" "neighbor.c" "performance.c" "lora_radio.c"
    INCLUDE_DIRS "." "include"
    REQUIRES wpa_supplicant esp_wifi nvs_flash esp_netif mqtt mbedtls lwip
)
