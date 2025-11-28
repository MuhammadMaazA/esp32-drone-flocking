idf_component_register(
  SRCS 
    "main.cpp"
    "core/cmac.cpp"
    "core/neighbors.cpp"
    "core/packet.cpp"
    "core/metrics.cpp"
    "drivers/radio_lora.cpp"
    "net/mqtt_client.cpp"
    "tasks/task_control.cpp"
    "tasks/task_physics.cpp"
    "tasks/task_radio.cpp"
    "tasks/task_telemetry.cpp"
  INCLUDE_DIRS "."
          REQUIRES jgromes__radiolib esp_driver_gpio esp_timer esp_wifi mqtt esp_netif nvs_flash mbedtls esp_event json esp_http_client
          PRIV_REQUIRES wpa_supplicant
)