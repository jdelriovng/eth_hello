idf_component_register(
    SRCS "eth_hello_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_event esp_netif esp_eth esp_http_server nvs_flash esp_driver_gpio esp_driver_i2c app_update
)