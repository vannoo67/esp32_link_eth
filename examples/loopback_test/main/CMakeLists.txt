idf_component_register(
    SRCS "loopback_test.c"
    INCLUDE_DIRS "."
    REQUIRES esp_eth esp_netif esp32_link_eth
)
