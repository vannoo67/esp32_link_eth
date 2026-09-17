/*
 * esp32link_mac.h
 *
 * esp_eth-compatible MAC driver pair that moves whole Ethernet frames
 * directly between two ESP32 boards over SPI or UART, in place of a
 * W5500 + physical Ethernet cable.
 *
 * Usage mirrors the stock chip drivers, e.g. esp_eth_mac_new_w5500():
 * construct a MAC with one of the functions below, pair it with
 * esp_eth_phy_new_esp32link() (see esp32link_phy.h), and hand both to
 * esp_eth_driver_install() as normal. Everything above the MAC/PHY
 * layer -- lwIP netif glue, bridging, ProxyARP -- is unaffected.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_eth_mac.h"
#include "esp_eth_com.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration for the SPI variant of the link MAC.
 */
typedef struct {
    int spi_host_id;          /*!< SPI host to use. Ignored fields below apply per role. */
    int gpio_mosi;
    int gpio_miso;
    int gpio_sclk;
    int gpio_cs;
    int gpio_handshake;       /*!< Slave -> master "frame ready" line, same GPIO number both ends */
    uint32_t clock_speed_hz;  /*!< Master role only; ignored when is_master == false */
    bool is_master;           /*!< Set explicitly per board -- see Kconfig ESP32LINK_SPI_ROLE */
} esp32link_spi_config_t;

/**
 * @brief Configuration for the UART variant of the link MAC.
 */
typedef struct {
    int uart_port;
    int gpio_tx;
    int gpio_rx;
    int gpio_rts;              /*!< -1 to disable hardware flow control */
    int gpio_cts;              /*!< -1 to disable hardware flow control */
    int baud_rate;
} esp32link_uart_config_t;

/**
 * @brief Default SPI link config. Caller MUST still set .is_master
 *        explicitly (see Kconfig ESP32LINK_SPI_ROLE on each board):
 *
 *          esp32link_spi_config_t cfg = ESP32LINK_SPI_DEFAULT_CONFIG();
 *      #if CONFIG_ESP32LINK_SPI_ROLE_MASTER
 *          cfg.is_master = true;
 *      #else
 *          cfg.is_master = false;
 *      #endif
 */
#define ESP32LINK_SPI_DEFAULT_CONFIG() \
    { \
        .spi_host_id    = CONFIG_ESP32LINK_SPI_HOST, \
        .gpio_mosi      = CONFIG_ESP32LINK_SPI_GPIO_MOSI, \
        .gpio_miso      = CONFIG_ESP32LINK_SPI_GPIO_MISO, \
        .gpio_sclk      = CONFIG_ESP32LINK_SPI_GPIO_SCLK, \
        .gpio_cs        = CONFIG_ESP32LINK_SPI_GPIO_CS, \
        .gpio_handshake = CONFIG_ESP32LINK_SPI_GPIO_HANDSHAKE, \
        .clock_speed_hz = (uint32_t)CONFIG_ESP32LINK_SPI_CLOCK_MHZ * 1000U * 1000U, \
        .is_master      = true, \
    }

#define ESP32LINK_UART_DEFAULT_CONFIG() \
    { \
        .uart_port = CONFIG_ESP32LINK_UART_PORT, \
        .gpio_tx   = CONFIG_ESP32LINK_UART_GPIO_TX, \
        .gpio_rx   = CONFIG_ESP32LINK_UART_GPIO_RX, \
        .gpio_rts  = -1, \
        .gpio_cts  = -1, \
        .baud_rate = CONFIG_ESP32LINK_UART_BAUD, \
    }

/**
 * @brief Create an Ethernet MAC instance backed by a direct SPI link to
 *        a peer ESP32 running the matching driver (opposite role).
 *
 * @param link_config SPI link configuration
 * @param mac_config   Generic esp_eth MAC configuration (as passed to
 *                      any other esp_eth_mac_new_* constructor)
 * @return MAC instance, or NULL on failure
 */
esp_eth_mac_t *esp_eth_mac_new_esp32link_spi(const esp32link_spi_config_t *link_config,
                                              const eth_mac_config_t *mac_config);

/**
 * @brief Create an Ethernet MAC instance backed by a direct UART link
 *        to a peer ESP32 running the matching driver.
 */
esp_eth_mac_t *esp_eth_mac_new_esp32link_uart(const esp32link_uart_config_t *link_config,
                                               const eth_mac_config_t *mac_config);

#ifdef __cplusplus
}
#endif
