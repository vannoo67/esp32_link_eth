/*
 * loopback_test.c
 *
 * Minimal standalone sanity check for esp32_link_eth -- no bridging,
 * no lwIP netif, just: bring the link up, log whatever frames arrive,
 * and periodically transmit a small test frame so the other board has
 * something to receive. Flash the SAME build to both boards; set
 * ESP32LINK_SPI_ROLE differently per board (menuconfig) if using SPI.
 *
 * This exists to let you validate/iterate on the transport and framing
 * in esp32_link_eth without pulling in a full router/bridge codebase.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_eth.h"
#include "esp32link_mac.h"
#include "esp32link_phy.h"

static const char *TAG = "loopback_test";

static void eth_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case ETHERNET_EVENT_CONNECTED:    ESP_LOGI(TAG, "link up");    break;
    case ETHERNET_EVENT_DISCONNECTED: ESP_LOGI(TAG, "link down");  break;
    case ETHERNET_EVENT_START:        ESP_LOGI(TAG, "driver started"); break;
    case ETHERNET_EVENT_STOP:         ESP_LOGI(TAG, "driver stopped");  break;
    default: break;
    }
}

/* Raw input path -- bypasses lwIP entirely, just logs what arrives.
 * This is the same mechanism the plain L2 bridge examples use to hand
 * frames to their own forwarding logic instead of a netif. */
static esp_err_t on_frame_received(esp_eth_handle_t eth_handle, uint8_t *buf, uint32_t len, void *priv)
{
    ESP_LOGI(TAG, "rx %u bytes: dst=%02x:%02x:%02x:%02x:%02x:%02x src=%02x:%02x:%02x:%02x:%02x:%02x ethertype=0x%02x%02x",
             (unsigned)len,
             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
             buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
             buf[12], buf[13]);
    free(buf); /* ownership transferred to us by stack_input() */
    return ESP_OK;
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL));

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac;

#if CONFIG_ESP32LINK_TRANSPORT_SPI
    esp32link_spi_config_t link_cfg = ESP32LINK_SPI_DEFAULT_CONFIG();
#if CONFIG_ESP32LINK_SPI_ROLE_MASTER
    link_cfg.is_master = true;
#else
    link_cfg.is_master = false;
#endif
    ESP_LOGI(TAG, "SPI transport, role=%s", link_cfg.is_master ? "master" : "slave");
    mac = esp_eth_mac_new_esp32link_spi(&link_cfg, &mac_config);
#else
    esp32link_uart_config_t link_cfg = ESP32LINK_UART_DEFAULT_CONFIG();
#if CONFIG_ESP32LINK_UART_FLOW_CTRL
    link_cfg.gpio_rts = CONFIG_ESP32LINK_UART_GPIO_RTS;
    link_cfg.gpio_cts = CONFIG_ESP32LINK_UART_GPIO_CTS;
#endif
    ESP_LOGI(TAG, "UART transport, baud=%d", link_cfg.baud_rate);
    mac = esp_eth_mac_new_esp32link_uart(&link_cfg, &mac_config);
#endif

    esp_eth_phy_t *phy = esp_eth_phy_new_esp32link();

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    /* Derive a MAC from this chip's factory-programmed base address
     * (ESP_MAC_ETH applies the same small, deterministic offset the
     * real Ethernet examples use) -- unique per board, and won't
     * collide with this board's WiFi STA MAC if it ever runs WiFi too. */
    uint8_t mac_addr[6];
    ESP_ERROR_CHECK(esp_read_mac(mac_addr, ESP_MAC_ETH));
    ESP_LOGI(TAG, "using MAC %02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr));

    ESP_ERROR_CHECK(esp_eth_update_input_path(eth_handle, on_frame_received, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    uint8_t test_frame[60];
    memset(test_frame, 0, sizeof(test_frame));
    memset(&test_frame[0], 0xFF, 6);            /* broadcast dest */
    memcpy(&test_frame[6], mac_addr, 6);         /* our src */
    test_frame[12] = 0x88; test_frame[13] = 0xB5; /* experimental ethertype */

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_err_t err = esp_eth_transmit(eth_handle, test_frame, sizeof(test_frame));
        ESP_LOGI(TAG, "tx test frame: %s", esp_err_to_name(err));
    }
}
