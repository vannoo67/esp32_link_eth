/*
 * esp32link_mac_uart.c
 *
 * FIRST-DRAFT skeleton -- see the note at the top of esp32link_mac_spi.c;
 * the same caveats (push-model RX, verify vtable fields against your
 * IDF version) apply here.
 *
 * Framing: [version(1)][len_hi][len_lo][payload(len)][crc_hi][crc_lo],
 * SLIP-style byte-stuffed (ESP32LINK_SLIP_END/ESC, not IP-only SLIP --
 * this carries the whole Ethernet frame including its header), framed
 * by an END byte on each side. Unlike SPI, UART is naturally full
 * duplex and symmetric -- no master/slave role to configure.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_eth_mac.h"
#include "esp_eth_com.h"
#include "esp32link_mac.h"
#include "esp32link_proto.h"

static const char *TAG = "esp32link_mac_uart";

#define UART_RX_BUF_SIZE   4096
#define UART_TX_BUF_SIZE   4096
#define RX_TASK_STACK      4096
#define RX_TASK_PRIO       (configMAX_PRIORITIES - 3)
/* Envelope before stuffing: version + len(2) + payload + crc(2) */
#define ENVELOPE_MAX       (1 + 2 + ESP32LINK_MAX_FRAME_LEN + 2)

typedef struct {
    esp_eth_mac_t parent;
    esp_eth_mediator_t *eth;
    esp32link_uart_config_t cfg;
    uint8_t mac_addr[6];
    SemaphoreHandle_t tx_lock;
    TaskHandle_t task;
    volatile bool running;
} esp32link_mac_uart_t;

static inline esp32link_mac_uart_t *to_impl(esp_eth_mac_t *mac)
{
    return __containerof(mac, esp32link_mac_uart_t, parent);
}

/* ---- SLIP encode/send ---- */

static esp_err_t send_stuffed_byte(int uart_port, uint8_t b)
{
    uint8_t out[2];
    int n;
    if (b == ESP32LINK_SLIP_END) {
        out[0] = ESP32LINK_SLIP_ESC; out[1] = ESP32LINK_SLIP_ESC_END; n = 2;
    } else if (b == ESP32LINK_SLIP_ESC) {
        out[0] = ESP32LINK_SLIP_ESC; out[1] = ESP32LINK_SLIP_ESC_ESC; n = 2;
    } else {
        out[0] = b; n = 1;
    }
    return (uart_write_bytes(uart_port, (const char *)out, n) == n) ? ESP_OK : ESP_FAIL;
}

static esp_err_t uart_send_frame(esp32link_mac_uart_t *emac, const uint8_t *payload, uint16_t len)
{
    uint8_t envelope[ENVELOPE_MAX];
    envelope[0] = ESP32LINK_PROTO_VERSION;
    envelope[1] = (uint8_t)(len >> 8);
    envelope[2] = (uint8_t)(len & 0xFF);
    memcpy(&envelope[3], payload, len);
    uint16_t crc = esp32link_crc16(envelope, 3 + len);
    envelope[3 + len]     = (uint8_t)(crc >> 8);
    envelope[3 + len + 1] = (uint8_t)(crc & 0xFF);
    size_t total = 3 + len + 2;

    xSemaphoreTake(emac->tx_lock, portMAX_DELAY);
    uint8_t end = ESP32LINK_SLIP_END;
    uart_write_bytes(emac->cfg.uart_port, (const char *)&end, 1);
    for (size_t i = 0; i < total; i++) {
        send_stuffed_byte(emac->cfg.uart_port, envelope[i]);
    }
    uart_write_bytes(emac->cfg.uart_port, (const char *)&end, 1);
    xSemaphoreGive(emac->tx_lock);
    return ESP_OK;
}

/* ---- SLIP decode / RX task ---- */

static void deliver_envelope(esp32link_mac_uart_t *emac, const uint8_t *envelope, size_t total)
{
    if (total < 5) { /* version + len(2) + crc(2), zero-length payload allowed but pointless */
        return;
    }
    uint8_t version = envelope[0];
    uint16_t len = ((uint16_t)envelope[1] << 8) | envelope[2];
    if (version != ESP32LINK_PROTO_VERSION) {
        ESP_LOGW(TAG, "protocol version mismatch: got %u, expected %u", version, ESP32LINK_PROTO_VERSION);
        return;
    }
    if (len > ESP32LINK_MAX_FRAME_LEN || total != (size_t)(3 + len + 2)) {
        ESP_LOGW(TAG, "malformed envelope (len=%u, total=%u)", len, (unsigned)total);
        return;
    }
    uint16_t expect_crc = esp32link_crc16(envelope, 3 + len);
    uint16_t got_crc = ((uint16_t)envelope[3 + len] << 8) | envelope[3 + len + 1];
    if (expect_crc != got_crc) {
        ESP_LOGW(TAG, "CRC mismatch, dropping frame (len=%u)", len);
        return;
    }
    uint8_t *buf = malloc(len);
    if (!buf) {
        ESP_LOGE(TAG, "no memory for received frame (len=%u)", len);
        return;
    }
    memcpy(buf, &envelope[3], len);
    if (emac->eth) {
        emac->eth->stack_input(emac->eth, buf, len);
    } else {
        free(buf);
    }
}

static void rx_task(void *arg)
{
    esp32link_mac_uart_t *emac = arg;
    uint8_t envelope[ENVELOPE_MAX];
    size_t elen = 0;
    bool in_frame = false;
    bool escaping = false;
    uint8_t byte;

    ESP_LOGI(TAG, "rx_task alive, reading uart port %d", emac->cfg.uart_port);

    while (emac->running) {
        int n = uart_read_bytes(emac->cfg.uart_port, &byte, 1, pdMS_TO_TICKS(50));
        if (n <= 0) {
            continue;
        }
        if (byte == ESP32LINK_SLIP_END) {
            if (in_frame && elen > 0) {
                deliver_envelope(emac, envelope, elen);
            }
            elen = 0;
            in_frame = true;
            escaping = false;
            continue;
        }
        if (!in_frame) {
            continue; /* resync: wait for the next END */
        }
        if (escaping) {
            escaping = false;
            if (byte == ESP32LINK_SLIP_ESC_END) {
                byte = ESP32LINK_SLIP_END;
            } else if (byte == ESP32LINK_SLIP_ESC_ESC) {
                byte = ESP32LINK_SLIP_ESC;
            } else {
                ESP_LOGW(TAG, "bad escape sequence, resyncing");
                in_frame = false;
                elen = 0;
                continue;
            }
        } else if (byte == ESP32LINK_SLIP_ESC) {
            escaping = true;
            continue;
        }
        if (elen < sizeof(envelope)) {
            envelope[elen++] = byte;
        } else {
            ESP_LOGW(TAG, "envelope overflow, resyncing");
            in_frame = false;
            elen = 0;
        }
    }
    vTaskDelete(NULL);
}

/* ---- esp_eth_mac_s vtable ---- */

static esp_err_t begin_running(esp32link_mac_uart_t *emac)
{
    if (emac->running) {
        return ESP_OK;
    }
    emac->running = true;
    BaseType_t ok = xTaskCreate(rx_task, "esp32link_uart_rx", RX_TASK_STACK, emac, RX_TASK_PRIO, &emac->task);
    if (ok != pdPASS) {
        emac->running = false;
        ESP_LOGE(TAG, "failed to create rx task");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "rx task created (handle=%p)", (void *)emac->task);
    return ESP_OK;
}

static esp_err_t mac_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *eth)
{
    ESP_LOGI(TAG, "mac_set_mediator called");
    ESP_RETURN_ON_FALSE(eth, ESP_ERR_INVALID_ARG, TAG, "mediator can't be null");
    to_impl(mac)->eth = eth;
    return ESP_OK;
}

static esp_err_t mac_init(esp_eth_mac_t *mac)
{
    ESP_LOGI(TAG, "mac_init called");
    esp32link_mac_uart_t *emac = to_impl(mac);
    const esp32link_uart_config_t *cfg = &emac->cfg;

    uart_config_t uart_cfg = {
        .baud_rate = cfg->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = (cfg->gpio_rts >= 0 && cfg->gpio_cts >= 0)
                         ? UART_HW_FLOWCTRL_CTS_RTS : UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 120,
    };
    ESP_RETURN_ON_ERROR(uart_param_config(cfg->uart_port, &uart_cfg), TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(cfg->uart_port, cfg->gpio_tx, cfg->gpio_rx,
                                      cfg->gpio_rts >= 0 ? cfg->gpio_rts : UART_PIN_NO_CHANGE,
                                      cfg->gpio_cts >= 0 ? cfg->gpio_cts : UART_PIN_NO_CHANGE),
                         TAG, "uart_set_pin failed");
    ESP_RETURN_ON_ERROR(uart_driver_install(cfg->uart_port, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 0, NULL, 0),
                         TAG, "uart_driver_install failed");
    /* The RX task is created here, not in mac_start() -- observed
     * behavior on IDF v5.5.5 is that esp_eth_start() does not call
     * mac->start() during normal bring-up, only mac->init() runs
     * unconditionally. mac_start() below still exists and is still
     * safe to call (idempotent), in case some other call path does
     * invoke it. */
    return begin_running(emac);
}

static esp_err_t mac_deinit(esp_eth_mac_t *mac)
{
    ESP_LOGI(TAG, "mac_deinit called");
    esp32link_mac_uart_t *emac = to_impl(mac);
    emac->running = false;
    vTaskDelay(pdMS_TO_TICKS(100)); /* let rx_task exit before we pull the uart driver out from under it */
    uart_driver_delete(emac->cfg.uart_port);
    return ESP_OK;
}

static esp_err_t mac_start(esp_eth_mac_t *mac)
{
    ESP_LOGI(TAG, "mac_start called");
    return begin_running(to_impl(mac));
}

static esp_err_t mac_stop(esp_eth_mac_t *mac)
{
    ESP_LOGI(TAG, "mac_stop called");
    to_impl(mac)->running = false;
    return ESP_OK;
}

static esp_err_t mac_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    esp32link_mac_uart_t *emac = to_impl(mac);
    ESP_RETURN_ON_FALSE(length <= ESP32LINK_MAX_FRAME_LEN, ESP_ERR_INVALID_SIZE, TAG, "frame too large");
    return uart_send_frame(emac, buf, (uint16_t)length);
}

static esp_err_t mac_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{
    /* Push-model driver -- see file header comment. */
    (void)mac; (void)buf;
    if (length) { *length = 0; }
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t mac_set_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    memcpy(to_impl(mac)->mac_addr, addr, 6);
    return ESP_OK;
}

static esp_err_t mac_get_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    memcpy(addr, to_impl(mac)->mac_addr, 6);
    return ESP_OK;
}

static esp_err_t mac_set_speed(esp_eth_mac_t *mac, eth_speed_t speed)   { return ESP_OK; }
static esp_err_t mac_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex) { return ESP_OK; }
static esp_err_t mac_set_link(esp_eth_mac_t *mac, eth_link_t link)      { return ESP_OK; }
static esp_err_t mac_set_promiscuous(esp_eth_mac_t *mac, bool enable)   { return ESP_OK; }
static esp_err_t mac_set_peer_pause_ability(esp_eth_mac_t *mac, uint32_t ability) { return ESP_OK; }
static esp_err_t mac_enable_flow_ctrl(esp_eth_mac_t *mac, bool enable)  { return ESP_OK; }
static esp_err_t mac_set_all_multicast(esp_eth_mac_t *mac, bool enable) { return ESP_OK; }

/* See the matching comment in esp32link_mac_spi.c -- same reasoning
 * applies here. */
static esp_err_t mac_add_mac_filter(esp_eth_mac_t *mac, uint8_t *addr) { (void)mac; (void)addr; return ESP_OK; }
static esp_err_t mac_rm_mac_filter(esp_eth_mac_t *mac, uint8_t *addr) { (void)mac; (void)addr; return ESP_OK; }
static esp_err_t mac_read_phy_reg(esp_eth_mac_t *mac, uint32_t a, uint32_t r, uint32_t *v) { if (v) *v = 0; return ESP_ERR_NOT_SUPPORTED; }
static esp_err_t mac_write_phy_reg(esp_eth_mac_t *mac, uint32_t a, uint32_t r, uint32_t v) { return ESP_ERR_NOT_SUPPORTED; }

static esp_err_t mac_del(esp_eth_mac_t *mac)
{
    esp32link_mac_uart_t *emac = to_impl(mac);
    if (emac->tx_lock) {
        vSemaphoreDelete(emac->tx_lock);
    }
    free(emac);
    return ESP_OK;
}

esp_eth_mac_t *esp_eth_mac_new_esp32link_uart(const esp32link_uart_config_t *link_config,
                                               const eth_mac_config_t *mac_config)
{
    esp32link_mac_uart_t *emac = calloc(1, sizeof(esp32link_mac_uart_t));
    if (!emac) {
        ESP_LOGE(TAG, "no memory for MAC instance");
        return NULL;
    }
    emac->cfg = *link_config;
    emac->tx_lock = xSemaphoreCreateMutex();
    if (!emac->tx_lock) {
        ESP_LOGE(TAG, "no memory for tx lock");
        free(emac);
        return NULL;
    }

    emac->parent.set_mediator            = mac_set_mediator;
    emac->parent.init                    = mac_init;
    emac->parent.deinit                  = mac_deinit;
    emac->parent.start                   = mac_start;
    emac->parent.stop                    = mac_stop;
    emac->parent.transmit                = mac_transmit;
    emac->parent.receive                 = mac_receive;
    emac->parent.set_addr                = mac_set_addr;
    emac->parent.get_addr                = mac_get_addr;
    emac->parent.set_speed               = mac_set_speed;
    emac->parent.set_duplex              = mac_set_duplex;
    emac->parent.set_link                = mac_set_link;
    emac->parent.set_promiscuous         = mac_set_promiscuous;
    emac->parent.set_peer_pause_ability  = mac_set_peer_pause_ability;
    emac->parent.enable_flow_ctrl        = mac_enable_flow_ctrl;
    emac->parent.set_all_multicast       = mac_set_all_multicast;
    emac->parent.add_mac_filter          = mac_add_mac_filter;
    emac->parent.rm_mac_filter           = mac_rm_mac_filter;
    emac->parent.read_phy_reg            = mac_read_phy_reg;
    emac->parent.write_phy_reg           = mac_write_phy_reg;
    emac->parent.del                     = mac_del;

    (void)mac_config;
    return &emac->parent;
}
