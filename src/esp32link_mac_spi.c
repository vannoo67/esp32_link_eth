/*
 * esp32link_mac_spi.c
 *
 * FIRST-DRAFT skeleton. Structurally wired into esp_eth's MAC vtable
 * and reasonably designed, but not yet bench-validated against real
 * hardware -- expect to iterate on timing, queue depths and the
 * push-vs-pull receive model once you have two boards on the desk.
 * See the top-level README for the framing/wiring summary.
 *
 * RX delivery model: like the built-in ESP32 EMAC and the DM9051/W5500
 * drivers, this uses a "push" model -- a background task owns the SPI
 * transactions and calls eth->stack_input() directly as frames arrive,
 * rather than waiting for something external to call mac->receive().
 * The receive() vtable entry is implemented for completeness but is
 * not exercised by this driver's own task; verify this matches your
 * IDF version's expectations if you see mac->receive() called from
 * elsewhere in the esp_eth core.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_eth_mac.h"
#include "esp_eth_com.h"
#include "esp32link_mac.h"
#include "esp32link_proto.h"

static const char *TAG = "esp32link_mac_spi";

#define TX_QUEUE_DEPTH   8
#define RX_TASK_STACK    4096
#define RX_TASK_PRIO     (configMAX_PRIORITIES - 3)

typedef struct {
    uint8_t *buf;
    uint16_t len;
} tx_item_t;

typedef struct {
    esp_eth_mac_t parent;
    esp_eth_mediator_t *eth;
    esp32link_spi_config_t cfg;
    uint8_t mac_addr[6];

    spi_device_handle_t spi_dev;   /* master role only */
    bool slave_bus_initialized;    /* slave role only  */

    QueueHandle_t tx_queue;        /* holds tx_item_t  */
    TaskHandle_t task;
    volatile bool running;
    SemaphoreHandle_t handshake_sem; /* master role only: given from ISR on handshake rising edge */

    volatile bool link_up;         /*!< MAC-owned link state -- see notify_link() */
    int consecutive_timeouts;      /*!< master/slave task-local; not touched from other tasks */
    int64_t task_start_us;         /*!< this role's task start time, for the initial grace period */
    bool ever_reported_up;         /*!< once true, the grace period no longer applies */
} esp32link_mac_spi_t;

/* Same reasoning and pattern as esp32link_mac_uart.c's notify_link():
 * the MAC owns link-state detection here, not the PHY, since there's
 * no real physical-layer signal to poll. Unlike UART, SPI needs no
 * added keepalive frame -- the master/slave handshake protocol already
 * means the two sides are continuously exchanging transactions (mostly
 * empty ones) as a side effect of normal operation, so "link up" is
 * just "transactions are still completing", detected via the timeouts
 * already present in both roles' wait calls below. */
#define LINK_DOWN_AFTER_CONSECUTIVE_TIMEOUTS  3  /* at ~1s per timeout, matches UART's 3s LINK_TIMEOUT_US */
/* Real hardware finding: reporting link-up the instant the peer is
 * proven alive (which for SPI can be under 20ms) can fire before the
 * consuming application has finished registering its own ETH_EVENT
 * handler -- esp_event does not buffer events for later subscribers,
 * so a notification posted before anything is listening is simply
 * lost. Only the very first "up" transition since boot is delayed by
 * this; real disconnect/reconnect afterward reports at full speed. */
#define INITIAL_LINK_UP_GRACE_US (3000 * 1000)

static void notify_link(esp32link_mac_spi_t *emac, bool up)
{
    if (emac->link_up == up) {
        return;
    }
    emac->link_up = up;
    ESP_LOGI(TAG, "link %s", up ? "up" : "down");
    if (emac->eth) {
        emac->eth->on_state_changed(emac->eth, ETH_STATE_LINK, (void *)(up ? ETH_LINK_UP : ETH_LINK_DOWN));
    }
}

/* Gate for the very first "up" report -- see INITIAL_LINK_UP_GRACE_US
 * above. Subsequent up-transitions (after a real disconnect) bypass
 * this entirely once ever_reported_up is set. */
static inline bool ok_to_report_up(esp32link_mac_spi_t *emac)
{
    return emac->ever_reported_up ||
           (esp_timer_get_time() - emac->task_start_us) >= INITIAL_LINK_UP_GRACE_US;
}

static inline esp32link_mac_spi_t *to_impl(esp_eth_mac_t *mac)
{
    return __containerof(mac, esp32link_mac_spi_t, parent);
}

/* ---- slot encode/decode helpers ---- */

static void encode_slot(esp32link_spi_slot_t *slot, const uint8_t *payload, uint16_t len)
{
    slot->proto_version = ESP32LINK_PROTO_VERSION;
    slot->length = len;
    if (len) {
        memcpy(slot->payload, payload, len);
    }
    /* CRC covers header + payload, not the trailing crc field itself */
    slot->crc16 = esp32link_crc16((const uint8_t *)slot,
                                   offsetof(esp32link_spi_slot_t, payload) + len);
}

/* Returns true if slot contains a validated, non-empty frame */
static bool decode_slot(const esp32link_spi_slot_t *slot, uint8_t **out_buf, uint16_t *out_len)
{
    if (slot->length == 0 || slot->length > ESP32LINK_MAX_FRAME_LEN) {
        return false;
    }
    if (slot->proto_version != ESP32LINK_PROTO_VERSION) {
        ESP_LOGW(TAG, "protocol version mismatch: got %u, expected %u -- check both boards are on the same esp32_link_eth release",
                 slot->proto_version, ESP32LINK_PROTO_VERSION);
        return false;
    }
    uint16_t expect_crc = esp32link_crc16((const uint8_t *)slot,
                                           offsetof(esp32link_spi_slot_t, payload) + slot->length);
    if (expect_crc != slot->crc16) {
        ESP_LOGW(TAG, "CRC mismatch, dropping frame (len=%u)", slot->length);
        return false;
    }
    uint8_t *buf = malloc(slot->length);
    if (!buf) {
        ESP_LOGE(TAG, "no memory for received frame (len=%u)", slot->length);
        return false;
    }
    memcpy(buf, slot->payload, slot->length);
    *out_buf = buf;
    *out_len = slot->length;
    return true;
}

static void deliver_if_present(esp32link_mac_spi_t *emac, const esp32link_spi_slot_t *rx_slot)
{
    uint8_t *buf;
    uint16_t len;
    if (decode_slot(rx_slot, &buf, &len)) {
        if (emac->eth) {
            emac->eth->stack_input(emac->eth, buf, len);
        } else {
            free(buf);
        }
    }
}

/* ---- master role task ---- */

static void IRAM_ATTR handshake_isr(void *arg)
{
    esp32link_mac_spi_t *emac = arg;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(emac->handshake_sem, &woken);
    if (woken) {
        portYIELD_FROM_ISR();
    }
}

static void master_task(void *arg)
{
    esp32link_mac_spi_t *emac = arg;
    ESP_LOGI(TAG, "master_task alive, host=%d clock=%u Hz", emac->cfg.spi_host_id, (unsigned)emac->cfg.clock_speed_hz);
    emac->task_start_us = esp_timer_get_time();
    esp32link_spi_slot_t *tx_slot = heap_caps_malloc(sizeof(esp32link_spi_slot_t), MALLOC_CAP_DMA);
    esp32link_spi_slot_t *rx_slot = heap_caps_malloc(sizeof(esp32link_spi_slot_t), MALLOC_CAP_DMA);
    spi_transaction_t trans;

    /* Startup race: if the slave already armed and raised the line
     * before this ISR was installed (very plausible -- slave's very
     * first arm happens as soon as its own task starts, with no
     * dependency on the master's state), the rising edge that would
     * have given this semaphore already happened and was missed. Prime
     * it manually if the level is already high by the time we get here. */
    if (gpio_get_level(emac->cfg.gpio_handshake) != 0) {
        xSemaphoreGive(emac->handshake_sem);
    }

    emac->consecutive_timeouts = 0;

    while (emac->running) {
        /* Block for a genuine edge, not a polled level -- see the
         * comment on handshake_isr / the file's framing notes for why
         * level-polling here is unreliable (can't distinguish "stale
         * high from the previous transaction" from "freshly armed"). */
        if (xSemaphoreTake(emac->handshake_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
            /* The slave normally re-arms and signals within
             * milliseconds under all conditions, even idle -- a full
             * 1s of silence here is already abnormal. Still require a
             * few in a row before declaring down, so one transient
             * scheduling hiccup doesn't flap the link state. */
            if (++emac->consecutive_timeouts >= LINK_DOWN_AFTER_CONSECUTIVE_TIMEOUTS) {
                notify_link(emac, false);
            }
            continue;
        }

        tx_item_t item;
        bool have_tx = xQueueReceive(emac->tx_queue, &item, 0) == pdTRUE;

        if (have_tx) {
            encode_slot(tx_slot, item.buf, item.len);
            free(item.buf);
        } else {
            encode_slot(tx_slot, NULL, 0);
        }

        memset(&trans, 0, sizeof(trans));
        trans.length = ESP32LINK_SPI_SLOT_SIZE * 8; /* bits */
        trans.tx_buffer = tx_slot;
        trans.rx_buffer = rx_slot;

        esp_err_t err = spi_device_transmit(emac->spi_dev, &trans);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "spi_device_transmit failed: %s", esp_err_to_name(err));
            continue;
        }
        /* A completed transaction -- post-handshake-fix -- can only
         * happen if the slave was genuinely armed and participating,
         * so this is solid proof of life. */
        emac->consecutive_timeouts = 0;
        if (ok_to_report_up(emac)) {
            notify_link(emac, true);
            emac->ever_reported_up = true;
        }
        deliver_if_present(emac, rx_slot);
    }

    heap_caps_free(tx_slot);
    heap_caps_free(rx_slot);
    vTaskDelete(NULL);
}

/* ---- slave role task ---- */

static void slave_task(void *arg)
{
    esp32link_mac_spi_t *emac = arg;
    ESP_LOGI(TAG, "slave_task alive, host=%d", emac->cfg.spi_host_id);
    emac->task_start_us = esp_timer_get_time();
    esp32link_spi_slot_t *tx_slot = heap_caps_malloc(sizeof(esp32link_spi_slot_t), MALLOC_CAP_DMA);
    /* Two RX buffers, ping-ponged: while one is being processed in
     * software, the other is safe for the hardware/DMA to fill for
     * the transaction we re-arm immediately below -- re-arming before
     * processing (rather than after) is what minimizes the window
     * where the slave isn't ready for the master's next transaction. */
    esp32link_spi_slot_t *rx_bufs[2] = {
        heap_caps_malloc(sizeof(esp32link_spi_slot_t), MALLOC_CAP_DMA),
        heap_caps_malloc(sizeof(esp32link_spi_slot_t), MALLOC_CAP_DMA),
    };
    int active = 0;
    tx_item_t item;

    /* Arm the very first transaction before the loop starts, then
     * signal ready. */
    encode_slot(tx_slot, NULL, 0);
    spi_slave_transaction_t trans = {
        .length = ESP32LINK_SPI_SLOT_SIZE * 8,
        .tx_buffer = tx_slot,
        .rx_buffer = rx_bufs[active],
    };
    spi_slave_queue_trans(emac->cfg.spi_host_id, &trans, portMAX_DELAY);
    gpio_set_level(emac->cfg.gpio_handshake, 1);

    emac->consecutive_timeouts = 0;

    while (emac->running) {
        spi_slave_transaction_t *result = NULL;
        esp_err_t err = spi_slave_get_trans_result(emac->cfg.spi_host_id, &result, pdMS_TO_TICKS(1000));
        if (err != ESP_OK || !result) {
            /* A full 1s with the master never clocking a transaction
             * we're already armed for is abnormal under normal
             * operation. Require a few in a row before declaring
             * down, same reasoning as the master's side. */
            if (++emac->consecutive_timeouts >= LINK_DOWN_AFTER_CONSECUTIVE_TIMEOUTS) {
                notify_link(emac, false);
            }
            continue; /* still armed and waiting -- handshake stays high */
        }
        emac->consecutive_timeouts = 0;
        if (ok_to_report_up(emac)) {
            notify_link(emac, true);
            emac->ever_reported_up = true;
        }
        gpio_set_level(emac->cfg.gpio_handshake, 0); /* consumed; briefly not ready while we re-arm */

        int just_completed = active;
        active = 1 - active;

        bool have_tx = xQueueReceive(emac->tx_queue, &item, 0) == pdTRUE;
        if (have_tx) {
            encode_slot(tx_slot, item.buf, item.len);
            free(item.buf);
        } else {
            encode_slot(tx_slot, NULL, 0);
        }

        spi_slave_transaction_t next_trans = {
            .length = ESP32LINK_SPI_SLOT_SIZE * 8,
            .tx_buffer = tx_slot,
            .rx_buffer = rx_bufs[active],
        };
        spi_slave_queue_trans(emac->cfg.spi_host_id, &next_trans, pdMS_TO_TICKS(50));
        gpio_set_level(emac->cfg.gpio_handshake, 1); /* re-armed, ready again */

        /* Safe to process now -- this buffer is no longer the one the
         * hardware is filling for the next transaction. */
        deliver_if_present(emac, rx_bufs[just_completed]);
    }

    heap_caps_free(tx_slot);
    heap_caps_free(rx_bufs[0]);
    heap_caps_free(rx_bufs[1]);
    vTaskDelete(NULL);
}

/* ---- esp_eth_mac_s vtable ---- */

static esp_err_t begin_running(esp32link_mac_spi_t *emac)
{
    if (emac->running) {
        return ESP_OK;
    }
    emac->running = true;
    BaseType_t ok = xTaskCreate(emac->cfg.is_master ? master_task : slave_task,
                                 "esp32link_spi", RX_TASK_STACK, emac, RX_TASK_PRIO, &emac->task);
    if (ok != pdPASS) {
        emac->running = false;
        ESP_LOGE(TAG, "failed to create link task");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "link task created (handle=%p, role=%s)", (void *)emac->task,
             emac->cfg.is_master ? "master" : "slave");
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
    esp32link_mac_spi_t *emac = to_impl(mac);
    const esp32link_spi_config_t *cfg = &emac->cfg;

    /* Master: pull-DOWN, not up -- default level must read as "not
     * ready" until the slave actively drives it high. A pull-up here
     * would make the master think the (not-yet-started) slave was
     * ready from the moment the master's own GPIO came up, causing it
     * to clock a transaction before the slave has armed anything. */
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << cfg->gpio_handshake,
        .mode = cfg->is_master ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = cfg->is_master ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "handshake gpio config failed");

    if (cfg->is_master) {
        spi_bus_config_t buscfg = {
            .mosi_io_num = cfg->gpio_mosi,
            .miso_io_num = cfg->gpio_miso,
            .sclk_io_num = cfg->gpio_sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = (int)ESP32LINK_SPI_SLOT_SIZE,
        };
        ESP_RETURN_ON_ERROR(spi_bus_initialize(cfg->spi_host_id, &buscfg, SPI_DMA_CH_AUTO),
                             TAG, "spi_bus_initialize failed");

        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = (int)cfg->clock_speed_hz,
            /* Mode 0 (CPOL=0, CPHA=0) -- tried mode 1 as an experiment
             * to buy the slave more output-settling margin at higher
             * clock speeds; it made things reliably worse (broke even
             * previously-clean 10 MHz), consistent with known ESP32
             * SPI *slave*-mode quirks around CPHA=1. Stick with mode 0. */
            .mode = 0,
            .spics_io_num = cfg->gpio_cs,
            .queue_size = 1,
        };
        ESP_RETURN_ON_ERROR(spi_bus_add_device(cfg->spi_host_id, &devcfg, &emac->spi_dev),
                             TAG, "spi_bus_add_device failed");

        esp_err_t isr_err = gpio_install_isr_service(0);
        ESP_RETURN_ON_FALSE(isr_err == ESP_OK || isr_err == ESP_ERR_INVALID_STATE, isr_err,
                             TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(isr_err));
        ESP_RETURN_ON_ERROR(gpio_set_intr_type(cfg->gpio_handshake, GPIO_INTR_POSEDGE),
                             TAG, "gpio_set_intr_type failed");
        ESP_RETURN_ON_ERROR(gpio_isr_handler_add(cfg->gpio_handshake, handshake_isr, emac),
                             TAG, "gpio_isr_handler_add failed");
    } else {
        spi_bus_config_t buscfg = {
            .mosi_io_num = cfg->gpio_mosi,
            .miso_io_num = cfg->gpio_miso,
            .sclk_io_num = cfg->gpio_sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        };
        spi_slave_interface_config_t slvcfg = {
            .mode = 0, /* see comment on the master's devcfg.mode above */
            .spics_io_num = cfg->gpio_cs,
            .queue_size = 1,
        };
        ESP_RETURN_ON_ERROR(spi_slave_initialize(cfg->spi_host_id, &buscfg, &slvcfg, SPI_DMA_CH_AUTO),
                             TAG, "spi_slave_initialize failed");
        emac->slave_bus_initialized = true;
    }

    /* Task created here, not in mac_start() -- see esp32link_mac_uart.c
     * for why: observed behavior is esp_eth_start() doesn't call
     * mac->start() during normal bring-up on IDF v5.5.5. */
    return begin_running(emac);
}

static esp_err_t mac_deinit(esp_eth_mac_t *mac)
{
    ESP_LOGI(TAG, "mac_deinit called");
    esp32link_mac_spi_t *emac = to_impl(mac);
    emac->running = false;
    vTaskDelay(pdMS_TO_TICKS(100)); /* let the link task exit before we tear down the bus */
    if (emac->cfg.is_master && emac->spi_dev) {
        gpio_isr_handler_remove(emac->cfg.gpio_handshake);
        spi_bus_remove_device(emac->spi_dev);
        spi_bus_free(emac->cfg.spi_host_id);
        emac->spi_dev = NULL;
    } else if (!emac->cfg.is_master && emac->slave_bus_initialized) {
        spi_slave_free(emac->cfg.spi_host_id);
        emac->slave_bus_initialized = false;
    }
    return ESP_OK;
}

static esp_err_t mac_start(esp_eth_mac_t *mac)
{
    ESP_LOGI(TAG, "mac_start called");
    return begin_running(to_impl(mac));
}

static esp_err_t mac_stop(esp_eth_mac_t *mac)
{
    esp32link_mac_spi_t *emac = to_impl(mac);
    emac->running = false; /* task exits its own loop and self-deletes */
    return ESP_OK;
}

static esp_err_t mac_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    esp32link_mac_spi_t *emac = to_impl(mac);
    ESP_RETURN_ON_FALSE(length <= ESP32LINK_MAX_FRAME_LEN, ESP_ERR_INVALID_SIZE, TAG, "frame too large");

    uint8_t *copy = malloc(length);
    ESP_RETURN_ON_FALSE(copy, ESP_ERR_NO_MEM, TAG, "no memory to queue tx frame");
    memcpy(copy, buf, length);

    tx_item_t item = { .buf = copy, .len = (uint16_t)length };
    if (xQueueSend(emac->tx_queue, &item, pdMS_TO_TICKS(20)) != pdTRUE) {
        free(copy);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static esp_err_t mac_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{
    /* Not used by this driver's own push-model task -- see file header
     * comment. Present for vtable completeness / API compatibility. */
    (void)mac; (void)buf;
    if (length) {
        *length = 0;
    }
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

/* No selective filtering hardware -- every frame that crosses the
 * link is already delivered upward regardless of destination address,
 * so "add/remove a filter entry" is inherently a no-op success: the
 * address would have been received and passed up either way. Without
 * these, esp_eth's core logs "add mac address to filter not
 * supported" whenever lwIP tries to join a multicast group (IGMP,
 * mDNS, IPv6 neighbor discovery, DHCP server's multicast use, etc.). */
static esp_err_t mac_add_mac_filter(esp_eth_mac_t *mac, uint8_t *addr) { (void)mac; (void)addr; return ESP_OK; }
static esp_err_t mac_rm_mac_filter(esp_eth_mac_t *mac, uint8_t *addr) { (void)mac; (void)addr; return ESP_OK; }

static esp_err_t mac_read_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value)
{
    /* No MDIO bus -- there's no real PHY to read. */
    if (reg_value) { *reg_value = 0; }
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t mac_write_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t mac_del(esp_eth_mac_t *mac)
{
    esp32link_mac_spi_t *emac = to_impl(mac);
    if (emac->tx_queue) {
        vQueueDelete(emac->tx_queue);
    }
    if (emac->handshake_sem) {
        vSemaphoreDelete(emac->handshake_sem);
    }
    free(emac);
    return ESP_OK;
}

esp_eth_mac_t *esp_eth_mac_new_esp32link_spi(const esp32link_spi_config_t *link_config,
                                              const eth_mac_config_t *mac_config)
{
    esp32link_mac_spi_t *emac = calloc(1, sizeof(esp32link_mac_spi_t));
    if (!emac) {
        ESP_LOGE(TAG, "no memory for MAC instance");
        return NULL;
    }
    emac->cfg = *link_config;
    emac->tx_queue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(tx_item_t));
    emac->handshake_sem = xSemaphoreCreateBinary();
    if (!emac->tx_queue || !emac->handshake_sem) {
        ESP_LOGE(TAG, "no memory for tx queue / handshake semaphore");
        if (emac->tx_queue) { vQueueDelete(emac->tx_queue); }
        if (emac->handshake_sem) { vSemaphoreDelete(emac->handshake_sem); }
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
    /* emac->parent.transmit_vargs / transmit_ctrl_vargs intentionally
     * left NULL -- add if your IDF version's esp_eth core calls them
     * unconditionally; check esp_eth_mac.h for your release. */

    (void)mac_config; /* nothing chip-specific to apply from here */
    return &emac->parent;
}
