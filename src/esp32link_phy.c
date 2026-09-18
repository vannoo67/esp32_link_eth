/*
 * esp32link_phy.c
 *
 * Minimal PHY stub. No MDIO, no real negotiation -- this link either
 * works (transport up) or doesn't (transport down), so "auto
 * negotiation" is trivial: report link up at a fixed speed/duplex as
 * soon as init() runs, and stay there until del().
 *
 * NOTE: the esp_eth_phy_s vtable has had minor field churn across
 * ESP-IDF releases (e.g. custom_ioctl is newer). Cross-check this
 * against the esp_eth_phy.h shipped with your IDF version -- if the
 * compiler complains about missing/extra initializers, add stub
 * functions for whatever's missing (returning ESP_OK / ESP_ERR_NOT_SUPPORTED
 * as appropriate) rather than removing the designated initializer list.
 */
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_eth_phy.h"
#include "esp_eth_com.h"
#include "esp32link_phy.h"

static const char *TAG = "esp32link_phy";

typedef struct {
    esp_eth_phy_t parent;
    esp_eth_mediator_t *eth;
    eth_link_t link;
    eth_link_t last_reported_link; /*!< what get_link() last actually notified, to suppress redundant re-notification on every poll */
} esp32link_phy_t;

static inline esp32link_phy_t *to_impl(esp_eth_phy_t *phy)
{
    return __containerof(phy, esp32link_phy_t, parent);
}

static esp_err_t phy_set_mediator(esp_eth_phy_t *phy, esp_eth_mediator_t *eth)
{
    ESP_LOGI(TAG, "phy_set_mediator called");
    ESP_RETURN_ON_FALSE(eth, ESP_ERR_INVALID_ARG, TAG, "mediator can't be null");
    to_impl(phy)->eth = eth;
    return ESP_OK;
}

static esp_err_t phy_reset(esp_eth_phy_t *phy)
{
    ESP_LOGI(TAG, "phy_reset called");
    to_impl(phy)->link = ETH_LINK_DOWN;
    return ESP_OK;
}

static esp_err_t phy_reset_hw(esp_eth_phy_t *phy)
{
    ESP_LOGI(TAG, "phy_reset_hw called");
    /* No reset pin -- nothing to do. */
    return ESP_OK;
}

static esp_err_t phy_init(esp_eth_phy_t *phy)
{
    ESP_LOGI(TAG, "phy_init called");
    esp32link_phy_t *impl = to_impl(phy);
    /* NOTE: previously this called on_state_changed(LINK_UP) here,
     * unconditionally, during init(). That happens before
     * esp_eth_start() -- too early for esp_eth core's state machine
     * to react to it, which looks like the reason mac->start() was
     * never reached. Link-up is now only reported from get_link(),
     * which esp_eth core is expected to poll after start() begins. */
    impl->link = ETH_LINK_UP;
    return ESP_OK;
}

static esp_err_t phy_deinit(esp_eth_phy_t *phy)
{
    ESP_LOGI(TAG, "phy_deinit called");
    to_impl(phy)->link = ETH_LINK_DOWN;
    return ESP_OK;
}

static esp_err_t phy_autonego_ctrl(esp_eth_phy_t *phy, eth_phy_autoneg_cmd_t cmd, bool *autonego_en_stat)
{
    ESP_LOGI(TAG, "phy_autonego_ctrl called, cmd=%d", (int)cmd);
    /* There's nothing to negotiate. Report "always enabled, always
     * done" for any query; ignore restart/enable/disable commands. */
    if (autonego_en_stat) {
        *autonego_en_stat = true;
    }
    return ESP_OK;
}

static esp_err_t phy_get_link(esp_eth_phy_t *phy)
{
    esp32link_phy_t *impl = to_impl(phy);
    /* Only notify on an actual state transition, matching phy_set_link
     * below -- this was previously unconditional, meaning esp_eth core
     * polling get_link() every check_link_period_ms re-fired
     * ETH_STATE_LINK/UP once per poll rather than once per real
     * transition, causing ETHERNET_EVENT_CONNECTED to keep re-firing
     * in consuming applications long after the link was already up. */
    if (impl->eth && impl->last_reported_link != impl->link) {
        impl->eth->on_state_changed(impl->eth, ETH_STATE_LINK, (void *)impl->link);
        impl->last_reported_link = impl->link;
    }
    return ESP_OK;
}

static esp_err_t phy_set_link(esp_eth_phy_t *phy, eth_link_t link)
{
    esp32link_phy_t *impl = to_impl(phy);
    if (impl->link != link) {
        impl->link = link;
        if (impl->eth) {
            impl->eth->on_state_changed(impl->eth, ETH_STATE_LINK, (void *)impl->link);
            impl->last_reported_link = impl->link;
        }
    }
    return ESP_OK;
}

static esp_err_t phy_pwrctl(esp_eth_phy_t *phy, bool enable)
{
    ESP_LOGI(TAG, "phy_pwrctl called, enable=%d", (int)enable);
    return ESP_OK; /* nothing to power-gate */
}

static esp_err_t phy_set_addr(esp_eth_phy_t *phy, uint32_t addr)
{
    ESP_LOGI(TAG, "phy_set_addr called");
    return ESP_OK; /* no MDIO address space */
}

static esp_err_t phy_get_addr(esp_eth_phy_t *phy, uint32_t *addr)
{
    ESP_LOGI(TAG, "phy_get_addr called");
    if (addr) {
        *addr = 0;
    }
    return ESP_OK;
}

static esp_err_t phy_advertise_pause_ability(esp_eth_phy_t *phy, uint32_t ability)
{
    ESP_LOGI(TAG, "phy_advertise_pause_ability called");
    return ESP_OK;
}

static esp_err_t phy_loopback(esp_eth_phy_t *phy, bool enable)
{
    ESP_LOGI(TAG, "phy_loopback called, enable=%d", (int)enable);
    return enable ? ESP_ERR_NOT_SUPPORTED : ESP_OK;
}

static esp_err_t phy_set_speed(esp_eth_phy_t *phy, eth_speed_t speed)
{
    ESP_LOGI(TAG, "phy_set_speed called");
    return ESP_OK; /* fixed, nothing to actually set */
}

static esp_err_t phy_set_duplex(esp_eth_phy_t *phy, eth_duplex_t duplex)
{
    ESP_LOGI(TAG, "phy_set_duplex called");
    return ESP_OK; /* fixed, nothing to actually set */
}

static esp_err_t phy_del(esp_eth_phy_t *phy)
{
    ESP_LOGI(TAG, "phy_del called");
    free(to_impl(phy));
    return ESP_OK;
}

esp_eth_phy_t *esp_eth_phy_new_esp32link(void)
{
    esp32link_phy_t *impl = calloc(1, sizeof(esp32link_phy_t));
    if (!impl) {
        ESP_LOGE(TAG, "no memory for PHY instance");
        return NULL;
    }
    impl->link = ETH_LINK_DOWN;
    impl->last_reported_link = ETH_LINK_DOWN;
    impl->parent.set_mediator            = phy_set_mediator;
    impl->parent.reset                   = phy_reset;
    impl->parent.reset_hw                = phy_reset_hw;
    impl->parent.init                    = phy_init;
    impl->parent.deinit                  = phy_deinit;
    impl->parent.autonego_ctrl           = phy_autonego_ctrl;
    impl->parent.get_link                = phy_get_link;
    impl->parent.set_link                = phy_set_link;
    impl->parent.pwrctl                  = phy_pwrctl;
    impl->parent.set_addr                = phy_set_addr;
    impl->parent.get_addr                = phy_get_addr;
    impl->parent.advertise_pause_ability = phy_advertise_pause_ability;
    impl->parent.loopback                = phy_loopback;
    impl->parent.set_speed               = phy_set_speed;
    impl->parent.set_duplex              = phy_set_duplex;
    impl->parent.del                     = phy_del;
    /* impl->parent.custom_ioctl intentionally left NULL -- only present
     * in newer IDF versions and explicitly optional per its docstring. */
    return &impl->parent;
}
