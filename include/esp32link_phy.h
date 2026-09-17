/*
 * esp32link_phy.h
 *
 * Placeholder PHY for esp32link_mac_*. There is no real PHY chip on
 * this link -- it exists purely because esp_eth's driver model expects
 * a MAC+PHY pair. Once the underlying transport (SPI/UART) reports the
 * link established, this PHY simply says "link up, fixed speed/duplex"
 * and stays there.
 */
#pragma once

#include "esp_eth_phy.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create the placeholder PHY instance.
 *
 * Takes no configuration -- there's nothing to configure on a link
 * that has no real physical-layer negotiation.
 */
esp_eth_phy_t *esp_eth_phy_new_esp32link(void);

#ifdef __cplusplus
}
#endif
