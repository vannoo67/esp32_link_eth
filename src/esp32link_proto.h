/*
 * esp32link_proto.h  (private / internal header)
 *
 * Shared framing definitions for both transports. This is a first-draft
 * protocol: simple, easy to reason about, not yet bandwidth-optimized.
 * Iterate here once you have hardware timing numbers.
 *
 * SPI framing
 * -----------
 * Fixed-size full-duplex transactions of ESP32LINK_SPI_SLOT_SIZE bytes.
 * Either side may have "nothing to send" in a given slot; the header's
 * length field of 0 means "no frame here", and the receiver just
 * discards the slot. Master-initiated transactions happen either when
 * the master has a frame queued for TX, or when the handshake GPIO
 * tells it the slave has one waiting.
 *
 * UART framing
 * ------------
 * SLIP-style byte-stuffing (not IP-only SLIP -- this carries the whole
 * Ethernet frame, header included) with a trailing CRC16 so corrupted
 * frames are dropped rather than handed upstream.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP32LINK_PROTO_VERSION   ((uint8_t)CONFIG_ESP32LINK_PROTOCOL_VERSION)

/* Largest frame we ever move: standard 1500 MTU + Ethernet header (14)
 * + a little headroom for VLAN tags etc. Round up. */
#define ESP32LINK_MAX_FRAME_LEN   1522

/* ---- SPI: fixed-size slot ---- */
#pragma pack(push, 1)
typedef struct {
    uint8_t  proto_version;
    uint16_t length;            /* 0 == empty slot */
    uint8_t  payload[ESP32LINK_MAX_FRAME_LEN];
    uint16_t crc16;             /* over proto_version + length + payload[0..length) */
} esp32link_spi_slot_t;
#pragma pack(pop)

#define ESP32LINK_SPI_SLOT_SIZE  sizeof(esp32link_spi_slot_t)

/* ---- UART: SLIP-style framing ---- */
#define ESP32LINK_SLIP_END       0xC0
#define ESP32LINK_SLIP_ESC       0xDB
#define ESP32LINK_SLIP_ESC_END   0xDC
#define ESP32LINK_SLIP_ESC_ESC   0xDD

/**
 * @brief CRC16-CCITT (0xFFFF init, poly 0x1021). Used to validate
 *        frames on both transports before handing them upstream.
 */
uint16_t esp32link_crc16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
