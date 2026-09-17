# esp32_link_eth

An `esp_eth` MAC/PHY driver pair that links two ESP32 boards directly
over SPI or UART, standing in for a W5500 module + Ethernet cable.
Drop it in anywhere a chip-specific driver like `esp_eth_mac_new_w5500()`
/ `esp_eth_phy_new_w5500()` is normally used — everything above the
MAC/PHY layer (lwIP netif glue, L2 bridging, ProxyARP) needs no changes.

**Status: first-draft skeleton.** The build-system plumbing (component
manifest, Kconfig, vtable wiring) is solid; the transport/framing
implementation compiles against the documented `esp_eth` APIs but has
not yet been validated on real hardware. Expect to iterate on timing,
queue depths, and the exact `esp_eth_mac_s` / `esp_eth_phy_s` field
list against whatever ESP-IDF version you're building with — see the
comments in `src/*.c` for the specific spots most likely to need
adjustment.

## Why two transports

| | SPI | UART |
|---|---|---|
| Roles | Asymmetric (one master, one slave) | Symmetric, no roles |
| Wiring | 5 lines (MOSI/MISO/SCLK/CS/handshake) | 2–4 lines (TX/RX, optional RTS/CTS) |
| Throughput | Tens of Mbps realistic | Low single-digit Mbps after framing overhead |
| Best for | General bridging, anything that needs to carry real WiFi-client traffic | Light/IoT traffic, longer or messier wiring runs |

## Wiring

### SPI

| Signal | Master GPIO (example) | Slave GPIO (example) |
|---|---|---|
| MOSI | `CONFIG_ESP32LINK_SPI_GPIO_MOSI` | same number, direct wire |
| MISO | `CONFIG_ESP32LINK_SPI_GPIO_MISO` | same number, direct wire |
| SCLK | `CONFIG_ESP32LINK_SPI_GPIO_SCLK` | same number, direct wire |
| CS | `CONFIG_ESP32LINK_SPI_GPIO_CS` | same number, direct wire |
| Handshake | `CONFIG_ESP32LINK_SPI_GPIO_HANDSHAKE` (input) | same number (output) |

Keep wires short (a few cm to ~20–30cm with care) and matched-length;
add a pull-up on CS if you see bus glitches. Start at a conservative
clock (`ESP32LINK_SPI_CLOCK_MHZ`, default 20 MHz) on jumper wiring and
raise it once the link is proven reliable.

### UART

TX↔RX crossed between boards, plus RTS↔CTS crossed if using hardware
flow control (recommended — see `ESP32LINK_UART_FLOW_CTRL`).

## Wire protocol (v1)

Deliberately simple; optimize once you have real timing numbers.

- **SPI**: fixed-size full-duplex "slots" of
  `sizeof(esp32link_spi_slot_t)` bytes: `[version(1)][length(2)][payload(≤1522)][crc16(2)]`.
  A `length` of 0 means "nothing in this slot". The slave re-arms the
  next transaction *before* processing the one it just received, then
  raises the handshake line; the master blocks on a genuine
  interrupt-driven edge on that line (not a polled level -- a level
  read can't distinguish "still high from the previous transaction,
  slave just hasn't gotten scheduled yet to lower it" from "freshly
  armed", which caused real, reproducible data corruption on real
  hardware in earlier drafts of this driver). The ESP32 SPI slave
  peripheral must be pre-armed before the master starts clocking, or
  the transaction's data is undefined, so this handshake is a hard
  correctness requirement, not a latency optimization.
- **UART**: SLIP-style byte-stuffed frames — `0xC0` (END) delimited,
  `0xDB` (ESC) escaping — wrapping the same
  `[version(1)][length(2)][payload][crc16(2)]` envelope. This is *not*
  the usual IP-only SLIP: it carries the whole Ethernet frame,
  destination/source MAC and all, which is required for ARP and
  broadcast frames to survive the link.
- Both ends check `ESP32LINK_PROTOCOL_VERSION` (Kconfig, `src/esp32link_proto.h`)
  and drop anything that doesn't match, rather than risk silently
  corrupting frames after a one-sided update. Bump this whenever you
  change the framing, and keep both boards' component versions pinned
  together (see "Packaging" below).

## Using it in a project

```yaml
# main/idf_component.yml
dependencies:
  esp32_link_eth:
    git: "https://github.com/vannoo67/esp32_link_eth.git"
    version: "v0.1.0"
```

And in `main/CMakeLists.txt`, add it to `REQUIRES` so its public headers
are visible to your code -- the manifest above only handles fetching;
CMake still needs telling explicitly:

```cmake
idf_component_register(
    SRCS ...
    REQUIRES esp32_link_eth
)
```

```c
#include "esp32link_mac.h"
#include "esp32link_phy.h"

eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();

#if CONFIG_ETH_DOWNLINK_ESP32LINK
#if CONFIG_ESP32LINK_TRANSPORT_SPI
    esp32link_spi_config_t link_cfg = ESP32LINK_SPI_DEFAULT_CONFIG();
    link_cfg.is_master = CONFIG_ESP32LINK_SPI_ROLE_MASTER; /* set per board */
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32link_spi(&link_cfg, &mac_config);
#else
    esp32link_uart_config_t link_cfg = ESP32LINK_UART_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32link_uart(&link_cfg, &mac_config);
#endif
    esp_eth_phy_t *phy = esp_eth_phy_new_esp32link();
#endif

esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
esp_eth_handle_t eth_handle;
esp_eth_driver_install(&eth_config, &eth_handle);
```

See `examples/loopback_test` for a complete, minimal standalone app —
useful for validating the link between two boards before wiring it
into a full bridge/router codebase.

## Packaging: two consumers, one shared component

This exists as its own repo specifically so both
[`esp32_ethernet_router`](https://github.com/martin-ger/esp32_ethernet_router)
and [`esp32_eth_wifi_bridge`](https://github.com/martin-ger/esp32_eth_wifi_bridge)
forks can depend on the exact same transport/framing code via the ESP-IDF
Component Manager, rather than maintaining two copies that have to be
kept in sync by hand. Tag releases with semver and pin both forks to
the same tag — a version drift between the two ends of this link isn't
just "a bug got fixed," it can mean the framing itself no longer
matches on both sides.

During active development, point at a local checkout instead of Git
via `override_path` in the consuming project's manifest, so you're not
pushing a commit for every iteration:

```yaml
dependencies:
  esp32_link_eth:
    git: "https://github.com/vannoo67/esp32_link_eth.git"
    version: "v0.1.0"
overrides:
  - esp32_link_eth:
      with:
        path: "/path/to/local/esp32_link_eth"
```

## Known gaps / next steps

- SPI clock ceiling: clean up to ~12 MHz on hookup-wire jumper wiring,
  corrupted reads from the slave side starting at 15 MHz — consistent
  with ESP32's SPI *slave* mode having a lower reliable clock ceiling
  than master mode (the slave's output settling time becomes the
  bottleneck as clock rises, not the master's sampling). Likely has
  more headroom with shorter wires or a real PCB trace, not yet tested.
  Tried switching both sides to SPI mode 1 (CPHA=1) hoping for extra
  settling margin -- this made things reliably *worse*, breaking even
  the previously-clean 10 MHz case, consistent with known ESP32 SPI
  slave-mode quirks around CPHA=1. Stick with mode 0; if you need more
  headroom than 10 MHz, look at wiring quality (specifically the MISO
  line) before touching SPI mode again.
- No retry/ack layer — a dropped or corrupted frame (bad CRC, version
  mismatch) is just discarded, same as a real Ethernet link would do
  under packet loss; upper-layer protocols (TCP, ARP retries) are
  expected to cover for it, but this hasn't been stress-tested under
  induced errors yet.
- `esp_eth_mac_s` / `esp_eth_phy_s` vtable fields are filled in against
  the documented ESP-IDF v5.x API surface, but that struct has had
  minor churn across releases. If your build complains about
  missing/extra designated initializers, that's the first place to
  look. One instance of this already found and fixed on real hardware:
  `add_mac_filter`/`rm_mac_filter` (added to the MAC vtable at some
  point after the version this driver was first written against)
  weren't implemented, causing harmless but noisy
  `"add mac address to filter not supported"` errors whenever lwIP
  tried to join a multicast group (IGMP, mDNS, IPv6 neighbor
  discovery, DHCP). Both are now no-op successes, which is correct
  behavior here: this driver has no selective filtering hardware, so
  every frame crossing the link is delivered upward regardless of
  destination address already -- "adding a filter" doesn't need to do
  anything.
