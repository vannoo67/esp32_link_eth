# Bringing up loopback_test

Recommended order: **UART first, unflavored, low baud** — it's the one
configuration where both boards run the *identical* binary (no
master/slave role to get wrong), which isolates "does the Component
Manager / build / framing plumbing work at all" from "does the SPI
master/slave handshake work" before you're debugging both at once.

## Stage 1 — UART, no flow control, low baud

Lowest-risk first test: fewest wires, no GPIO role differences, and a
baud rate slow enough that jumper-wire signal integrity is very
unlikely to be the problem if something fails.

1. In `sdkconfig.defaults` (or via `idf.py menuconfig` →
   `Component config → ESP32 Link Ethernet`):
   - Transport: **UART**
   - `ESP32LINK_UART_FLOW_CTRL`: **disabled** (2-wire test first)
   - `ESP32LINK_UART_BAUD`: drop to `115200` for this first pass
2. `idf.py set-target <your chip>` (`esp32c3`, `esp32`, whatever both
   boards use), then `idf.py build`.
3. Flash the **same** binary to both boards:
   `idf.py -p <port_a> flash` and `idf.py -p <port_b> flash`.
4. Wire: **TX(A) → RX(B)**, **RX(A) → TX(B)**, and — easy to
   forget — **GND ↔ GND**. Leave RTS/CTS disconnected for this pass.
5. Open two monitors, one per board's console port (which is a
   *different* UART/USB-JTAG interface than the link pins — don't wire
   the link to the same pins your console/programmer uses):
   `idf.py -p <port_a> monitor` in one terminal, same for board B in
   another.

**Expect to see**, within a couple of seconds on each side:
```
I (xxx) loopback_test: driver started
I (xxx) loopback_test: link up
I (xxx) loopback_test: tx test frame: ESP_OK
I (xxx) loopback_test: rx 60 bytes: dst=ff:ff:ff:ff:ff:ff src=02:00:00:4c:49:6e ethertype=0x88b5
```
repeating every ~2 seconds on both sides.

### If nothing arrives at all
- Check TX/RX are actually crossed, not straight-through (easy to
  mis-wire the first time).
- Check GND is connected — floating ground is the single most common
  "silence" cause on a jumper-wire UART link.
- Check the TX/RX GPIOs you picked don't collide with something else
  on your specific board (strapping pins, onboard LED/button, the
  board's own USB-JTAG console lines). Cross-reference against your
  board's pinout, not just the defaults in `Kconfig`.
- Add `esp_log_level_set("esp32link_mac_uart", ESP_LOG_DEBUG);` near
  the top of `app_main()` temporarily for more visibility into what
  the driver's doing.

### If you see "CRC mismatch, dropping frame" repeatedly
Signal integrity, not a logic bug — shorten wires, double-check GND,
confirm both boards are actually built at the same baud rate.

### If you see "malformed envelope, resyncing"
Possible genuine framing bug in the SLIP decode state machine, or
bytes getting dropped (buffer overrun) — worth checking `UART_RX_BUF_SIZE`
is comfortably larger than a burst, though at one 60-byte frame every
2 seconds this shouldn't be a factor. Flag this one back if you hit it
repeatedly with clean wiring — it points at the decoder, not the wire.

## Stage 2 — UART, raise the baud, enable flow control

Once Stage 1 is clean: bump `ESP32LINK_UART_BAUD` back up (try
921600 → 3000000 in steps rather than jumping straight to 3 Mbaud),
and wire + enable RTS/CTS (`ESP32LINK_UART_FLOW_CTRL`). Watch for CRC
mismatches creeping back in as you raise the rate — that's your signal
integrity ceiling for this wiring, not a driver bug.

## Stage 3 — SPI

Only move here once UART is solid — SPI adds the master/slave role
split and handshake timing on top of everything UART already
exercised.

1. Transport: **SPI**. Build and flash **twice**, with
   `ESP32LINK_SPI_ROLE` set to **Master** for board A and **Slave**
   for board B (`idf.py menuconfig` between builds, or maintain
   separate `sdkconfig.defaults.master` / `.slave` and pass via
   `-D SDKCONFIG_DEFAULTS=...`).
2. Start `ESP32LINK_SPI_CLOCK_MHZ` low (1–5 MHz) for the first pass,
   not the default 20 MHz — rule out timing issues before chasing
   logic bugs.
3. Wire MOSI/MISO/SCLK/CS straight across (not crossed — SPI MOSI is
   always master-out, so master's MOSI goes to slave's MOSI pin
   position on its own bus config, which is how the `spi_bus_config_t`
   / `spi_slave_interface_config_t` role split already handles it),
   plus the handshake GPIO, same number both ends, and GND.
4. If a logic analyzer is available, this is the stage where it earns
   its keep — SPI master/slave timing mismatches are much faster to
   spot on a capture than by log-reading.

### Known rough edge to expect here
The master currently polls on a fixed ~5ms cadence rather than
reacting to the handshake GPIO interrupt (`handshake_isr` in
`esp32link_mac_spi.c` is a placeholder — see the file comment). Don't
be surprised by a few ms of extra round-trip latency versus what the
handshake line implies it should be; that's the known gap, not a new
bug, and is worth fixing once the SPI framing itself is proven
correct.

## Reporting back

If you get stuck at any stage, the most useful things to paste back
are: which stage, the exact log lines from both boards, and — for SPI
— clock speed and role config. That'll usually pin down whether it's
wiring, timing, or a real bug in the framing/decode logic quickly.
