# Hardware Workflow

This folder is for real wearable device input.

Near-term target:

```text
BNO085 on each ESP32-S3
    -> fused quaternion report
    -> 40-byte UDP packet
    -> Wi-Fi UDP over the shared phone hotspot
    -> laptop or other receiver on the same hotspot
    -> sensor/ normalization layer
```

The BNO085 can provide fused quaternion reports directly. We should preserve
the raw reported quaternion and metadata, then normalize ordering and frame
meaning in `sensor/`.

Responsibilities:

- connect to physical sensors
- assign stable `sensor_id` values to physical ESP32-S3 boards
- assign `segment_id` values to body placements for the current session
- read quaternion reports
- attach timestamps and quality/status information
- pass packets forward without doing IK or calibration

## Network Architecture

Use direct ESP32-S3 to receiver UDP over the phone hotspot first:

```text
ESP32-S3 pelvis      \
ESP32-S3 left thigh   \
ESP32-S3 left shank    \
ESP32-S3 left foot      -> laptop UDP receiver
ESP32-S3 right thigh   /
ESP32-S3 right shank  /
ESP32-S3 right foot  /
```

The pelvis ESP32-S3 is not the hub in the current design. The phone hotspot only
provides Wi-Fi connectivity; each ESP32 sends its own UDP packets to the
receiver IP.

## ESP32 Firmware

The production firmware lives in `esp32_bno085_udp/`. The other `esp32s3_*`
sketches in this folder (`esp32s3_led_sanity/`, `esp32s3_bno085_uart_sanity/`,
`esp32s3_bno085_ble_sanity/`) are throwaway bring-up tests and are not part of
the streaming path; this document covers only the UDP firmware.

Use Arduino IDE and `.ino` for hardware bring-up. It is enough for this stage:
we need reliable sensor reads, Wi-Fi, UDP packets, and fast iteration. If the
firmware grows into shared drivers later, the packet contract should stay the
same.

## Per-Board Configuration

Each board needs a unique `sensor_id` and `segment_id`, plus the Wi-Fi
credentials and receiver IP. These have compiled-in defaults but are stored in
flash (NVS) and can be changed at runtime over the Serial Monitor at `115200`
baud — no reflash needed. Send `help` to list the commands:

```text
ssid <name>      set Wi-Fi SSID          (reboot to apply)
pass <password>  set Wi-Fi password      (reboot to apply)
ip <a.b.c.d>     set UDP receiver IP     (applies live)
port <n>         set UDP receiver port   (applies live)
sensor <n>       set sensor id           (applies live)
segment <n>      set segment id          (applies live)
show             print current config
clear            erase stored config, revert to compiled-in defaults
reboot           restart the board
```

Each set is saved to flash immediately. SSID/password changes need a `reboot`
to reconnect; IP, port, sensor, and segment apply live. Saved values survive
reflashing the firmware — only `clear` or a full flash erase resets them.

The compiled-in defaults still live as `#define`s at the top of the sketch
(`SENSOR_ID`, `SEGMENT_ID`, `RECEIVER_IP`, `RECEIVER_PORT`, `WIFI_SSID`,
`WIFI_PASSWORD`, `BNO08X_TX_PIN`, `BNO08X_RX_PIN`, `BNO08X_UART_BAUD`).
`RECEIVER_IP` is a dotted-decimal string, e.g. `"192.168.1.50"`. The pin and baud values
are compile-time only; the rest are just the fallback used before anything is
stored in flash. Copy `esp32_bno085_udp/secrets.example.h` to `secrets.h` to
set Wi-Fi defaults locally without editing the sketch. `secrets.h` is ignored
by Git.

Segment IDs (each board's onboard RGB LED is colored from its `segment_id`, so
you can tell at a glance which placement a board is configured for):

| ID | Segment | LED color | RGB |
|---|---|---|---|
| 0 | pelvis | white | `255,255,255` |
| 1 | left_thigh | green | `0,220,80` |
| 2 | left_shank | yellow | `255,230,0` |
| 3 | left_foot | blue | `0,60,255` |
| 4 | right_thigh | orange | `255,140,0` |
| 5 | right_shank | purple | `190,70,255` |
| 6 | right_foot | red | `255,40,40` |
| 255 | unknown | gray | `80,80,80` |

The color updates live when you set `segment <n>` over serial, so it doubles as
a quick confirmation that the segment id took.

The firmware sends 40-byte little-endian UDP packets with `IMUQ` magic and
`wxyz` quaternion order. The matching parser lives in `sensor/packet.py`.

The UDP firmware prints Wi-Fi diagnostics over Serial Monitor at `115200`.
During connection it will:

- scan nearby networks
- report whether `WIFI_SSID` was found
- print Wi-Fi status changes such as `WL_NO_SSID_AVAIL` or `WL_CONNECT_FAILED`
- time out after `WIFI_CONNECT_TIMEOUT_MS`
- print the ESP32 IP, gateway, and RSSI after connecting

Common readings:

```text
Target SSID was NOT found
```

The SSID is misspelled, the network is not nearby, or it is not available on
2.4 GHz.

```text
WL_CONNECT_FAILED
```

The password/security settings are probably wrong.

```text
Wi-Fi connected, but receiver is silent
```

Check `RECEIVER_IP`, macOS/Windows firewall rules, and that the laptop receiver
is listening on `0.0.0.0:5005`. `RECEIVER_IP` is the laptop IP on the phone
hotspot; set it over serial with `ip <a.b.c.d>` or as a compile-time default.

## UDP Timing Checks

The ESP32 streams quaternion packets to:

```text
RECEIVER_IP:RECEIVER_PORT
```

and also listens for latency pings on:

```text
ESP32_UDP_LOCAL_PORT
```

Default ports:

```text
RECEIVER_PORT = 5005
ESP32_UDP_LOCAL_PORT = 5006
```

Run the quaternion receiver:

```powershell
conda run --no-capture-output -n humanoid-sim python demos\demo_udp_quaternion_receiver.py --host 0.0.0.0 --port 5005
```

The receiver reports:

- `hz`: packet receive rate
- `age_ms`: time since the latest packet was received
- `rj_ms`: receive-side interval jitter
- `sj_ms`: ESP32 sensor timestamp interval jitter
- `drops`: inferred sequence-number drops

True one-way latency cannot be measured from `ESP32 micros()` alone because the
ESP32 clock and laptop clock are not synchronized. For a practical latency
number, measure round-trip time:

```powershell
conda run --no-capture-output -n humanoid-sim python demos\demo_udp_latency_ping.py 192.168.1.164 --port 5006
```

Use the ESP32 IP printed by Serial Monitor.

## BNO085 Transport

Use **full UART** for the BNO085 on ESP32-S3. This is the transport mode for
the BNO08x report interface, including rotation-vector quaternion reports.

Do not use I2C for this pairing, and do not use UART-RVC for the solver path.
UART-RVC is a different simplified mode and does not provide the quaternion
report path we are building around.

Default wiring:

| ESP32-S3 pin | ESP32 role | BNO085 pin | BNO085 role |
|---|---|---|---|
| GPIO17 / U1TXD | TX, data out | SCL | UART RX, data in |
| GPIO18 / U1RXD | RX, data in | SDA | UART TX, data out |
| 3V3 | power | Vin | power |
| GND | ground | GND | ground |
| 3V3 | mode select high | P1 | full UART mode select |

Full UART details from the Adafruit BNO085 breakout documentation:

- `SCL` is UART data **into** the BNO085, so connect it to ESP32 TX.
- `SDA` is UART data **out of** the BNO085, so connect it to ESP32 RX.
- `P1` must be pulled high to select full UART mode.
- `P0` should not be pulled high for our full UART setup.
- the Adafruit BNO08x library uses `begin_UART(&Serial1)` for this mode.
- the CircuitPython example uses 3000000 baud for full UART; our Arduino
  firmware currently follows that value with `BNO08X_UART_BAUD`.

GPIO check for ESP32-S3-DevKitC-1 v1.1:

- GPIO17 is broken out on header J1 and labeled `U1TXD`.
- GPIO18 is broken out on header J1 and labeled `U1RXD`.
- GPIO38 is broken out on header J3 but is also the onboard RGB LED signal.
- GPIO35, GPIO36, and GPIO37 may be unavailable on some Octal SPI flash/PSRAM
  variants, so avoid them for BNO085 wiring.

The BNO085 mode pins/jumpers must be set for full UART according to the breakout
documentation.

Identity rule:

- `sensor_id`: physical board identity
- `segment_id`: body placement identity

This lets us swap boards later without confusing calibration profiles.
