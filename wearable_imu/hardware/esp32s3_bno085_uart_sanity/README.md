# ESP32-S3 BNO085 UART Sanity Check

We run this after the board LED sanity check and before UDP streaming.

This sketch verifies:

- ESP32-S3 can talk to BNO085 over full UART
- BNO085 can emit rotation-vector quaternions
- Arduino Serial Monitor shows live `wxyz` quaternion values

## Wiring

Default ESP32-S3 pins in the sketch:

| ESP32-S3 pin | ESP32 role | BNO085 pin | BNO085 role |
|---|---|---|---|
| GPIO17 / U1TXD | TX, data out | SCL | UART RX, data in |
| GPIO18 / U1RXD | RX, data in | SDA | UART TX, data out |
| 3V3 | power | Vin | power |
| GND | ground | GND | ground |
| 3V3 | mode select high | P1 | full UART mode select |

These pins are usable on ESP32-S3-DevKitC-1 v1.1:

- GPIO17 is header J1 `U1TXD`
- GPIO18 is header J1 `U1RXD`

Avoid GPIO35, GPIO36, and GPIO37 for this project because some module variants
reserve them for Octal SPI flash/PSRAM.

Set the BNO085 mode pins for **full UART**, not I2C and not UART-RVC. On the
Adafruit breakout, this is controlled by the P0/P1 mode jumpers/pins. Check the
board guide before soldering jumpers.

## Arduino IDE

Install:

- `Adafruit BNO08x`
- `Adafruit BusIO`

Open `esp32s3_bno085_uart_sanity.ino`, select `ESP32S3 Dev Module`, upload, and
open Serial Monitor at `115200`.

Expected output:

```text
ESP32-S3 BNO085 UART sanity check
BNO085 found over UART
quat wxyz: ...
```
