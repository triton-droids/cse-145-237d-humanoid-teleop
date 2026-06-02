# ESP32-S3 BNO085 BLE Sanity Check

We use this when Wi-Fi is unavailable. It keeps the same BNO085 full UART wiring as
the UDP firmware, but sends quaternion text notifications over BLE.

This is a test path, not our main 7-sensor architecture.

## Wiring

| ESP32-S3 pin | ESP32 role | BNO085 pin | BNO085 role |
|---|---|---|---|
| GPIO17 / U1TXD | TX, data out | SCL | UART RX, data in |
| GPIO18 / U1RXD | RX, data in | SDA | UART TX, data out |
| 3V3 | power | Vin | power |
| GND | ground | GND | ground |
| 3V3 | mode select high | P1 | full UART mode select |

## Arduino IDE

Install:

- `Adafruit BNO08x`
- `Adafruit BusIO`

The BLE headers come from the ESP32 Arduino board package.

Open `esp32s3_bno085_ble_sanity.ino`, select `ESP32S3 Dev Module`, upload, and
open Serial Monitor at `115200`.

## BLE Test

Use a BLE scanner/client app such as `nRF Connect`.

1. Scan for `imu-ble-sanity`.
2. Connect.
3. Open service `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`.
4. Enable notifications on characteristic `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`.

Notification format:

```text
Q,sequence,sensor_id,segment_id,qw,qx,qy,qz,accuracy,status
```

Example:

```text
Q,15,0,255,0.99812,0.00120,-0.05882,0.00411,0.220,3
```

The same line is also printed to Serial Monitor, so if BLE is annoying you can
still verify the BNO085 data path over USB serial.
