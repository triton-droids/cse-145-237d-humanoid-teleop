// Copy this file to secrets.h and edit for each ESP32.
// secrets.h is ignored by Git so Wi-Fi credentials stay local.

#pragma once

#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "Password"
#define JETSON_IP IPAddress(192, 168, 29, 156)
#define JETSON_PORT 5005

// Segment IDs:
// 0 pelvis
// 1 left_thigh
// 2 left_shank
// 3 left_foot
// 4 right_thigh
// 5 right_shank
// 6 right_foot
// 255 unknown
#define SENSOR_ID 0
#define SEGMENT_ID 255

// 10000 us = 100 Hz.
#define REPORT_INTERVAL_US 10000

// ESP32-S3 UART pins for BNO085 full UART mode.
// ESP32-S3-DevKitC-1 header exposes GPIO17 as U1TXD and GPIO18 as U1RXD.
// GPIO17 / U1TXD: ESP32 TX data out -> BNO085 SCL / UART RX data in
// GPIO18 / U1RXD: ESP32 RX data in  <- BNO085 SDA / UART TX data out
#define BNO08X_TX_PIN 17
#define BNO08X_RX_PIN 18
#define BNO08X_UART_BAUD 3000000

// Wi-Fi diagnostics timeout before retrying.
#define WIFI_CONNECT_TIMEOUT_MS 20000
