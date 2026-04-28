/*
  ESP32-S3 + BNO085 quaternion UDP node.

  BNO085 transport: full UART, not I2C.

  Install Arduino libraries:
    - Adafruit BNO08x
    - Adafruit BusIO

  Optional local secrets file, not committed:
    hardware/esp32_bno085_udp/secrets.h

  Example secrets.h:
    #define WIFI_SSID "your-network"
    #define WIFI_PASSWORD "your-password"
    #define JETSON_IP IPAddress(192, 168, 1, 50)
*/

#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "CHANGE_ME"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "CHANGE_ME"
#endif

#ifndef JETSON_IP
#define JETSON_IP IPAddress(192, 168, 1, 50)
#endif

#ifndef JETSON_PORT
#define JETSON_PORT 5005
#endif

// Give each ESP32 a unique sensor id and segment id before flashing.
// Segment ids match sensor/packet.py:
// 0 pelvis, 1 left_thigh, 2 left_shank, 3 left_foot,
// 4 right_thigh, 5 right_shank, 6 right_foot, 255 unknown.
#ifndef SENSOR_ID
#define SENSOR_ID 0
#endif

#ifndef SEGMENT_ID
#define SEGMENT_ID 255
#endif

#ifndef REPORT_INTERVAL_US
#define REPORT_INTERVAL_US 10000
#endif

#ifndef BNO08X_RX_PIN
// ESP32-S3 GPIO18 is U1RXD on ESP32-S3-DevKitC-1.
#define BNO08X_RX_PIN 18
#endif

#ifndef BNO08X_TX_PIN
// ESP32-S3 GPIO17 is U1TXD on ESP32-S3-DevKitC-1.
#define BNO08X_TX_PIN 17
#endif

#ifndef BNO08X_UART_BAUD
#define BNO08X_UART_BAUD 3000000
#endif

#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 20000
#endif

static constexpr uint8_t PACKET_VERSION = 1;
static constexpr uint8_t QUAT_ORDER_WXYZ = 1;
static constexpr uint8_t REPORT_TYPE_ROTATION_VECTOR = 1;

struct __attribute__((packed)) QuaternionPacket {
  char magic[4];
  uint8_t version;
  uint8_t sensor_id;
  uint8_t segment_id;
  uint8_t quat_order;
  uint32_t sequence;
  uint32_t sensor_time_us;
  float qw;
  float qx;
  float qy;
  float qz;
  float accuracy;
  uint8_t status;
  uint8_t report_type;
  uint16_t reserved;
};

static_assert(sizeof(QuaternionPacket) == 40, "QuaternionPacket must stay 40 bytes");

Adafruit_BNO08x bno08x;
WiFiUDP udp;
uint32_t sequence_number = 0;
uint32_t sent_packets = 0;
uint32_t failed_udp_packets = 0;
uint32_t last_heartbeat_ms = 0;
float last_qw = 0.0f;
float last_qx = 0.0f;
float last_qy = 0.0f;
float last_qz = 0.0f;

const char *wifiStatusName(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS:
      return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL:
      return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED:
      return "WL_SCAN_COMPLETED";
    case WL_CONNECTED:
      return "WL_CONNECTED";
    case WL_CONNECT_FAILED:
      return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED:
      return "WL_DISCONNECTED";
    default:
      return "WL_UNKNOWN_STATUS";
  }
}

void printWifiScan() {
  Serial.println("Scanning Wi-Fi networks...");
  const int network_count = WiFi.scanNetworks();
  if (network_count < 0) {
    Serial.print("Wi-Fi scan failed with code ");
    Serial.println(network_count);
    return;
  }

  bool found_target = false;
  Serial.print("Networks found: ");
  Serial.println(network_count);
  for (int index = 0; index < network_count; index++) {
    const String ssid = WiFi.SSID(index);
    if (ssid == WIFI_SSID) {
      found_target = true;
    }
    Serial.print("  ");
    Serial.print(index + 1);
    Serial.print(": ");
    Serial.print(ssid);
    Serial.print(" RSSI=");
    Serial.print(WiFi.RSSI(index));
    Serial.print(" dBm enc=");
    Serial.println(WiFi.encryptionType(index));
  }

  Serial.print("Target SSID \"");
  Serial.print(WIFI_SSID);
  Serial.println(found_target ? "\" was found." : "\" was NOT found.");
}

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(200);

  Serial.println("Wi-Fi setup");
  Serial.print("Target SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("Target UDP host: ");
  Serial.print(JETSON_IP);
  Serial.print(":");
  Serial.println(JETSON_PORT);

  printWifiScan();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting Wi-Fi...");
  const unsigned long start_ms = millis();
  wl_status_t last_status = WL_IDLE_STATUS;
  while (millis() - start_ms < WIFI_CONNECT_TIMEOUT_MS) {
    const wl_status_t status = WiFi.status();
    if (status == WL_CONNECTED) {
      Serial.println("Wi-Fi connected.");
      Serial.print("ESP32 IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("Gateway: ");
      Serial.println(WiFi.gatewayIP());
      Serial.print("RSSI: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
      return true;
    }

    if (status != last_status) {
      Serial.print("Wi-Fi status: ");
      Serial.println(wifiStatusName(status));
      last_status = status;
    }
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Wi-Fi connection timed out.");
  Serial.print("Final Wi-Fi status: ");
  Serial.println(wifiStatusName(WiFi.status()));
  Serial.println("Hints:");
  Serial.println("  - If target SSID was NOT found: check SSID spelling or 2.4 GHz availability.");
  Serial.println("  - If status is WL_CONNECT_FAILED: check password/security.");
  Serial.println("  - If connected but receiver is silent: check JETSON_IP and firewall.");
  return false;
}

void setupBNO085() {
  Serial1.begin(BNO08X_UART_BAUD, SERIAL_8N1, BNO08X_RX_PIN, BNO08X_TX_PIN);
  delay(100);

  if (!bno08x.begin_UART(&Serial1)) {
    Serial.println("Could not find BNO085 over UART");
    Serial.println("Check wiring, BNO085 mode pins, and UART RX/TX crossing.");
    while (true) {
      delay(1000);
    }
  }

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
    Serial.println("Could not enable BNO085 rotation vector report");
    while (true) {
      delay(1000);
    }
  }
}

void sendQuaternionPacket(const sh2_SensorValue_t &sensorValue) {
  const auto &rotation = sensorValue.un.rotationVector;
  last_qw = rotation.real;
  last_qx = rotation.i;
  last_qy = rotation.j;
  last_qz = rotation.k;

  QuaternionPacket packet = {
    {'I', 'M', 'U', 'Q'},
    PACKET_VERSION,
    static_cast<uint8_t>(SENSOR_ID),
    static_cast<uint8_t>(SEGMENT_ID),
    QUAT_ORDER_WXYZ,
    sequence_number++,
    micros(),
    rotation.real,
    rotation.i,
    rotation.j,
    rotation.k,
    rotation.accuracy,
    sensorValue.status,
    REPORT_TYPE_ROTATION_VECTOR,
    0,
  };

  if (!udp.beginPacket(JETSON_IP, JETSON_PORT)) {
    failed_udp_packets++;
    return;
  }
  const size_t written = udp.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  if (written != sizeof(packet) || !udp.endPacket()) {
    failed_udp_packets++;
    return;
  }
  sent_packets++;
}

void printStreamingHeartbeat() {
  const uint32_t now_ms = millis();
  if (now_ms - last_heartbeat_ms < 1000) {
    return;
  }

  last_heartbeat_ms = now_ms;
  Serial.print("UDP heartbeat sent=");
  Serial.print(sent_packets);
  Serial.print(" failed=");
  Serial.print(failed_udp_packets);
  Serial.print(" target=");
  Serial.print(JETSON_IP);
  Serial.print(":");
  Serial.print(JETSON_PORT);
  Serial.print(" wifi=");
  Serial.print(wifiStatusName(WiFi.status()));
  Serial.print(" rssi=");
  Serial.print(WiFi.RSSI());
  Serial.print(" last_q_wxyz=");
  Serial.print(last_qw, 4);
  Serial.print(",");
  Serial.print(last_qx, 4);
  Serial.print(",");
  Serial.print(last_qy, 4);
  Serial.print(",");
  Serial.println(last_qz, 4);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("ESP32-S3 BNO085 UART UDP streamer");
  Serial.print("ESP32 RX data-in pin, from BNO085 SDA/UART-TX: GPIO ");
  Serial.println(BNO08X_RX_PIN);
  Serial.print("ESP32 TX data-out pin, to BNO085 SCL/UART-RX: GPIO ");
  Serial.println(BNO08X_TX_PIN);

  while (!connectWiFi()) {
    Serial.println("Retrying Wi-Fi in 5 seconds...");
    delay(5000);
  }
  udp.begin(0);
  setupBNO085();

  Serial.println("Streaming BNO085 rotation-vector quaternions over UDP");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected.");
    while (!connectWiFi()) {
      Serial.println("Retrying Wi-Fi in 5 seconds...");
      delay(5000);
    }
  }

  sh2_SensorValue_t sensorValue;
  if (!bno08x.getSensorEvent(&sensorValue)) {
    delay(1);
    return;
  }

  if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
    sendQuaternionPacket(sensorValue);
  }
  printStreamingHeartbeat();
}
