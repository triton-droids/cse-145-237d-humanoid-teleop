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

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
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

  udp.beginPacket(JETSON_IP, JETSON_PORT);
  udp.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("ESP32-S3 BNO085 UART UDP streamer");
  Serial.print("ESP32 RX data-in pin, from BNO085 SDA/UART-TX: GPIO ");
  Serial.println(BNO08X_RX_PIN);
  Serial.print("ESP32 TX data-out pin, to BNO085 SCL/UART-RX: GPIO ");
  Serial.println(BNO08X_TX_PIN);

  connectWiFi();
  udp.begin(0);
  setupBNO085();

  Serial.println("Streaming BNO085 rotation-vector quaternions over UDP");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  sh2_SensorValue_t sensorValue;
  if (!bno08x.getSensorEvent(&sensorValue)) {
    delay(1);
    return;
  }

  if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
    sendQuaternionPacket(sensorValue);
  }
}
