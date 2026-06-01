/*
  ESP32-S3 + BNO085 quaternion UDP node.

  BNO085 transport: full UART, not I2C.

  Install Arduino libraries:
    - Adafruit BNO08x
    - Adafruit BusIO
    - Adafruit NeoPixel

  Optional local secrets file, not committed:
    hardware/esp32_bno085_udp/secrets.h

  Example secrets.h:
    #define WIFI_SSID "your-network"
    #define WIFI_PASSWORD "your-password"
    #define RECEIVER_IP "192.168.1.50"

  Runtime configuration over serial:
    The #define values below are only DEFAULTS. On boot they are
    overridden by any values stored in flash (NVS). Open the serial
    monitor at 115200 baud, send "help", and use commands like:
      ssid <name>        set Wi-Fi SSID            (reboot to apply)
      pass <password>    set Wi-Fi password        (reboot to apply)
      ip <a.b.c.d>       set UDP receiver IP       (applies live)
      port <n>           set UDP receiver port     (applies live)
      sensor <n>         set sensor id             (applies live)
      segment <n>        set segment id 0..6,255   (applies live)
      show               print current config
      clear              erase stored config, revert to defaults
      reboot             restart the board
    Every set command is saved to flash immediately and survives reflash
    of these defaults as long as the partition is not erased.
*/

#include <Adafruit_BNO08x.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
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

// Receiver (laptop) IP as a dotted-decimal string, e.g. "192.168.1.50".
#ifndef RECEIVER_IP
#define RECEIVER_IP "192.168.1.50"
#endif

#ifndef RECEIVER_PORT
#define RECEIVER_PORT 5005
#endif

#ifndef ESP32_UDP_LOCAL_PORT
#define ESP32_UDP_LOCAL_PORT 5006
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

#ifndef LED_PIN
// ESP32-S3-DevKitC-1 v1.1 onboard addressable RGB LED.
#define LED_PIN 38
#endif

#ifndef LED_BRIGHTNESS
// Keep the onboard RGB LED visible but not distracting.
#define LED_BRIGHTNESS 16
#endif

#ifndef WIFI_LED_PULSE_PERIOD_MS
#define WIFI_LED_PULSE_PERIOD_MS 2000
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
Adafruit_NeoPixel rgb_led(1, LED_PIN, NEO_GRB + NEO_KHZ800);
WiFiUDP udp;

// Runtime configuration. Initialized from the #define defaults, then
// overridden by loadConfig() with anything saved in flash (NVS).
Preferences prefs;
static const char *PREFS_NAMESPACE = "imucfg";
String g_wifi_ssid = WIFI_SSID;
String g_wifi_password = WIFI_PASSWORD;
IPAddress g_receiver_ip(RECEIVER_IP);
uint16_t g_receiver_port = RECEIVER_PORT;
uint8_t g_sensor_id = SENSOR_ID;
uint8_t g_segment_id = SEGMENT_ID;
String serial_line;

uint32_t sequence_number = 0;
uint32_t sent_packets = 0;
uint32_t failed_udp_packets = 0;
uint32_t ping_replies = 0;
uint32_t last_heartbeat_ms = 0;
float last_qw = 0.0f;
float last_qx = 0.0f;
float last_qy = 0.0f;
float last_qz = 0.0f;

uint32_t ledColor(uint8_t red, uint8_t green, uint8_t blue) {
  return rgb_led.Color(
      static_cast<uint8_t>((red * LED_BRIGHTNESS) / 255),
      static_cast<uint8_t>((green * LED_BRIGHTNESS) / 255),
      static_cast<uint8_t>((blue * LED_BRIGHTNESS) / 255));
}

uint32_t ledColorScaled(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness) {
  return rgb_led.Color(
      static_cast<uint8_t>((red * brightness) / 255),
      static_cast<uint8_t>((green * brightness) / 255),
      static_cast<uint8_t>((blue * brightness) / 255));
}

uint32_t segmentLedColor(uint8_t segment_id) {
  switch (segment_id) {
    case 0:  // pelvis
      return ledColor(255, 255, 255);
    case 1:  // left_thigh
      return ledColor(0, 220, 80);
    case 2:  // left_shank
      return ledColor(255, 230, 0);
    case 3:  // left_foot
      return ledColor(0, 60, 255);
    case 4:  // right_thigh
      return ledColor(255, 140, 0);
    case 5:  // right_shank
      return ledColor(190, 70, 255);
    case 6:  // right_foot
      return ledColor(255, 40, 40);
    default:
      return ledColor(80, 80, 80);
  }
}

const char *segmentName(uint8_t segment_id) {
  switch (segment_id) {
    case 0:
      return "pelvis";
    case 1:
      return "left_thigh";
    case 2:
      return "left_shank";
    case 3:
      return "left_foot";
    case 4:
      return "right_thigh";
    case 5:
      return "right_shank";
    case 6:
      return "right_foot";
    default:
      return "unknown";
  }
}

void setLed(uint32_t color) {
  rgb_led.setPixelColor(0, color);
  rgb_led.show();
}

void updateWifiConnectingLed() {
  const uint32_t phase_ms = millis() % WIFI_LED_PULSE_PERIOD_MS;
  const uint32_t half_period_ms = WIFI_LED_PULSE_PERIOD_MS / 2;
  const uint32_t ramp_ms = phase_ms < half_period_ms
      ? phase_ms
      : WIFI_LED_PULSE_PERIOD_MS - phase_ms;
  const uint8_t min_brightness = 2;
  const uint8_t brightness = min_brightness
      + static_cast<uint8_t>((ramp_ms * (LED_BRIGHTNESS - min_brightness)) / half_period_ms);
  setLed(ledColorScaled(0, 0, 255, brightness));
}

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
    if (ssid == g_wifi_ssid) {
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
  Serial.print(g_wifi_ssid);
  Serial.println(found_target ? "\" was found." : "\" was NOT found.");
}

bool connectWiFi() {
  updateWifiConnectingLed();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(200);

  Serial.println("Wi-Fi setup");
  Serial.print("Target SSID: ");
  Serial.println(g_wifi_ssid);
  Serial.print("Target UDP host: ");
  Serial.print(g_receiver_ip);
  Serial.print(":");
  Serial.println(g_receiver_port);
  Serial.print("ESP32 UDP local port: ");
  Serial.println(ESP32_UDP_LOCAL_PORT);

  printWifiScan();

  WiFi.begin(g_wifi_ssid.c_str(), g_wifi_password.c_str());

  Serial.println("Connecting Wi-Fi...");
  const unsigned long start_ms = millis();
  wl_status_t last_status = WL_IDLE_STATUS;
  while (millis() - start_ms < WIFI_CONNECT_TIMEOUT_MS) {
    updateWifiConnectingLed();
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
      setLed(ledColor(0, 255, 0));
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
  setLed(ledColor(255, 0, 0));
  Serial.print("Final Wi-Fi status: ");
  Serial.println(wifiStatusName(WiFi.status()));
  Serial.println("Hints:");
  Serial.println("  - If target SSID was NOT found: check SSID spelling or 2.4 GHz availability.");
  Serial.println("  - If status is WL_CONNECT_FAILED: check password/security.");
  Serial.println("  - If connected but receiver is silent: check RECEIVER_IP and firewall.");
  return false;
}

void setupBNO085() {
  Serial1.begin(BNO08X_UART_BAUD, SERIAL_8N1, BNO08X_RX_PIN, BNO08X_TX_PIN);
  delay(100);

  if (!bno08x.begin_UART(&Serial1)) {
    Serial.println("Could not find BNO085 over UART");
    Serial.println("Check wiring, BNO085 mode pins, and UART RX/TX crossing.");
    setLed(ledColor(255, 0, 0));
    while (true) {
      delay(1000);
    }
  }

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
    Serial.println("Could not enable BNO085 rotation vector report");
    setLed(ledColor(255, 0, 0));
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
    g_sensor_id,
    g_segment_id,
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

  if (!udp.beginPacket(g_receiver_ip, g_receiver_port)) {
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
  Serial.print(" ping_replies=");
  Serial.print(ping_replies);
  Serial.print(" target=");
  Serial.print(g_receiver_ip);
  Serial.print(":");
  Serial.print(g_receiver_port);
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

void handleUdpPing() {
  const int packet_size = udp.parsePacket();
  if (packet_size <= 0) {
    return;
  }

  char buffer[96];
  const int read_size = udp.read(buffer, min(packet_size, static_cast<int>(sizeof(buffer) - 1)));
  if (read_size <= 0) {
    return;
  }
  buffer[read_size] = '\0';

  if (strncmp(buffer, "PING,", 5) != 0) {
    return;
  }

  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.write(reinterpret_cast<const uint8_t *>(buffer), read_size);
  if (udp.endPacket()) {
    ping_replies++;
  }
}

void loadConfig() {
  prefs.begin(PREFS_NAMESPACE, true);  // read-only
  g_wifi_ssid = prefs.getString("ssid", WIFI_SSID);
  g_wifi_password = prefs.getString("pass", WIFI_PASSWORD);
  g_receiver_port = prefs.getUShort("port", RECEIVER_PORT);
  g_sensor_id = prefs.getUChar("sensor", SENSOR_ID);
  g_segment_id = prefs.getUChar("segment", SEGMENT_ID);
  const String ip_str = prefs.getString("ip", RECEIVER_IP);
  prefs.end();

  // Both the stored value and the compiled-in default are dotted-decimal
  // strings. Fall back to the default if a stored value fails to parse.
  if (!g_receiver_ip.fromString(ip_str)) {
    g_receiver_ip.fromString(RECEIVER_IP);
  }
}

void printConfig() {
  Serial.println("---- current config ----");
  Serial.print("ssid    : ");
  Serial.println(g_wifi_ssid);
  Serial.print("pass    : ");
  Serial.println(g_wifi_password);
  Serial.print("ip      : ");
  Serial.println(g_receiver_ip);
  Serial.print("port    : ");
  Serial.println(g_receiver_port);
  Serial.print("sensor  : ");
  Serial.println(g_sensor_id);
  Serial.print("segment : ");
  Serial.print(g_segment_id);
  Serial.print(" (");
  Serial.print(segmentName(g_segment_id));
  Serial.println(")");
  Serial.println("-------------------------");
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  ssid <name>      set Wi-Fi SSID        (reboot to apply)");
  Serial.println("  pass <password>  set Wi-Fi password    (reboot to apply)");
  Serial.println("  ip <a.b.c.d>     set UDP target IP      (applies live)");
  Serial.println("  port <n>         set UDP target port    (applies live)");
  Serial.println("  sensor <n>       set sensor id          (applies live)");
  Serial.println("  segment <n>      set segment id 0..6,255(applies live)");
  Serial.println("  show             print current config");
  Serial.println("  clear            erase stored config, revert to defaults");
  Serial.println("  reboot           restart the board");
  Serial.println("  help             print this help");
}

void applyConfigCommand(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line == "help" || line == "?") {
    printHelp();
    return;
  }
  if (line == "show") {
    printConfig();
    return;
  }
  if (line == "reboot") {
    Serial.println("Rebooting...");
    delay(100);
    ESP.restart();
    return;
  }
  if (line == "clear") {
    prefs.begin(PREFS_NAMESPACE, false);
    prefs.clear();
    prefs.end();
    Serial.println("Cleared stored config. Reboot to apply compiled-in defaults.");
    return;
  }

  const int space = line.indexOf(' ');
  if (space < 0) {
    Serial.print("Unknown command: ");
    Serial.println(line);
    Serial.println("Send \"help\" for the command list.");
    return;
  }
  const String key = line.substring(0, space);
  String value = line.substring(space + 1);
  value.trim();
  if (value.length() == 0) {
    Serial.print("Missing value for: ");
    Serial.println(key);
    return;
  }

  if (key == "ssid") {
    g_wifi_ssid = value;
    prefs.begin(PREFS_NAMESPACE, false);
    prefs.putString("ssid", g_wifi_ssid);
    prefs.end();
    Serial.print("Saved ssid = ");
    Serial.println(g_wifi_ssid);
    Serial.println("Reboot to reconnect with the new SSID.");
  } else if (key == "pass") {
    g_wifi_password = value;
    prefs.begin(PREFS_NAMESPACE, false);
    prefs.putString("pass", g_wifi_password);
    prefs.end();
    Serial.println("Saved password.");
    Serial.println("Reboot to reconnect with the new password.");
  } else if (key == "ip") {
    IPAddress parsed;
    if (!parsed.fromString(value)) {
      Serial.print("Invalid IP address: ");
      Serial.println(value);
      return;
    }
    g_receiver_ip = parsed;
    prefs.begin(PREFS_NAMESPACE, false);
    prefs.putString("ip", value);
    prefs.end();
    Serial.print("Saved ip = ");
    Serial.println(g_receiver_ip);
  } else if (key == "port") {
    const long port = value.toInt();
    if (port < 1 || port > 65535) {
      Serial.print("Invalid port: ");
      Serial.println(value);
      return;
    }
    g_receiver_port = static_cast<uint16_t>(port);
    prefs.begin(PREFS_NAMESPACE, false);
    prefs.putUShort("port", g_receiver_port);
    prefs.end();
    Serial.print("Saved port = ");
    Serial.println(g_receiver_port);
  } else if (key == "sensor") {
    const long id = value.toInt();
    if (id < 0 || id > 255) {
      Serial.print("Invalid sensor id (0..255): ");
      Serial.println(value);
      return;
    }
    g_sensor_id = static_cast<uint8_t>(id);
    prefs.begin(PREFS_NAMESPACE, false);
    prefs.putUChar("sensor", g_sensor_id);
    prefs.end();
    Serial.print("Saved sensor = ");
    Serial.println(g_sensor_id);
  } else if (key == "segment") {
    const long id = value.toInt();
    if (id < 0 || id > 255) {
      Serial.print("Invalid segment id (0..255): ");
      Serial.println(value);
      return;
    }
    g_segment_id = static_cast<uint8_t>(id);
    prefs.begin(PREFS_NAMESPACE, false);
    prefs.putUChar("segment", g_segment_id);
    prefs.end();
    setLed(segmentLedColor(g_segment_id));
    Serial.print("Saved segment = ");
    Serial.print(g_segment_id);
    Serial.print(" (");
    Serial.print(segmentName(g_segment_id));
    Serial.println(")");
  } else {
    Serial.print("Unknown command: ");
    Serial.println(key);
    Serial.println("Send \"help\" for the command list.");
  }
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serial_line.length() > 0) {
        applyConfigCommand(serial_line);
        serial_line = "";
      }
    } else {
      serial_line += c;
      if (serial_line.length() > 200) {
        serial_line = "";  // guard against runaway input
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  loadConfig();

  rgb_led.begin();
  rgb_led.clear();
  rgb_led.show();
  setLed(ledColor(255, 180, 0));

  Serial.println("ESP32-S3 BNO085 UART UDP streamer");
  Serial.println("Send \"help\" over serial to view/change config.");
  Serial.print("Sensor ID: ");
  Serial.println(g_sensor_id);
  Serial.print("Segment ID: ");
  Serial.print(g_segment_id);
  Serial.print(" (");
  Serial.print(segmentName(g_segment_id));
  Serial.println(")");
  Serial.print("Segment LED color is assigned from segment ID on GPIO ");
  Serial.println(LED_PIN);
  Serial.print("ESP32 RX data-in pin, from BNO085 SDA/UART-TX: GPIO ");
  Serial.println(BNO08X_RX_PIN);
  Serial.print("ESP32 TX data-out pin, to BNO085 SCL/UART-RX: GPIO ");
  Serial.println(BNO08X_TX_PIN);

  while (!connectWiFi()) {
    Serial.println("Retrying Wi-Fi in 5 seconds...");
    delay(5000);
  }
  udp.begin(ESP32_UDP_LOCAL_PORT);
  setupBNO085();

  setLed(segmentLedColor(g_segment_id));
  Serial.println("Streaming BNO085 rotation-vector quaternions over UDP");
}

void loop() {
  handleSerialCommands();
  handleUdpPing();

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
