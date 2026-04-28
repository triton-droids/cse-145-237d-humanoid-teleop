/*
  ESP32-S3 + BNO085 BLE sanity check.

  This is a no-Wi-Fi test path. It reads rotation-vector quaternions from the
  BNO085 over full UART and publishes compact text notifications over BLE using
  a Nordic UART Service-compatible TX characteristic.
*/

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#ifndef BLE_DEVICE_NAME
#define BLE_DEVICE_NAME "imu-ble-sanity"
#endif

#ifndef SENSOR_ID
#define SENSOR_ID 0
#endif

#ifndef SEGMENT_ID
#define SEGMENT_ID 255
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

#ifndef REPORT_INTERVAL_US
#define REPORT_INTERVAL_US 50000
#endif

#define BNO08X_RESET -1

// Nordic UART Service UUIDs. Most BLE scanner apps recognize this pattern.
static const char *NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *NUS_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *NUS_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
BLECharacteristic *txCharacteristic = nullptr;
bool bleClientConnected = false;
uint32_t sequenceNumber = 0;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    bleClientConnected = true;
    Serial.println("BLE client connected");
  }

  void onDisconnect(BLEServer *server) override {
    bleClientConnected = false;
    Serial.println("BLE client disconnected");
    server->getAdvertising()->start();
  }
};

void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(NUS_SERVICE_UUID);
  txCharacteristic = service->createCharacteristic(
      NUS_TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  txCharacteristic->addDescriptor(new BLE2902());

  service->createCharacteristic(
      NUS_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);

  service->start();
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(NUS_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();
}

void enableReports() {
  Serial.println("Enabling rotation-vector report");
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
    Serial.println("Could not enable rotation-vector report");
    while (true) {
      delay(1000);
    }
  }
}

void setupBNO085() {
  Serial1.begin(BNO08X_UART_BAUD, SERIAL_8N1, BNO08X_RX_PIN, BNO08X_TX_PIN);
  delay(100);

  if (!bno08x.begin_UART(&Serial1)) {
    Serial.println("Could not find BNO085 over UART");
    Serial.println("Check power, ground, crossed RX/TX, and BNO085 P0/P1 mode pins.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("BNO085 found over UART");
  enableReports();
}

void notifyLine(const char *line) {
  if (!bleClientConnected || txCharacteristic == nullptr) {
    return;
  }

  txCharacteristic->setValue((uint8_t *)line, strlen(line));
  txCharacteristic->notify();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("ESP32-S3 BNO085 BLE sanity check");
  Serial.print("BLE name: ");
  Serial.println(BLE_DEVICE_NAME);
  Serial.print("ESP32 RX data-in pin, from BNO085 SDA/UART-TX: GPIO ");
  Serial.println(BNO08X_RX_PIN);
  Serial.print("ESP32 TX data-out pin, to BNO085 SCL/UART-RX: GPIO ");
  Serial.println(BNO08X_TX_PIN);

  setupBLE();
  setupBNO085();

  Serial.println("BLE advertising started");
}

void loop() {
  if (bno08x.wasReset()) {
    Serial.println("BNO085 reset detected");
    enableReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    delay(1);
    return;
  }

  if (sensorValue.sensorId != SH2_ROTATION_VECTOR) {
    return;
  }

  const auto &q = sensorValue.un.rotationVector;
  char line[128];
  snprintf(
      line,
      sizeof(line),
      "Q,%lu,%u,%u,%.5f,%.5f,%.5f,%.5f,%.3f,%u\n",
      static_cast<unsigned long>(sequenceNumber++),
      static_cast<unsigned int>(SENSOR_ID),
      static_cast<unsigned int>(SEGMENT_ID),
      q.real,
      q.i,
      q.j,
      q.k,
      q.accuracy,
      static_cast<unsigned int>(sensorValue.status));

  Serial.print(line);
  notifyLine(line);
}
