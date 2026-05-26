/*
  ESP32-S3 + BNO085 UART sanity check.

  Use this before the UDP streaming firmware. It verifies that the ESP32-S3 can
  talk to the BNO085 over full UART and print rotation-vector quaternions.
*/

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

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
#define REPORT_INTERVAL_US 10000
#endif

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void enableReports() {
  Serial.println("Enabling rotation-vector report");
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
    Serial.println("Could not enable rotation-vector report");
    while (true) {
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("ESP32-S3 BNO085 UART sanity check");
  Serial.print("ESP32 RX data-in pin, from BNO085 SDA/UART-TX: GPIO ");
  Serial.println(BNO08X_RX_PIN);
  Serial.print("ESP32 TX data-out pin, to BNO085 SCL/UART-RX: GPIO ");
  Serial.println(BNO08X_TX_PIN);

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
  Serial.print("quat wxyz: ");
  Serial.print(q.real, 6);
  Serial.print(" ");
  Serial.print(q.i, 6);
  Serial.print(" ");
  Serial.print(q.j, 6);
  Serial.print(" ");
  Serial.print(q.k, 6);
  Serial.print(" accuracy: ");
  Serial.print(q.accuracy, 4);
  Serial.print(" status: ");
  Serial.println(sensorValue.status);
}
