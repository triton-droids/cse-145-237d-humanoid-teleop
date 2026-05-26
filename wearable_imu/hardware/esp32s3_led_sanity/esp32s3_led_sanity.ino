/*
  ESP32-S3-DevKitC-1 board sanity check.

  The v1.1 board has an addressable RGB LED on GPIO 38, not a plain LED.
  This sketch cycles that LED through colors and prints status over serial.
*/

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#ifndef LED_PIN
#define LED_PIN 38
#endif

#ifndef BLINK_INTERVAL_MS
#define BLINK_INTERVAL_MS 700
#endif

Adafruit_NeoPixel rgb_led(1, LED_PIN, NEO_GRB + NEO_KHZ800);
uint8_t color_index = 0;
unsigned long last_toggle_ms = 0;

uint32_t colorForIndex(uint8_t index) {
  switch (index % 4) {
    case 0:
      return rgb_led.Color(32, 0, 0);
    case 1:
      return rgb_led.Color(0, 32, 0);
    case 2:
      return rgb_led.Color(0, 0, 32);
    default:
      return rgb_led.Color(0, 0, 0);
  }
}

const char *labelForIndex(uint8_t index) {
  switch (index % 4) {
    case 0:
      return "red";
    case 1:
      return "green";
    case 2:
      return "blue";
    default:
      return "off";
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  rgb_led.begin();
  rgb_led.clear();
  rgb_led.show();

  Serial.println("ESP32-S3 RGB LED sanity check");
  Serial.print("RGB LED pin: GPIO ");
  Serial.println(LED_PIN);
  Serial.println("If the board is alive, the RGB LED should cycle red/green/blue/off.");
}

void loop() {
  const unsigned long now_ms = millis();
  if (now_ms - last_toggle_ms < BLINK_INTERVAL_MS) {
    return;
  }

  last_toggle_ms = now_ms;
  rgb_led.setPixelColor(0, colorForIndex(color_index));
  rgb_led.show();

  Serial.print("RGB LED ");
  Serial.println(labelForIndex(color_index));
  color_index++;
}
