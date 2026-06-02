# ESP32-S3 LED Sanity Check

Use this first to make sure the ESP32-S3 board can flash and run code before
testing the BNO085.

The sketch cycles the onboard addressable RGB LED on GPIO 38 and prints status
over serial.

## Arduino IDE

1. Open `esp32s3_led_sanity.ino`.
2. Select your ESP32-S3 board.
3. Install the `Adafruit NeoPixel` library.
4. Upload the sketch.
5. Open Serial Monitor at `115200`.

Expected behavior:

```text
ESP32-S3 RGB LED sanity check
RGB LED pin: GPIO 38
If the board is alive, the RGB LED should cycle red/green/blue/off.
RGB LED red
RGB LED green
RGB LED blue
RGB LED off
...
```
