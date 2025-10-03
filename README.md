# ESP32-Truly-Universal-Remote
ESP32-based universal remote control with IR, Bluetooth, and OLED display capabilities.
# Domninus Scepter: ESP32 IR Remote Control

**Domninus Scepter** is a versatile ESP32-based IR remote control system designed to control infrared devices such as TVs, air conditioners (ACs), and speakers, with additional Bluetooth keyboard functionality for media control. It features an OLED display for user interaction, a rotary encoder for navigation, and persistent storage for IR codes. The project supports a wide range of IR protocols and includes deep sleep for power efficiency.

## Features
- **IR Control**: Supports multiple IR protocols (NEC, Sony, Samsung, Coolix, Daikin, Mitsubishi, etc.) for TVs, ACs, and speakers.
- **Bluetooth Keyboard**: Controls media playback (play/pause, volume, mute) on Android/Windows devices via BLE.
- **OLED Display**: Shows menu, selected profile, battery status, and feedback messages.
- **Rotary Encoder**: Navigate profiles and modes with a rotary encoder and button.
- **Persistent Storage**: Saves IR codes and settings using the ESP32's NVS (Preferences).
- **Battery Monitoring**: Displays battery percentage based on voltage readings.
- **Deep Sleep**: Enters low-power mode after 15 seconds of inactivity to conserve battery.
- **Debug Mode**: Extensive serial output for debugging (enable with `#define DEBUG`).

## Hardware Requirements
- **ESP32 Board** (e.g., ESP32 DevKitC)
- **IR Receiver** (e.g., TSOP4838, connected to GPIO 15)
- **IR LED** (connected to GPIO 4)
- **OLED Display** (128x64, SSD1306, I2C, address 0x3C)
- **Rotary Encoder** (CLK: GPIO 25, DT: GPIO 26, SW: GPIO 27)
- **Push Buttons**:
  - BTN1 (Play/Pause): GPIO 33
  - BTN2 (Volume Up): GPIO 32
  - BTN3 (Mute): GPIO 13
  - BTN4 (Volume Down): GPIO 12
  - REC (Device Switch/Record): GPIO 5
- **Battery Monitoring**: Voltage divider connected to GPIO 34
- **Debug LED**: GPIO 2 (optional, for visual feedback)

## Software Requirements
- **Arduino IDE** (2.x recommended)
- **ESP32 Board Support**: Install via Boards Manager (`esp32` by Espressif)
- **Libraries**:
  - [IRremoteESP8266](https://github.com/crankyoldgit/IRremoteESP8266) (v2.8.2 or higher)
  - [Adafruit_SSD1306](https://github.com/adafruit/Adafruit_SSD1306) (for OLED)
  - [Preferences](https://github.com/espressif/arduino-esp32) (included with ESP32 core)
  - [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) (for BLE)
  - [BleKeyboard](https://github.com/T-vK/ESP32-BLE-Keyboard) (for Bluetooth keyboard)

## Installation
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/domninus-scepter.git
