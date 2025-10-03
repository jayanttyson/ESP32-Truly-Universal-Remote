# ESP32-Truly-Universal-Remote
ESP32-based universal remote control with IR, Bluetooth, and OLED display capabilities.
# Domninus Scepter: ESP32 IR/Bluetooth Remote Control
<img width="3000" height="4000" alt="image" src="https://github.com/user-attachments/assets/32203d06-d468-45f8-ba30-a9eb29805c4d" />


**Domninus Scepter** is a versatile ESP32-based IR/Bluetooth remote control system designed to control infrared devices such as TVs, air conditioners (ACs), and speakers, with additional Bluetooth keyboard functionality for media control. It features an OLED display for user interaction, a rotary encoder for navigation, and persistent storage for IR codes. The project supports a wide range of IR protocols and includes deep sleep for power efficiency.

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
- **Battery Monitoring**: Voltage divider connected to GPIO 34, used 2 x 1MΩ for Minimum Battery Drainage
- **Debug LED**: GPIO 2 {inbuild LED}/(optional, for visual feedback)
- 2N2222 General Purpose Transistor for IR LED
- 5v voltage regulator for powering the ESP32 with a stable Voltage
- 0.1uf Ceramic Capacitor at GPIO34 to GND for signal filtering (highly recomended)

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
   git clone https://github.com/jayanttyson/ESP32-Truly-Universal-Remote.git
Install Arduino IDE:Download and install from arduino.cc.

Install ESP32 Board Support:In Arduino IDE, go to File > Preferences, add the following to Additional Boards Manager URLs:

https://raw.githubusercontent.com/espressif/arduino-esp32/master/package_esp32_index.json

Go to Tools > Board > Boards Manager, search for esp32, and install.

Install Libraries:In Arduino IDE, go to Sketch > Include Library > Manage Libraries.
Install IRremoteESP8266 (v2.8.2+), Adafruit_SSD1306, NimBLE-Arduino, and BleKeyboard.

Open the Sketch:Open domninus_scepter.ino from the cloned repository in Arduino IDE.

Configure and Upload:Select your ESP32 board under Tools > Board (e.g., ESP32 Dev Module).
Connect the ESP32 via USB, select the correct port under Tools > Port.
Upload the sketch (Sketch > Upload).

UsagePower On:Connect the ESP32 to a power source (e.g., battery or USB).
The OLED displays the menu with available profiles: TV, AC, SPK, AC_X, BT.

Navigate Profiles:Rotate the encoder to highlight a profile.
Press the encoder button to select the profile or toggle between menu and control mode.

Record IR Commands:In control mode, press the REC button (GPIO 5) to enter record mode (REC appears on OLED).
Point the target remote at the IR receiver and press a button.
If successful, the OLED shows "Signal captured! Press BTN1/2/3/4 to save".
Press a button (1-4) to save the IR code to that button for the current profile.

Send IR Commands:In control mode, press BTN1, BTN2, BTN3, or BTN4 to send the saved IR command.
For AC profiles (AC, AC_X), supports protocols like Coolix, Daikin, etc.
For TV/SPK, supports NEC, Sony, etc., or raw IR codes.

Bluetooth Mode:Select the BT profile.
Pair with a device (Android/Windows) via Bluetooth settings (device name: Domninus Scepter).
Use BTN1 (Play/Pause), BTN2 (Volume Up), BTN3 (Mute), BTN4 (Volume Down).
Press REC to switch between Android and Windows profiles.

Battery Monitoring:Battery percentage is displayed on the OLED, updated every 30 seconds.

Deep Sleep:The device enters deep sleep after 15 seconds of inactivity to save power.
Wake by pressing the encoder button (GPIO 27).

DebuggingEnable Debug Mode:Uncomment #define DEBUG in the sketch to enable serial output at 115200 baud.
Open the Serial Monitor (Tools > Serial Monitor) to view logs, including:IR capture details (protocol, value, raw timings).
Profile load/save status.
Bluetooth connection status.

Common Issues:IR Commands Not Working:Ensure IRremoteESP8266 is v2.8.2 or higher (older versions may cause compilation errors).
Check IR receiver/emitter wiring (GPIO 15 for receiver, GPIO 4 for emitter).
Verify the IR code was captured and saved (check Serial Monitor for Saved AC ... or Saved ...).

Compilation Errors:If enableIRIn() errors occur, update IRremoteESP8266 to v2.8.2+ or use the provided modified sketch for older versions.
Ensure all libraries are installed and up-to-date.

Bluetooth Issues:Reset BLE by pressing REC in BT profile to restart advertising.
Check Serial Monitor for Bluetooth initialized or BT Not Connected!.

Timer Errors (gptimer_register_to_group):Ensure irrecv.disableIRIn() is called before irsend operations (handled in the sketch).
If persistent, disable BLE temporarily to isolate conflicts.

Profile Persistence:If profiles don’t save, clear NVS with a sketch like:cpp

#include <Preferences.h>
void setup() {
  Preferences prefs;
  prefs.begin("IRRemote", false);
  prefs.clear();
  prefs.end();
  Serial.begin(115200);
  Serial.println("NVS cleared");
}
void loop() {}

Example Serial OutputOn IR Capture (AC Profile):

=== IR Capture Debug ===
Profile: AC
Decode type: COOLIX
Value: 0x123456
Raw length: 68
...
Saved AC COOLIX
AC value: 0x123456, len: 0
Saved AC raw: 68

On Profile Load:

Loaded profile AC
BTN1 AC value: 0x123456
BTN1 AC proto: COOLIX
BTN1 raw len: 68

On Command Send:

Sending AC cmd for BTN1: proto=COOLIX, value=0x123456, len=0, raw_len=68
SENT COOLIX
IR receiver re-enabled after sendACCmd

ContributingContributions are welcome! Please:Fork the repository.
Create a feature branch (git checkout -b feature/your-feature).
Commit changes (git commit -m 'Add your feature').
Push to the branch (git push origin feature/your-feature).
Open a Pull Request.

LicenseThis project is licensed under the MIT License. See the LICENSE file for details.AcknowledgmentsIRremoteESP8266 for IR protocol support.
Adafruit_SSD1306 for OLED display drivers.
NimBLE-Arduino and BleKeyboard for Bluetooth functionality.


