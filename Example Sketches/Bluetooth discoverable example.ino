#include <Arduino.h>
#include <NimBLEDevice.h>
#include <BleKeyboard.h>

#define DEBUG
#define BT_DEVICE_NAME "ESP32_MediaKey"
#define BUTTON_PIN 5 // Command and switch button
#define SHORT_PRESS_TIME 1000 // 1s for short press
#define DEBOUNCE_DELAY 50 // 50ms debounce

BleKeyboard bleKeyboard(BT_DEVICE_NAME, "ESP32NimBLE", 100);
bool isBLEInitialized = false;
unsigned long lastAdvertisingStart = 0;
unsigned long connectStartTime = 0;
const unsigned long advertisingTimeout = 10000; // 10s timeout
bool lastButtonState = HIGH; // Pull-up, HIGH = not pressed
unsigned long buttonPressStartTime = 0;
bool buttonHandled = false;
int connectedDeviceIndex = 0; // 0 = none, 1 = Android, 2 = Windows
String deviceNames[] = {"None", "Android", "Windows"};

void resetBLE(bool clearBonds = false) {
  if (isBLEInitialized) {
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    if (pAdvertising) {
      pAdvertising->stop();
#ifdef DEBUG
      Serial.println("Advertising stopped");
#endif
    }
    bleKeyboard.end();
    if (clearBonds) {
      NimBLEDevice::deleteAllBonds();
#ifdef DEBUG
      Serial.println("Bonding data cleared");
#endif
    }
    delay(200); // Stabilize stack
    isBLEInitialized = false;
#ifdef DEBUG
    Serial.println("BLE reset complete");
#endif
  }
  NimBLEDevice::init(BT_DEVICE_NAME);
  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  bleKeyboard.begin();
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setMinInterval(32); // 20ms
  pAdvertising->setMaxInterval(64); // 40ms
  pAdvertising->addServiceUUID(NimBLEUUID("1812"));
  pAdvertising->setName(BT_DEVICE_NAME);
  pAdvertising->setAppearance(0x03C0);
  pAdvertising->start();
  lastAdvertisingStart = millis();
  isBLEInitialized = true;
#ifdef DEBUG
  Serial.println("Bluetooth initialized with HID service");
#endif
}

void switchDevice() {
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  if (pAdvertising) {
    pAdvertising->stop();
#ifdef DEBUG
    Serial.println("Advertising stopped for switch");
#endif
  }
  bleKeyboard.end();
#ifdef DEBUG
  Serial.println("Disconnected to switch device");
#endif
  delay(200); // Stabilize stack
  connectedDeviceIndex = (connectedDeviceIndex % 2) + 1;
#ifdef DEBUG
  Serial.printf("Switching to device: %s\n", deviceNames[connectedDeviceIndex].c_str());
#endif
  // Reset button state to ensure functionality post-switch
  lastButtonState = HIGH;
  buttonPressStartTime = 0;
  buttonHandled = false;
  // Restart advertising
  pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setMinInterval(32);
  pAdvertising->setMaxInterval(64);
  pAdvertising->addServiceUUID(NimBLEUUID("1812"));
  pAdvertising->setName(BT_DEVICE_NAME);
  pAdvertising->setAppearance(0x03C0);
  pAdvertising->start();
  lastAdvertisingStart = millis();
#ifdef DEBUG
  Serial.println("Advertising restarted for new device");
#endif
}

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  delay(100);
  Serial.println("Setup started");
#endif
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  resetBLE(false);
}

void loop() {
  // Manual BLE reset ('r' for reset, 'b' for reset with bond clear)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      resetBLE(false);
    } else if (c == 'b') {
      resetBLE(true);
    }
  }

  // Button handling (short press: media, long press: switch)
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (currentButtonState != lastButtonState) {
    if (currentButtonState == LOW) { // Button pressed
      buttonPressStartTime = millis();
      buttonHandled = false;
#ifdef DEBUG
      Serial.println("Button pressed");
#endif
    } else if (currentButtonState == HIGH && !buttonHandled) { // Button released
      unsigned long pressDuration = millis() - buttonPressStartTime;
      if (pressDuration < SHORT_PRESS_TIME && bleKeyboard.isConnected()) { // Short press
        bleKeyboard.releaseAll();
        bleKeyboard.write(KEY_MEDIA_PLAY_PAUSE);
#ifdef DEBUG
        Serial.printf("Short press: Sent Play/Pause to %s\n", deviceNames[connectedDeviceIndex].c_str());
#endif
        delay(100);
        bleKeyboard.releaseAll();
      }
      buttonHandled = true;
    }
  }
  // Check for long press while button is held
  if (currentButtonState == LOW && !buttonHandled && millis() - buttonPressStartTime >= SHORT_PRESS_TIME) {
    switchDevice();
    buttonHandled = true;
  }
  lastButtonState = currentButtonState;

  if (bleKeyboard.isConnected()) {
    if (connectStartTime > 0) {
#ifdef DEBUG
      Serial.printf("Connected to %s in %lu ms\n", deviceNames[connectedDeviceIndex].c_str(), millis() - connectStartTime);
#endif
      connectStartTime = 0;
      bleKeyboard.releaseAll();
      delay(200);
#ifdef DEBUG
      Serial.println("Released all keys to refresh HID service");
#endif
    }
  } else {
#ifdef DEBUG
    Serial.println("Not connected, advertising...");
#endif
    if (connectStartTime == 0) {
      connectStartTime = millis();
    }
    if (millis() - lastAdvertisingStart >= advertisingTimeout) {
      NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
      pAdvertising->stop();
      delay(50);
      pAdvertising->setMinInterval(32);
      pAdvertising->setMaxInterval(64);
      pAdvertising->addServiceUUID(NimBLEUUID("1812"));
      pAdvertising->setName(BT_DEVICE_NAME);
      pAdvertising->setAppearance(0x03C0);
      pAdvertising->start();
      lastAdvertisingStart = millis();
#ifdef DEBUG
      Serial.println("Advertising restarted");
#endif
    }
  }
  delay(20);
}