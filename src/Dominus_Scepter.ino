#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <ir_Voltas.h>
#include <ir_Coolix.h>
#include <ir_Daikin.h>
#include <ir_Mitsubishi.h>
#include <ir_Toshiba.h>
#include <ir_Fujitsu.h>
#include <ir_LG.h>
#include <ir_Haier.h>
#include <ir_Hitachi.h>
#include <ir_Panasonic.h>
#include <ir_Samsung.h>
#include <ir_Whirlpool.h>
#include <ir_Gree.h>
#include <ir_Midea.h>
#include <ir_Tcl.h>
#include <ir_Carrier.h>
#include <ir_Sharp.h>
#include <IRac.h>
#include <IRutils.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <BleKeyboard.h>
#include <esp_task_wdt.h>

#define DECODE_HASH
//#define DEBUG // Comment out for release build
// Change Partition Sceheme to No OTA (2MB APP / 2MB SPIFFS)  for serial debugging 
// ==== Battery Monitoring ====
#define BATTERY_PIN 34
#define VOLTAGE_DIVIDER_RATIO 2.993
#define ADC_VREF 2573
#define ADC_MAX 4095
#define BATT_MIN_V 2.9
#define BATT_MAX_V 4.2
#define BATT_UPDATE_INTERVAL 30000
#define ADC_SAMPLES 64

// ==== OLED ====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C

// ==== Rotary Encoder ====
#define CLK_PIN 25
#define DT_PIN 26
#define SW_PIN 27

// ==== Buttons ====
#define BTN1_PIN 33 // Play/Pause
#define BTN2_PIN 32 // Volume Up
#define BTN3_PIN 13 // Mute
#define BTN4_PIN 12 // Volume Down
#define REC_BTN_PIN 5 // Device switch in BT mode

// ==== IR ====
#define IR_SEND_PIN 4
#define IR_RECV_PIN 15
#define DEBUG_LED 2

// ===== AC-OPTIMIZED IR SETTINGS =====
const uint16_t kCaptureBufferSize = 2048;
const uint16_t kMaxRawLenNonAC = 1024;
const uint8_t kTimeout = 100;
const uint16_t kMinUnknownSize = 12;
const int MAX_RAW_LEN = 500;
const uint16_t MAX_AC_STATE_LEN = 32;
const int AC_RAW_FREQ = 38;
const int TV_RAW_FREQ = 38;
const int SPK_RAW_FREQ = 38;

// Deep sleep timeout
#define SLEEP_TIMEOUT 20000

// Bluetooth settings
#define BT_DEVICE_NAME "Dominus Scepter"
BleKeyboard bleKeyboard(BT_DEVICE_NAME, "ESP32", 100);
bool isBLEInitialized = false;
unsigned long lastAdvertisingStart = 0;
const unsigned long advertisingTimeout = 30000;
String deviceNames[] = {"None", "Android", "Windows"};
int connectedDeviceIndex = 0;

RTC_DATA_ATTR int savedCurrentIndex = 0;

// Volatile variables for ISR
volatile bool encoderChanged = false;
volatile int encoderDirection = 0;
volatile bool buttonPressed[5] = {false};
volatile bool encoderBtnPressed = false;
volatile unsigned long lastButtonTime = 0;
const unsigned long DEBOUNCE_TIME = 200;

bool isDisplayOn = true;
bool needsDisplayUpdate = false;
bool wdtInitialized = false;
int batteryPercentage = 100;
unsigned long lastBatteryUpdate = 0;

struct IRCode {
  uint16_t data[MAX_RAW_LEN];
  uint16_t length = 0;
};

struct ACCode {
  uint8_t data[MAX_AC_STATE_LEN];
  uint16_t length = 0;
  decode_type_t protocol = UNKNOWN;
  String description = "";
  uint64_t value = 0;
};

struct RemoteProfile {
  char name[10];
  uint64_t command1, command2, command3, command4;
  int8_t proto1, proto2, proto3, proto4;
  IRCode raw1, raw2, raw3, raw4;
  ACCode ac1, ac2, ac3, ac4;
};

RemoteProfile remotes[] = {
  {"TV", 0, 0, 0, 0, -1, -1, -1, -1, {}, {}, {}, {}, {}, {}, {}, {}},
  {"AC", 0, 0, 0, 0, -1, -1, -1, -1, {}, {}, {}, {}, {}, {}, {}, {}},
  {"SPK", 0, 0, 0, 0, -1, -1, -1, -1, {}, {}, {}, {}, {}, {}, {}, {}},
  {"AC_X", 0, 0, 0, 0, -1, -1, -1, -1, {}, {}, {}, {}, {}, {}, {}, {}},
  {"BT", 0, 0, 0, 0, -1, -1, -1, -1, {}, {}, {}, {}, {}, {}, {}, {}}
};

const int numRemotes = sizeof(remotes) / sizeof(remotes[0]);

int currentIndex = 0;
int highlighted = 0;
bool isMenu = true;
bool recordMode = false;
bool hasReceived = false;
decode_results lastReceived;
uint64_t lastCapturedCode = 0;
int8_t lastCapturedProto = -1;
ACCode lastCapturedAC = {{0}, 0, UNKNOWN, "", 0};
unsigned long lastBlink = 0;
bool recVisible = false;
unsigned long lastDisplayUpdate = 0;
unsigned long lastActivity = 0;
unsigned long feedbackStart = 0;
bool showingFeedback = false;

IRrecv irrecv(IR_RECV_PIN, kCaptureBufferSize, kTimeout, true);
IRsend irsend(IR_SEND_PIN);
IRac ac(IR_SEND_PIN);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Preferences prefs;

int voltageToPercentage(float voltage) {
  if (voltage >= 4.20) return 100;
  if (voltage <= 2.90) return 0;
  float voltagePoints[] = {4.20, 4.10, 4.00, 3.90, 3.80, 3.70, 3.60, 3.50, 3.20, 2.90};
  int percentagePoints[] = {100, 90, 80, 70, 50, 30, 20, 10, 5, 0};
  int numPoints = 10;
  for (int i = 0; i < numPoints - 1; i++) {
    if (voltage <= voltagePoints[i] && voltage > voltagePoints[i + 1]) {
      float voltageRange = voltagePoints[i] - voltagePoints[i + 1];
      int percentageRange = percentagePoints[i] - percentagePoints[i + 1];
      float fraction = (voltage - voltagePoints[i + 1]) / voltageRange;
      return percentagePoints[i + 1] + (int)(fraction * percentageRange + 0.5);
    }
  }
  return 0;
}

void updateBatteryPercentage() {
  long adcSum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    adcSum += analogRead(BATTERY_PIN);
    ets_delay_us(100);
  }
  int adcValue = adcSum / ADC_SAMPLES;
  if (adcValue < 100 || adcValue > 4000) {
#ifdef DEBUG
    Serial.println("Error: Invalid ADC reading!");
    Serial.printf("Raw ADC: %d\n", adcValue);
#endif
    lastBatteryUpdate = millis();
    return;
  }
  float voltage = (adcValue * (ADC_VREF / 1000.0) / ADC_MAX) * VOLTAGE_DIVIDER_RATIO;
  batteryPercentage = voltageToPercentage(voltage);
#ifdef DEBUG
  Serial.printf("Battery ADC: %d, Voltage: %.2fV, Percentage: %d%%\n", adcValue, voltage, batteryPercentage);
#endif
  lastBatteryUpdate = millis();
}

void resetBLE(bool clearBonds = false) {
  if (isBLEInitialized) {
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    if (pAdvertising) pAdvertising->stop();
    bleKeyboard.end();
    if (clearBonds) NimBLEDevice::deleteAllBonds();
    delay(200);
    isBLEInitialized = false;
  }
  NimBLEDevice::init(BT_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(185);
  bleKeyboard.begin();
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setMinInterval(80);
  pAdvertising->setMaxInterval(128);
  pAdvertising->addServiceUUID(NimBLEUUID("1812"));
  pAdvertising->setName(BT_DEVICE_NAME);
  pAdvertising->setAppearance(0x03C0);
  pAdvertising->start();
  lastAdvertisingStart = millis();
  isBLEInitialized = true;
#ifdef DEBUG
  Serial.println("Bluetooth initialized");
#endif
  showFeedback("Pair with PC");
}

void IRAM_ATTR handleEncoder() {
  static int lastClk = HIGH;
  int clk = digitalRead(CLK_PIN);
  if (lastClk != clk) {
    int dt = digitalRead(DT_PIN);
    encoderDirection = (clk != dt) ? 1 : -1;
    encoderChanged = true;
  }
  lastClk = clk;
}

void IRAM_ATTR handleEncoderButton() {
  static unsigned long lastInterruptTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > DEBOUNCE_TIME) {
    encoderBtnPressed = true;
    lastButtonTime = currentTime;
    lastInterruptTime = currentTime;
  }
}

void IRAM_ATTR handleButton() {
  static unsigned long lastInterruptTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime < DEBOUNCE_TIME) return;
  lastInterruptTime = currentTime;
  lastButtonTime = currentTime;
  if (digitalRead(BTN1_PIN) == LOW) buttonPressed[0] = true;
  else if (digitalRead(BTN2_PIN) == LOW) buttonPressed[1] = true;
  else if (digitalRead(BTN3_PIN) == LOW) buttonPressed[2] = true;
  else if (digitalRead(BTN4_PIN) == LOW) buttonPressed[3] = true;
  else if (digitalRead(REC_BTN_PIN) == LOW) buttonPressed[4] = true;
}

void checkWakeupButtons() {
  if (encoderBtnPressed) {
    noInterrupts();
    encoderBtnPressed = false;
    interrupts();
#ifdef DEBUG
    Serial.println("Woke up via SW_PIN press");
#endif
    isMenu = !isMenu;
    if (isMenu) {
      highlighted = currentIndex;
      showMenu();
    } else {
      currentIndex = highlighted;
      prefs.putInt("currentIndex", currentIndex);
      showSelected();
    }
    lastActivity = millis();
  }
}

void manageDisplayPower() {}

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  delay(100);
  Serial.println("Setup started");
#endif
  analogReadResolution(12);
  pinMode(BATTERY_PIN, INPUT);
  updateBatteryPercentage();
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(BTN4_PIN, INPUT_PULLUP);
  pinMode(REC_BTN_PIN, INPUT_PULLUP);
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_PIN), handleEncoderButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN1_PIN), handleButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN2_PIN), handleButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN3_PIN), handleButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN4_PIN), handleButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(REC_BTN_PIN), handleButton, FALLING);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)SW_PIN, 0);
  esp_task_wdt_config_t wdt_config = {.timeout_ms = 15000, .idle_core_mask = 0, .trigger_panic = false};
  if (esp_task_wdt_init(&wdt_config) == ESP_OK) {
    wdtInitialized = true;
    esp_task_wdt_add(NULL);
  }
  WiFi.mode(WIFI_OFF);
  btStop();
  irsend.begin();
  irrecv.setUnknownThreshold(kMinUnknownSize);
  irrecv.disableIRIn(); // Initialize as disabled
  irrecv.enableIRIn();
#ifdef DEBUG
  Serial.println("IR receiver enabled in setup");
#endif
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
#ifdef DEBUG
    Serial.println("SSD1306 allocation failed");
#endif
    for (;;);
  }
  Wire.setClock(100000);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.ssd1306_command(SSD1306_DISPLAYON);
  isDisplayOn = true;
  display.display();
  if (!prefs.begin("IRRemote", false)) {
#ifdef DEBUG
    Serial.println("Failed to initialize Preferences");
#endif
    showFeedback("Prefs Init Failed!");
    currentIndex = highlighted = 0;
  } else {
    loadProfiles();
    highlighted = currentIndex = prefs.getInt("currentIndex", currentIndex);
    if (currentIndex < 0 || currentIndex >= numRemotes) currentIndex = highlighted = 0;
  }
  resetBLE(false);
  showMenu();
  lastActivity = millis();
  isDisplayOn = true;
  needsDisplayUpdate = true;
}

#ifdef DEBUG
void debugProfile(int profile) {
  Serial.println("Profile: " + String(remotes[profile].name));
  for (int i = 1; i <= 4; i++) {
    uint64_t cmd = 0;
    int8_t proto = -1;
    uint16_t rawLen = 0;
    uint16_t acLen = 0;
    uint64_t acValue = 0;
    decode_type_t acProto = UNKNOWN;
    switch (i) {
      case 1: cmd = remotes[profile].command1; proto = remotes[profile].proto1; rawLen = remotes[profile].raw1.length; acLen = remotes[profile].ac1.length; acValue = remotes[profile].ac1.value; acProto = remotes[profile].ac1.protocol; break;
      case 2: cmd = remotes[profile].command2; proto = remotes[profile].proto2; rawLen = remotes[profile].raw2.length; acLen = remotes[profile].ac2.length; acValue = remotes[profile].ac2.value; acProto = remotes[profile].ac2.protocol; break;
      case 3: cmd = remotes[profile].command3; proto = remotes[profile].proto3; rawLen = remotes[profile].raw3.length; acLen = remotes[profile].ac3.length; acValue = remotes[profile].ac3.value; acProto = remotes[profile].ac3.protocol; break;
      case 4: cmd = remotes[profile].command4; proto = remotes[profile].proto4; rawLen = remotes[profile].raw4.length; acLen = remotes[profile].ac4.length; acValue = remotes[profile].ac4.value; acProto = remotes[profile].ac4.protocol; break;
    }
    Serial.printf("BTN%d cmd: 0x%llX\n", i, cmd);
    Serial.println("BTN" + String(i) + " proto: " + getProtocolName(proto));
    Serial.println("BTN" + String(i) + " raw len: " + String(rawLen));
    Serial.println("BTN" + String(i) + " AC len: " + String(acLen));
    Serial.println("BTN" + String(i) + " AC value: 0x" + String(acValue, HEX));
    Serial.println("BTN" + String(i) + " AC proto: " + String(typeToString(acProto)));
  }
}
#endif

String getProtocolName(int8_t proto) {
  switch (proto) {
    case NEC: return "NEC";
    case SONY: return "SONY";
    case RC5: return "RC5";
    case RC6: return "RC6";
    case PANASONIC: return "PANASONIC";
    case JVC: return "JVC";
    case SAMSUNG: return "SAMSUNG";
    case LG: return "LG";
    case SHARP: return "SHARP";
    case RC5X: return "RC5X";
    case DENON: return "DENON";
    case AIWA_RC_T501: return "AIWA";
    case PIONEER: return "PIONEER";
    case COOLIX: return "COOLIX";
    case VOLTAS: return "VOLTAS";
    case DAIKIN: return "DAIKIN";
    case MITSUBISHI: return "MITSUBISHI";
    case TOSHIBA_AC: return "TOSHIBA_AC";
    case FUJITSU_AC: return "FUJITSU_AC";
    case HAIER_AC: return "HAIER_AC";
    case HITACHI_AC: return "HITACHI_AC";
    case GREE: return "GREE";
    case MIDEA: return "MIDEA";
    case TCL112AC: return "TCL112AC";
    case CARRIER_AC: return "CARRIER_AC";
    case WHIRLPOOL_AC: return "WHIRLPOOL_AC";
    case UNKNOWN: return "UNKNOWN";
    default: return "INVALID";
  }
}

bool isProtocolSupported(int8_t proto) {
  switch (proto) {
    case NEC: case SONY: case RC5: case RC6: case PANASONIC: case JVC:
    case SAMSUNG: case LG: case SHARP: case RC5X: case DENON:
    case AIWA_RC_T501: case PIONEER: return true;
    default: return false;
  }
}

bool isACProtocol(int8_t proto) {
  switch (proto) {
    case COOLIX: case VOLTAS: case DAIKIN: case MITSUBISHI: case TOSHIBA_AC:
    case FUJITSU_AC: case HAIER_AC: case HITACHI_AC: case GREE: case MIDEA:
    case TCL112AC: case CARRIER_AC: case WHIRLPOOL_AC: return true;
    default: return false;
  }
}

void loadProfiles() {
  for (int i = 0; i < numRemotes; i++) {
    String prefix = String(remotes[i].name);
    char k1[20], k2[20], k3[20], k4[20];
    sprintf(k1, "%s_cmd1", remotes[i].name);
    sprintf(k2, "%s_cmd2", remotes[i].name);
    sprintf(k3, "%s_cmd3", remotes[i].name);
    sprintf(k4, "%s_cmd4", remotes[i].name);
    remotes[i].command1 = prefs.getULong64(k1, 0);
    remotes[i].command2 = prefs.getULong64(k2, 0);
    remotes[i].command3 = prefs.getULong64(k3, 0);
    remotes[i].command4 = prefs.getULong64(k4, 0);
    sprintf(k1, "%s_proto1", remotes[i].name);
    sprintf(k2, "%s_proto2", remotes[i].name);
    sprintf(k3, "%s_proto3", remotes[i].name);
    sprintf(k4, "%s_proto4", remotes[i].name);
    remotes[i].proto1 = prefs.getChar(k1, -1);
    remotes[i].proto2 = prefs.getChar(k2, -1);
    remotes[i].proto3 = prefs.getChar(k3, -1);
    remotes[i].proto4 = prefs.getChar(k4, -1);
    sprintf(k1, "%s_raw1", remotes[i].name);
    sprintf(k2, "%s_raw2", remotes[i].name);
    sprintf(k3, "%s_raw3", remotes[i].name);
    sprintf(k4, "%s_raw4", remotes[i].name);
    size_t len = prefs.getBytes(k1, remotes[i].raw1.data, MAX_RAW_LEN * sizeof(uint16_t));
    remotes[i].raw1.length = (len > 0 && len % sizeof(uint16_t) == 0) ? len / sizeof(uint16_t) : 0;
    len = prefs.getBytes(k2, remotes[i].raw2.data, MAX_RAW_LEN * sizeof(uint16_t));
    remotes[i].raw2.length = (len > 0 && len % sizeof(uint16_t) == 0) ? len / sizeof(uint16_t) : 0;
    len = prefs.getBytes(k3, remotes[i].raw3.data, MAX_RAW_LEN * sizeof(uint16_t));
    remotes[i].raw3.length = (len > 0 && len % sizeof(uint16_t) == 0) ? len / sizeof(uint16_t) : 0;
    len = prefs.getBytes(k4, remotes[i].raw4.data, MAX_RAW_LEN * sizeof(uint16_t));
    remotes[i].raw4.length = (len > 0 && len % sizeof(uint16_t) == 0) ? len / sizeof(uint16_t) : 0;
    sprintf(k1, "%s_ac1", remotes[i].name);
    sprintf(k2, "%s_ac2", remotes[i].name);
    sprintf(k3, "%s_ac3", remotes[i].name);
    sprintf(k4, "%s_ac4", remotes[i].name);
    len = prefs.getBytes(k1, remotes[i].ac1.data, MAX_AC_STATE_LEN);
    remotes[i].ac1.length = len;
    len = prefs.getBytes(k2, remotes[i].ac2.data, MAX_AC_STATE_LEN);
    remotes[i].ac2.length = len;
    len = prefs.getBytes(k3, remotes[i].ac3.data, MAX_AC_STATE_LEN);
    remotes[i].ac3.length = len;
    len = prefs.getBytes(k4, remotes[i].ac4.data, MAX_AC_STATE_LEN);
    remotes[i].ac4.length = len;
    sprintf(k1, "%s_ac1_proto", remotes[i].name);
    sprintf(k2, "%s_ac2_proto", remotes[i].name);
    sprintf(k3, "%s_ac3_proto", remotes[i].name);
    sprintf(k4, "%s_ac4_proto", remotes[i].name);
    remotes[i].ac1.protocol = (decode_type_t)prefs.getChar(k1, UNKNOWN);
    remotes[i].ac2.protocol = (decode_type_t)prefs.getChar(k2, UNKNOWN);
    remotes[i].ac3.protocol = (decode_type_t)prefs.getChar(k3, UNKNOWN);
    remotes[i].ac4.protocol = (decode_type_t)prefs.getChar(k4, UNKNOWN);
    sprintf(k1, "%s_ac1_desc", remotes[i].name);
    sprintf(k2, "%s_ac2_desc", remotes[i].name);
    sprintf(k3, "%s_ac3_desc", remotes[i].name);
    sprintf(k4, "%s_ac4_desc", remotes[i].name);
    remotes[i].ac1.description = prefs.getString(k1, "");
    remotes[i].ac2.description = prefs.getString(k2, "");
    remotes[i].ac3.description = prefs.getString(k3, "");
    remotes[i].ac4.description = prefs.getString(k4, "");
    sprintf(k1, "%s_ac1_value", remotes[i].name);
    sprintf(k2, "%s_ac2_value", remotes[i].name);
    sprintf(k3, "%s_ac3_value", remotes[i].name);
    sprintf(k4, "%s_ac4_value", remotes[i].name);
    remotes[i].ac1.value = prefs.getULong64(k1, 0);
    remotes[i].ac2.value = prefs.getULong64(k2, 0);
    remotes[i].ac3.value = prefs.getULong64(k3, 0);
    remotes[i].ac4.value = prefs.getULong64(k4, 0);
#ifdef DEBUG
    Serial.printf("Loaded profile %s\n", remotes[i].name);
    debugProfile(i);
#endif
  }
}

void saveProfiles() {
  bool saveSuccess = true;
  for (int i = 0; i < numRemotes; i++) {
    String prefix = String(remotes[i].name);
    char k1[20], k2[20], k3[20], k4[20];
    sprintf(k1, "%s_cmd1", remotes[i].name);
    sprintf(k2, "%s_cmd2", remotes[i].name);
    sprintf(k3, "%s_cmd3", remotes[i].name);
    sprintf(k4, "%s_cmd4", remotes[i].name);
    saveSuccess &= prefs.putULong64(k1, remotes[i].command1);
    saveSuccess &= prefs.putULong64(k2, remotes[i].command2);
    saveSuccess &= prefs.putULong64(k3, remotes[i].command3);
    saveSuccess &= prefs.putULong64(k4, remotes[i].command4);
    sprintf(k1, "%s_proto1", remotes[i].name);
    sprintf(k2, "%s_proto2", remotes[i].name);
    sprintf(k3, "%s_proto3", remotes[i].name);
    sprintf(k4, "%s_proto4", remotes[i].name);
    saveSuccess &= prefs.putChar(k1, remotes[i].proto1);
    saveSuccess &= prefs.putChar(k2, remotes[i].proto2);
    saveSuccess &= prefs.putChar(k3, remotes[i].proto3);
    saveSuccess &= prefs.putChar(k4, remotes[i].proto4);
    sprintf(k1, "%s_raw1", remotes[i].name);
    sprintf(k2, "%s_raw2", remotes[i].name);
    sprintf(k3, "%s_raw3", remotes[i].name);
    sprintf(k4, "%s_raw4", remotes[i].name);
    if (remotes[i].raw1.length > 0) saveSuccess &= prefs.putBytes(k1, remotes[i].raw1.data, remotes[i].raw1.length * sizeof(uint16_t));
    if (remotes[i].raw2.length > 0) saveSuccess &= prefs.putBytes(k2, remotes[i].raw2.data, remotes[i].raw2.length * sizeof(uint16_t));
    if (remotes[i].raw3.length > 0) saveSuccess &= prefs.putBytes(k3, remotes[i].raw3.data, remotes[i].raw3.length * sizeof(uint16_t));
    if (remotes[i].raw4.length > 0) saveSuccess &= prefs.putBytes(k4, remotes[i].raw4.data, remotes[i].raw4.length * sizeof(uint16_t));
    sprintf(k1, "%s_ac1", remotes[i].name);
    sprintf(k2, "%s_ac2", remotes[i].name);
    sprintf(k3, "%s_ac3", remotes[i].name);
    sprintf(k4, "%s_ac4", remotes[i].name);
    if (remotes[i].ac1.length > 0) saveSuccess &= prefs.putBytes(k1, remotes[i].ac1.data, remotes[i].ac1.length);
    if (remotes[i].ac2.length > 0) saveSuccess &= prefs.putBytes(k2, remotes[i].ac2.data, remotes[i].ac2.length);
    if (remotes[i].ac3.length > 0) saveSuccess &= prefs.putBytes(k3, remotes[i].ac3.data, remotes[i].ac3.length);
    if (remotes[i].ac4.length > 0) saveSuccess &= prefs.putBytes(k4, remotes[i].ac4.data, remotes[i].ac4.length);
    sprintf(k1, "%s_ac1_proto", remotes[i].name);
    sprintf(k2, "%s_ac2_proto", remotes[i].name);
    sprintf(k3, "%s_ac3_proto", remotes[i].name);
    sprintf(k4, "%s_ac4_proto", remotes[i].name);
    saveSuccess &= prefs.putChar(k1, (char)remotes[i].ac1.protocol);
    saveSuccess &= prefs.putChar(k2, (char)remotes[i].ac2.protocol);
    saveSuccess &= prefs.putChar(k3, (char)remotes[i].ac3.protocol);
    saveSuccess &= prefs.putChar(k4, (char)remotes[i].ac4.protocol);
    sprintf(k1, "%s_ac1_desc", remotes[i].name);
    sprintf(k2, "%s_ac2_desc", remotes[i].name);
    sprintf(k3, "%s_ac3_desc", remotes[i].name);
    sprintf(k4, "%s_ac4_desc", remotes[i].name);
    saveSuccess &= prefs.putString(k1, remotes[i].ac1.description);
    saveSuccess &= prefs.putString(k2, remotes[i].ac2.description);
    saveSuccess &= prefs.putString(k3, remotes[i].ac3.description);
    saveSuccess &= prefs.putString(k4, remotes[i].ac4.description);
    sprintf(k1, "%s_ac1_value", remotes[i].name);
    sprintf(k2, "%s_ac2_value", remotes[i].name);
    sprintf(k3, "%s_ac3_value", remotes[i].name);
    sprintf(k4, "%s_ac4_value", remotes[i].name);
    saveSuccess &= prefs.putULong64(k1, remotes[i].ac1.value);
    saveSuccess &= prefs.putULong64(k2, remotes[i].ac2.value);
    saveSuccess &= prefs.putULong64(k3, remotes[i].ac3.value);
    saveSuccess &= prefs.putULong64(k4, remotes[i].ac4.value);
#ifdef DEBUG
    if (!saveSuccess) Serial.println("Save failed for " + prefix);
    Serial.println("Profiles saved for " + prefix);
    debugProfile(i);
#endif
  }
  saveSuccess &= prefs.putInt("currentIndex", currentIndex);
  if (!saveSuccess) showFeedback("Save Failed!");
}

void sendACCmd(int profile, int btn) {
  ACCode* acCode = nullptr;
  IRCode* rawCode = nullptr;
  switch (btn) {
    case 1: acCode = &remotes[profile].ac1; rawCode = &remotes[profile].raw1; break;
    case 2: acCode = &remotes[profile].ac2; rawCode = &remotes[profile].raw2; break;
    case 3: acCode = &remotes[profile].ac3; rawCode = &remotes[profile].raw3; break;
    case 4: acCode = &remotes[profile].ac4; rawCode = &remotes[profile].raw4; break;
    default:
#ifdef DEBUG
      Serial.println("Invalid button");
#endif
      return;
  }
#ifdef DEBUG
  Serial.printf("Sending AC cmd for BTN%d: proto=%s, value=0x%llX, len=%d, raw_len=%d\n",
                btn, typeToString(acCode->protocol).c_str(), acCode->value, acCode->length, rawCode->length);
#endif
  bool commandSent = false;
  irrecv.disableIRIn();
  digitalWrite(DEBUG_LED, HIGH);
  if (acCode && (acCode->protocol != UNKNOWN && (acCode->length > 0 || acCode->value != 0))) {
    switch (acCode->protocol) {
      case VOLTAS: {
        IRVoltas acVoltas(IR_SEND_PIN);
        acVoltas.begin();
        acVoltas.setRaw(acCode->data);
        acVoltas.send();
#ifdef DEBUG
        Serial.println("SENT VOLTAS");
#endif
        break;
      }
      case COOLIX: {
        IRCoolixAC acCoolix(IR_SEND_PIN);
        acCoolix.begin();
        acCoolix.setRaw((uint32_t)acCode->value);
        acCoolix.send();
#ifdef DEBUG
        Serial.println("SENT COOLIX");
#endif
        break;
      }
      case DAIKIN: {
        IRDaikinESP acDaikin(IR_SEND_PIN);
        acDaikin.begin();
        acDaikin.setRaw(acCode->data);
        acDaikin.send();
#ifdef DEBUG
        Serial.println("SENT DAIKIN");
#endif
        break;
      }
      case MITSUBISHI: {
        IRMitsubishiAC acMitsubishi(IR_SEND_PIN);
        acMitsubishi.begin();
        acMitsubishi.setRaw(acCode->data);
        acMitsubishi.send();
#ifdef DEBUG
        Serial.println("SENT MITSUBISHI");
#endif
        break;
      }
      case TOSHIBA_AC: {
        IRToshibaAC acToshiba(IR_SEND_PIN);
        acToshiba.begin();
        acToshiba.setRaw(acCode->data);
        acToshiba.send();
#ifdef DEBUG
        Serial.println("SENT TOSHIBA_AC");
#endif
        break;
      }
      case FUJITSU_AC: {
        IRFujitsuAC acFujitsu(IR_SEND_PIN);
        acFujitsu.begin();
        acFujitsu.setRaw(acCode->data, acCode->length);
        acFujitsu.send();
#ifdef DEBUG
        Serial.println("SENT FUJITSU_AC");
#endif
        break;
      }
      case HAIER_AC: {
        IRHaierAC acHaier(IR_SEND_PIN);
        acHaier.begin();
        acHaier.setRaw(acCode->data);
        acHaier.send();
#ifdef DEBUG
        Serial.println("SENT HAIER_AC");
#endif
        break;
      }
      case HITACHI_AC: {
        IRHitachiAc acHitachi(IR_SEND_PIN);
        acHitachi.begin();
        acHitachi.setRaw(acCode->data);
        acHitachi.send();
#ifdef DEBUG
        Serial.println("SENT HITACHI_AC");
#endif
        break;
      }
      case GREE: {
        IRGreeAC acGree(IR_SEND_PIN);
        acGree.begin();
        acGree.setRaw(acCode->data);
        acGree.send();
#ifdef DEBUG
        Serial.println("SENT GREE");
#endif
        break;
      }
      case MIDEA: {
        IRMideaAC acMidea(IR_SEND_PIN);
        acMidea.begin();
        acMidea.setRaw(acCode->value);
        acMidea.send();
#ifdef DEBUG
        Serial.println("SENT MIDEA");
#endif
        break;
      }
      case TCL112AC: {
        IRTcl112Ac acTcl(IR_SEND_PIN);
        acTcl.begin();
        acTcl.setRaw(acCode->data);
        acTcl.send();
#ifdef DEBUG
        Serial.println("SENT TCL112AC");
#endif
        break;
      }
      case CARRIER_AC: {
        IRCarrierAc64 acCarrier(IR_SEND_PIN);
        acCarrier.begin();
        acCarrier.setRaw(acCode->value);
        acCarrier.send();
#ifdef DEBUG
        Serial.println("SENT CARRIER_AC");
#endif
        break;
      }
      case WHIRLPOOL_AC: {
        IRWhirlpoolAc acWhirlpool(IR_SEND_PIN);
        acWhirlpool.begin();
        acWhirlpool.setRaw(acCode->data);
        acWhirlpool.send();
#ifdef DEBUG
        Serial.println("SENT WHIRLPOOL_AC");
#endif
        break;
      }
      default:
        if (rawCode && rawCode->length > 0) {
          irsend.sendRaw(rawCode->data, rawCode->length, AC_RAW_FREQ);
#ifdef DEBUG
          Serial.println("SENT RAW");
#endif
          commandSent = true;
        }
        break;
    }
    commandSent = true;
  } else if (rawCode && rawCode->length > 0) {
    irsend.sendRaw(rawCode->data, rawCode->length, AC_RAW_FREQ);
#ifdef DEBUG
    Serial.println("SENT RAW");
#endif
    commandSent = true;
  }
  digitalWrite(DEBUG_LED, LOW);
  irrecv.enableIRIn();
#ifdef DEBUG
  Serial.println("IR receiver re-enabled after sendACCmd");
#endif
  if (!commandSent) {
#ifdef DEBUG
    Serial.println("No AC cmd for BTN" + String(btn));
#endif
    showFeedback("No Cmd Saved!");
  }
}

void sendCmd(int profile, int btn) {
  unsigned long startTime = millis();
#ifdef DEBUG
  Serial.println("Send " + String(remotes[profile].name) + ": BTN" + String(btn));
#endif
  if (strcmp(remotes[profile].name, "AC") == 0 || strcmp(remotes[profile].name, "AC_X") == 0) {
    sendACCmd(profile, btn);
  } else {
    IRCode* codeRaw = nullptr;
    uint64_t code = 0;
    int8_t proto = -1;
    switch (btn) {
      case 1: code = remotes[profile].command1; proto = remotes[profile].proto1; codeRaw = &remotes[profile].raw1; break;
      case 2: code = remotes[profile].command2; proto = remotes[profile].proto2; codeRaw = &remotes[profile].raw2; break;
      case 3: code = remotes[profile].command3; proto = remotes[profile].proto3; codeRaw = &remotes[profile].raw3; break;
      case 4: code = remotes[profile].command4; proto = remotes[profile].proto4; codeRaw = &remotes[profile].raw4; break;
      default:
#ifdef DEBUG
        Serial.println("Invalid button");
#endif
        return;
    }
#ifdef DEBUG
    Serial.printf("Sending cmd for BTN%d: proto=%s, value=0x%llX, raw_len=%d\n",
                  btn, getProtocolName(proto).c_str(), code, codeRaw->length);
#endif
    bool commandSent = false;
    irrecv.disableIRIn();
    digitalWrite(DEBUG_LED, HIGH);
    if (isProtocolSupported(proto) && code != 0) {
      switch (proto) {
        case NEC: irsend.sendNEC(code, 32); break;
        case SONY: irsend.sendSony(code, 12); break;
        case RC5: irsend.sendRC5(code, 12); break;
        case RC6: irsend.sendRC6(code, 20); break;
        case PANASONIC: irsend.sendPanasonic64(code); break;
        case JVC: irsend.sendJVC(code, 16); break;
        case SAMSUNG: irsend.sendSAMSUNG(code, 32); break;
        case LG: irsend.sendLG(code, 28); break;
        case SHARP: irsend.sendSharpRaw(code, 15); break;
        case RC5X: irsend.sendRC5(code, 12); break;
        case DENON: irsend.sendDenon(code, 14); break;
        case AIWA_RC_T501: irsend.sendAiwaRCT501(code); break;
        case PIONEER: irsend.sendPioneer(code, 64); break;
      }
#ifdef DEBUG
      Serial.println("SENT " + getProtocolName(proto));
#endif
      commandSent = true;
    } else if (codeRaw && codeRaw->length > 0) {
      irsend.sendRaw(codeRaw->data, codeRaw->length, strcmp(remotes[profile].name, "TV") == 0 ? TV_RAW_FREQ : SPK_RAW_FREQ);
#ifdef DEBUG
      Serial.println("SENT RAW");
#endif
      commandSent = true;
    }
    digitalWrite(DEBUG_LED, LOW);
    irrecv.enableIRIn();
#ifdef DEBUG
    Serial.println("IR receiver re-enabled after sendCmd");
#endif
    if (!commandSent) {
#ifdef DEBUG
      Serial.println("No cmd for BTN" + String(btn));
#endif
      showFeedback("No Cmd Saved!");
    }
  }
}

void sendBTCmd(int btn) {
  static unsigned long lastBTCmdTime = 0;
  const unsigned long btLockoutTime = 200;
  if (millis() - lastBTCmdTime < btLockoutTime) return;
  if (bleKeyboard.isConnected()) {
    bleKeyboard.releaseAll();
    switch (btn) {
      case 1: bleKeyboard.write(KEY_MEDIA_PLAY_PAUSE); showFeedback("Play/Pause Sent"); break;
      case 2: bleKeyboard.write(KEY_MEDIA_VOLUME_UP); showFeedback("Volume Up Sent"); break;
      case 3: bleKeyboard.write(KEY_MEDIA_MUTE); showFeedback("Mute Sent"); break;
      case 4: bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN); showFeedback("Volume Down Sent"); break;
      default: showFeedback("Invalid BT Cmd"); return;
    }
    delay(100);
    bleKeyboard.releaseAll();
#ifdef DEBUG
    Serial.printf("Sent BT command: BTN%d to %s\n", btn, deviceNames[connectedDeviceIndex].c_str());
#endif
    lastBTCmdTime = millis();
  } else {
    showFeedback("BT Not Connected!");
  }
}

void switchDevice() {
  connectedDeviceIndex = (connectedDeviceIndex % 2) + 1;
  if (!bleKeyboard.isConnected()) resetBLE(false);
  showFeedback(("Pair to " + deviceNames[connectedDeviceIndex]).c_str());
}

bool validateRawTimings(const decode_results& results) {
  for (int i = 1; i < results.rawlen; i++) {
    if (results.rawbuf[i] < 10 || results.rawbuf[i] > 20000) {
#ifdef DEBUG
      Serial.println("Invalid timing: " + String(results.rawbuf[i]) + "us");
#endif
      return false;
    }
  }
  return true;
}

void handleButtonPress(int btn) {
  lastActivity = millis();
  if (btn == 5) {
    if (strcmp(remotes[currentIndex].name, "BT") == 0) {
      static unsigned long lastRecPressTime = 0;
      const unsigned long lockoutTime = 200;
      if (millis() - lastRecPressTime >= lockoutTime) {
        switchDevice();
        lastRecPressTime = millis();
      }
    } else {
      recordMode = !recordMode;
      hasReceived = false;
      recVisible = false;
      lastBlink = millis();
      needsDisplayUpdate = true;
    }
    return;
  }
  if (strcmp(remotes[currentIndex].name, "BT") == 0 && !recordMode && !isMenu) {
    sendBTCmd(btn);
    return;
  }
  if (hasReceived) {
#ifdef DEBUG
    Serial.println("BTN" + String(btn) + " save");
#endif
    if (strcmp(remotes[currentIndex].name, "AC") == 0 || strcmp(remotes[currentIndex].name, "AC_X") == 0) {
      ACCode* acTarget = nullptr;
      IRCode* rawTarget = nullptr;
      switch (btn) {
        case 1: acTarget = &remotes[currentIndex].ac1; rawTarget = &remotes[currentIndex].raw1; break;
        case 2: acTarget = &remotes[currentIndex].ac2; rawTarget = &remotes[currentIndex].raw2; break;
        case 3: acTarget = &remotes[currentIndex].ac3; rawTarget = &remotes[currentIndex].raw3; break;
        case 4: acTarget = &remotes[currentIndex].ac4; rawTarget = &remotes[currentIndex].raw4; break;
      }
      if (acTarget && (lastCapturedAC.length > 0 || lastCapturedAC.value != 0)) {
        memcpy(acTarget->data, lastCapturedAC.data, lastCapturedAC.length);
        acTarget->length = lastCapturedAC.length;
        acTarget->protocol = lastCapturedAC.protocol;
        acTarget->description = lastCapturedAC.description;
        acTarget->value = lastCapturedAC.value;
#ifdef DEBUG
        Serial.println("Saved AC " + String(typeToString(acTarget->protocol)));
        Serial.printf("AC value: 0x%llX, len: %d\n", acTarget->value, acTarget->length);
#endif
      }
      if (rawTarget && lastReceived.rawlen > 0 && lastReceived.rawlen <= MAX_RAW_LEN && validateRawTimings(lastReceived)) {
        memcpy(rawTarget->data, (const uint16_t*)lastReceived.rawbuf, lastReceived.rawlen * sizeof(uint16_t));
        rawTarget->length = lastReceived.rawlen;
#ifdef DEBUG
        Serial.println("Saved AC raw: " + String(rawTarget->length));
#endif
      }
      if (acTarget->length > 0 || acTarget->value != 0 || rawTarget->length > 0) {
        saveProfiles();
        recordMode = false;
        hasReceived = false;
        recVisible = false;
        showFeedback(("Cmd" + String(btn) + " Saved").c_str());
      } else {
        showFeedback("Save Failed!");
      }
    } else {
      IRCode* rawTarget = nullptr;
      uint64_t* command = nullptr;
      int8_t* proto = nullptr;
      switch (btn) {
        case 1: rawTarget = &remotes[currentIndex].raw1; command = &remotes[currentIndex].command1; proto = &remotes[currentIndex].proto1; break;
        case 2: rawTarget = &remotes[currentIndex].raw2; command = &remotes[currentIndex].command2; proto = &remotes[currentIndex].proto2; break;
        case 3: rawTarget = &remotes[currentIndex].raw3; command = &remotes[currentIndex].command3; proto = &remotes[currentIndex].proto3; break;
        case 4: rawTarget = &remotes[currentIndex].raw4; command = &remotes[currentIndex].command4; proto = &remotes[currentIndex].proto4; break;
      }
      bool saved = false;
      if (lastCapturedProto != -1 && isProtocolSupported(lastCapturedProto) && lastCapturedCode != 0) {
        *command = lastCapturedCode;
        *proto = lastCapturedProto;
        saved = true;
      }
      if (rawTarget && lastReceived.rawlen > 0 && lastReceived.rawlen <= MAX_RAW_LEN && validateRawTimings(lastReceived)) {
        memcpy(rawTarget->data, (const uint16_t*)lastReceived.rawbuf, lastReceived.rawlen * sizeof(uint16_t));
        rawTarget->length = lastReceived.rawlen;
        saved = true;
      }
      if (saved) {
        saveProfiles();
        recordMode = false;
        hasReceived = false;
        recVisible = false;
        showFeedback(("Cmd" + String(btn) + " Saved").c_str());
      } else {
        showFeedback("Save Failed!");
      }
    }
  } else if (!recordMode && !isMenu) {
    sendCmd(currentIndex, btn);
  }
}

void showMenu() {
  display.ssd1306_command(SSD1306_DISPLAYON);
  isDisplayOn = true;
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("Select:");
  display.setTextSize(4);
  String text = String(remotes[highlighted].name);
  int textWidth = text.length() * 24;
  int x = (SCREEN_WIDTH - textWidth) / 2;
  if (x < 0) x = 0;
  display.setCursor(x, 20);
  display.println(text);
  if (recordMode && recVisible) {
    display.setTextSize(1);
    display.setCursor(SCREEN_WIDTH - 30, 8);
    display.println("REC");
  }
  display.setTextSize(1);
  display.setCursor(SCREEN_WIDTH - 30, 0);
  display.printf("%d%%", batteryPercentage);
  if (strcmp(remotes[currentIndex].name, "BT") == 0) {
    display.setCursor(0, 56);
    display.println(bleKeyboard.isConnected() ? "BT Connected" : "BT Disconnected");
  }
  display.display();
  lastDisplayUpdate = millis();
  lastActivity = millis();
  needsDisplayUpdate = false;
}

void showSelected() {
  display.ssd1306_command(SSD1306_DISPLAYON);
  isDisplayOn = true;
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("Selected:");
  display.setTextSize(2);
  String text = String("* ") + remotes[currentIndex].name + " *";
  int textWidth = text.length() * 12;
  int x = (SCREEN_WIDTH - textWidth) / 2;
  if (x < 0) x = 0;
  display.setCursor(x, 20);
  display.println(text);
  if (recordMode) {
    display.setTextSize(1);
    display.setCursor(0, 56);
    if (hasReceived) {
      display.println("Signal captured!");
      display.println("Press BTN1/2/3/4 to save");
    } else {
      display.println("Point remote & press key");
    }
    if (recVisible) {
      display.setTextSize(1);
      display.setCursor(SCREEN_WIDTH - 30, 8);
      display.println("REC");
    }
  } else {
    display.setTextSize(1);
    display.setCursor(0, 56);
    display.println(bleKeyboard.isConnected() ? "BT Connected" : "BT Disconnected");
  }
  display.setTextSize(1);
  display.setCursor(SCREEN_WIDTH - 30, 0);
  display.printf("%d%%", batteryPercentage);
  display.display();
  lastDisplayUpdate = millis();
  lastActivity = millis();
  needsDisplayUpdate = false;
}

void showFeedback(const char* msg) {
  if (!showingFeedback) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    isDisplayOn = true;
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("Feedback:");
    display.setTextSize(2);
    String text = String(remotes[currentIndex].name);
    int textWidth = text.length() * 12;
    int x = (SCREEN_WIDTH - textWidth) / 2;
    if (x < 0) x = 0;
    display.setCursor(x, 20);
    display.println(text);
    display.setTextSize(2);
    display.setCursor(0, 48);
    display.println(msg);
    display.setTextSize(1);
    display.setCursor(SCREEN_WIDTH - 30, 0);
    display.printf("%d%%", batteryPercentage);
    display.display();
    feedbackStart = millis();
    showingFeedback = true;
  }
}

void enterDeepSleep() {
#ifdef DEBUG
  Serial.println("Entering deep sleep...");
#endif
  if (wdtInitialized) {
    esp_task_wdt_reset();
    esp_task_wdt_deinit();
    wdtInitialized = false;
  }
  display.clearDisplay();
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  isDisplayOn = false;
  display.display();
  irrecv.disableIRIn();
  digitalWrite(DEBUG_LED, LOW);
  savedCurrentIndex = currentIndex;
  esp_deep_sleep_start();
}

#ifdef DEBUG
void debugIRCapture(const decode_results& results) {
  Serial.println("=== IR Capture Debug ===");
  Serial.println("Profile: " + String(remotes[currentIndex].name));
  Serial.println("Decode type: " + String(typeToString(results.decode_type)));
  Serial.println("Bits: " + String(results.bits));
  Serial.println("Value: 0x" + String(results.value, HEX));
  Serial.println("Raw length: " + String(results.rawlen));
  Serial.println("Overflow: " + String(results.overflow ? "Yes" : "No"));
  if (results.rawlen > 0) {
    Serial.print("Raw timings: ");
    for (int i = 1; i < results.rawlen; i++) {
      Serial.print(results.rawbuf[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  if (results.state != NULL && results.bits > 0) {
    Serial.print("State data: ");
    for (int i = 0; i < (results.bits + 7) / 8; i++) {
      Serial.print("0x");
      Serial.print(results.state[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("Is AC protocol: " + String(isACProtocol(results.decode_type) ? "Yes" : "No"));
  Serial.println("Valid signal: " + String((results.rawlen > 0 || results.decode_type != UNKNOWN) ? "Yes" : "No"));
  Serial.println("=====================");
}
#endif

void processButtons() {
  for (int i = 0; i < 5; i++) {
    if (buttonPressed[i]) {
      noInterrupts();
      buttonPressed[i] = false;
      interrupts();
#ifdef DEBUG
      Serial.println("BTN" + String(i + 1) + (i == 4 ? " (REC)" : "") + " pressed");
#endif
      handleButtonPress(i + 1);
      lastActivity = millis();
    }
  }
}

void loop() {
  if (wdtInitialized) esp_task_wdt_reset();
  if (millis() - lastBatteryUpdate >= BATT_UPDATE_INTERVAL) {
    updateBatteryPercentage();
    needsDisplayUpdate = true;
  }
  if (millis() - lastActivity >= SLEEP_TIMEOUT) enterDeepSleep();
  manageDisplayPower();
  if (showingFeedback && millis() - feedbackStart >= 500) {
    showingFeedback = false;
    needsDisplayUpdate = true;
  }
  if (recordMode && millis() - lastBlink >= 500 && millis() - lastDisplayUpdate >= 500) {
    recVisible = !recVisible;
    lastBlink = millis();
    needsDisplayUpdate = true;
  }
  static bool lastRecordMode = false;
  static bool lastHasReceived = false;
  static int lastHighlighted = -1;
  static int lastCurrentIndex = -1;
  if ((lastRecordMode != recordMode || lastHasReceived != hasReceived ||
       lastHighlighted != highlighted || lastCurrentIndex != currentIndex ||
       needsDisplayUpdate) && millis() - lastDisplayUpdate >= 500) {
    if (isMenu) showMenu();
    else showSelected();
    lastRecordMode = recordMode;
    lastHasReceived = hasReceived;
    lastHighlighted = highlighted;
    lastCurrentIndex = currentIndex;
  }
  if (encoderChanged && isMenu) {
    noInterrupts();
    highlighted = (highlighted + encoderDirection + numRemotes) % numRemotes;
    encoderChanged = false;
    encoderDirection = 0;
    interrupts();
    lastActivity = millis();
    needsDisplayUpdate = true;
  }
  if (encoderBtnPressed) {
    noInterrupts();
    encoderBtnPressed = false;
    interrupts();
    isMenu = !isMenu;
    if (isMenu) {
      highlighted = currentIndex;
      showMenu();
    } else {
      currentIndex = highlighted;
      prefs.putInt("currentIndex", currentIndex);
      showSelected();
    }
    lastActivity = millis();
    needsDisplayUpdate = true;
  }
  processButtons();
  if (!bleKeyboard.isConnected() && isBLEInitialized && millis() - lastAdvertisingStart >= advertisingTimeout) {
    resetBLE(false);
  }
  static unsigned long captureStart = 0;
  static bool captureWaiting = false;
  if (recordMode && !captureWaiting && irrecv.decode(&lastReceived)) {
    unsigned long startTime = millis();
    digitalWrite(DEBUG_LED, HIGH);
    captureStart = millis();
    captureWaiting = true;
#ifdef DEBUG
    debugIRCapture(lastReceived);
#endif
    bool validSignal = false;
    bool isACProfile = (strcmp(remotes[currentIndex].name, "AC") == 0 || strcmp(remotes[currentIndex].name, "AC_X") == 0);
    uint16_t maxRawLen = isACProfile ? kCaptureBufferSize : kMaxRawLenNonAC;
    if (lastReceived.overflow) {
      showFeedback("Buffer Overflow!");
    } else if (lastReceived.rawlen > 0 || lastReceived.decode_type != UNKNOWN) {
      if (lastReceived.rawlen > maxRawLen) {
        showFeedback("Raw Too Long!");
      } else {
        validSignal = true;
      }
    }
    if (validSignal && isACProfile) {
      lastCapturedAC.protocol = lastReceived.decode_type;
      lastCapturedAC.description = String(typeToString(lastReceived.decode_type)) + " Cmd";
      if (isACProtocol(lastReceived.decode_type) && lastReceived.state && lastReceived.bits > 0) {
        size_t stateLen = (lastReceived.bits + 7) / 8;
        if (stateLen > MAX_AC_STATE_LEN) {
          showFeedback("State Too Large!");
        } else {
          memcpy(lastCapturedAC.data, lastReceived.state, stateLen);
          lastCapturedAC.length = stateLen;
          lastCapturedAC.value = lastReceived.value;
          hasReceived = true;
        }
      } else if (lastReceived.bits <= 64) {
        lastCapturedAC.value = lastReceived.value;
        size_t stateLen = (lastReceived.bits + 7) / 8;
        if (stateLen > MAX_AC_STATE_LEN) {
          showFeedback("State Too Large!");
        } else {
          for (int i = 0; i < stateLen; i++) {
            lastCapturedAC.data[i] = (lastReceived.value >> (i * 8)) & 0xFF;
          }
          lastCapturedAC.length = stateLen;
          hasReceived = true;
        }
      }
      if (lastReceived.rawlen > 0 && lastReceived.rawlen <= MAX_RAW_LEN && validateRawTimings(lastReceived)) {
        hasReceived = true;
      }
      if (hasReceived) {
        irrecv.disableIRIn();
        if (isACProtocol(lastCapturedAC.protocol) && (lastCapturedAC.length > 0 || lastCapturedAC.value != 0)) {
          digitalWrite(DEBUG_LED, HIGH);
          switch (lastCapturedAC.protocol) {
            case VOLTAS: {
              IRVoltas acVoltas(IR_SEND_PIN);
              acVoltas.begin();
              acVoltas.setRaw(lastCapturedAC.data);
              acVoltas.send();
#ifdef DEBUG
              Serial.println("SENT VOLTAS TEST");
#endif
              break;
            }
            case COOLIX: {
              IRCoolixAC acCoolix(IR_SEND_PIN);
              acCoolix.begin();
              acCoolix.setRaw((uint32_t)lastCapturedAC.value);
              acCoolix.send();
#ifdef DEBUG
              Serial.println("SENT COOLIX TEST");
#endif
              break;
            }
            case DAIKIN: {
              IRDaikinESP acDaikin(IR_SEND_PIN);
              acDaikin.begin();
              acDaikin.setRaw(lastCapturedAC.data);
              acDaikin.send();
#ifdef DEBUG
              Serial.println("SENT DAIKIN TEST");
#endif
              break;
            }
            case MITSUBISHI: {
              IRMitsubishiAC acMitsubishi(IR_SEND_PIN);
              acMitsubishi.begin();
              acMitsubishi.setRaw(lastCapturedAC.data);
              acMitsubishi.send();
#ifdef DEBUG
              Serial.println("SENT MITSUBISHI TEST");
#endif
              break;
            }
            case TOSHIBA_AC: {
              IRToshibaAC acToshiba(IR_SEND_PIN);
              acToshiba.begin();
              acToshiba.setRaw(lastCapturedAC.data);
              acToshiba.send();
#ifdef DEBUG
              Serial.println("SENT TOSHIBA_AC TEST");
#endif
              break;
            }
            case FUJITSU_AC: {
              IRFujitsuAC acFujitsu(IR_SEND_PIN);
              acFujitsu.begin();
              acFujitsu.setRaw(lastCapturedAC.data, lastCapturedAC.length);
              acFujitsu.send();
#ifdef DEBUG
              Serial.println("SENT FUJITSU_AC TEST");
#endif
              break;
            }
            case HAIER_AC: {
              IRHaierAC acHaier(IR_SEND_PIN);
              acHaier.begin();
              acHaier.setRaw(lastCapturedAC.data);
              acHaier.send();
#ifdef DEBUG
              Serial.println("SENT HAIER_AC TEST");
#endif
              break;
            }
            case HITACHI_AC: {
              IRHitachiAc acHitachi(IR_SEND_PIN);
              acHitachi.begin();
              acHitachi.setRaw(lastCapturedAC.data);
              acHitachi.send();
#ifdef DEBUG
              Serial.println("SENT HITACHI_AC TEST");
#endif
              break;
            }
            case GREE: {
              IRGreeAC acGree(IR_SEND_PIN);
              acGree.begin();
              acGree.setRaw(lastCapturedAC.data);
              acGree.send();
#ifdef DEBUG
              Serial.println("SENT GREE TEST");
#endif
              break;
            }
            case MIDEA: {
              IRMideaAC acMidea(IR_SEND_PIN);
              acMidea.begin();
              acMidea.setRaw(lastCapturedAC.value);
              acMidea.send();
#ifdef DEBUG
              Serial.println("SENT MIDEA TEST");
#endif
              break;
            }
            case TCL112AC: {
              IRTcl112Ac acTcl(IR_SEND_PIN);
              acTcl.begin();
              acTcl.setRaw(lastCapturedAC.data);
              acTcl.send();
#ifdef DEBUG
              Serial.println("SENT TCL112AC TEST");
#endif
              break;
            }
            case CARRIER_AC: {
              IRCarrierAc64 acCarrier(IR_SEND_PIN);
              acCarrier.begin();
              acCarrier.setRaw(lastCapturedAC.value);
              acCarrier.send();
#ifdef DEBUG
              Serial.println("SENT CARRIER_AC TEST");
#endif
              break;
            }
            case WHIRLPOOL_AC: {
              IRWhirlpoolAc acWhirlpool(IR_SEND_PIN);
              acWhirlpool.begin();
              acWhirlpool.setRaw(lastCapturedAC.data);
              acWhirlpool.send();
#ifdef DEBUG
              Serial.println("SENT WHIRLPOOL_AC TEST");
#endif
              break;
            }
          }
          digitalWrite(DEBUG_LED, LOW);
        } else if (lastReceived.rawlen > 0) {
          digitalWrite(DEBUG_LED, HIGH);
          irsend.sendRaw((uint16_t*)lastReceived.rawbuf, lastReceived.rawlen, AC_RAW_FREQ);
          digitalWrite(DEBUG_LED, LOW);
#ifdef DEBUG
          Serial.println("SENT RAW TEST");
#endif
        }
        irrecv.enableIRIn();
#ifdef DEBUG
        Serial.println("IR receiver re-enabled after AC test");
#endif
      }
    } else if (validSignal) {
      if (lastReceived.decode_type != UNKNOWN && isProtocolSupported(lastReceived.decode_type) && lastReceived.value != 0) {
        lastCapturedCode = lastReceived.value;
        lastCapturedProto = lastReceived.decode_type;
        hasReceived = true;
        irrecv.disableIRIn();
        digitalWrite(DEBUG_LED, HIGH);
        switch (lastCapturedProto) {
          case NEC: irsend.sendNEC(lastCapturedCode, 32); break;
          case SONY: irsend.sendSony(lastCapturedCode, 12); break;
          case RC5: irsend.sendRC5(lastCapturedCode, 12); break;
          case RC6: irsend.sendRC6(lastCapturedCode, 20); break;
          case PANASONIC: irsend.sendPanasonic64(lastCapturedCode); break;
          case JVC: irsend.sendJVC(lastCapturedCode, 16); break;
          case SAMSUNG: irsend.sendSAMSUNG(lastCapturedCode, 32); break;
          case LG: irsend.sendLG(lastCapturedCode, 28); break;
          case SHARP: irsend.sendSharpRaw(lastCapturedCode, 15); break;
          case RC5X: irsend.sendRC5(lastCapturedCode, 12); break;
          case DENON: irsend.sendDenon(lastCapturedCode, 14); break;
          case AIWA_RC_T501: irsend.sendAiwaRCT501(lastCapturedCode); break;
          case PIONEER: irsend.sendPioneer(lastCapturedCode, 64); break;
        }
        digitalWrite(DEBUG_LED, LOW);
        irrecv.enableIRIn();
#ifdef DEBUG
        Serial.println("IR receiver re-enabled after non-AC test");
#endif
#ifdef DEBUG
        Serial.println("SENT " + getProtocolName(lastCapturedProto) + " TEST");
#endif
      } else if (lastReceived.rawlen > 0 && lastReceived.rawlen <= MAX_RAW_LEN && validateRawTimings(lastReceived)) {
        lastCapturedProto = UNKNOWN;
        hasReceived = true;
        irrecv.disableIRIn();
        digitalWrite(DEBUG_LED, HIGH);
        irsend.sendRaw((uint16_t*)lastReceived.rawbuf, lastReceived.rawlen, strcmp(remotes[currentIndex].name, "TV") == 0 ? TV_RAW_FREQ : SPK_RAW_FREQ);
        digitalWrite(DEBUG_LED, LOW);
        irrecv.enableIRIn();
#ifdef DEBUG
        Serial.println("IR receiver re-enabled after raw test");
#endif
#ifdef DEBUG
        Serial.println("SENT RAW TEST");
#endif
      }
    }
    digitalWrite(DEBUG_LED, LOW);
    captureWaiting = false;
  }
}
