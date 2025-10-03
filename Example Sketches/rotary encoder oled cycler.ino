#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ezButton.h>
#include <IRremote.h>

// ==== OLED ====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==== Rotary Encoder ====
#define CLK_PIN 25 // ESP32 pin GPIO25 (CLK)
#define DT_PIN 26  // ESP32 pin GPIO26 (DT)
#define SW_PIN 27  // ESP32 pin GPIO27 (SW)

// ==== Buttons and IR ====
#define BTN1_PIN 33 // Adjust if needed
#define BTN2_PIN 32 // Adjust if needed
#define IR_SEND_PIN 4 // IR LED pin

// ==== Remote Profiles ====
struct RemoteProfile {
  const char* name;
  uint16_t address;
  uint8_t command1;
  uint8_t command2;
};
RemoteProfile remotes[] = {
  {"TV", 0x80, 0x0A, 0x0B},  // Example IR codes
  {"AC", 0x10, 0x15, 0x20},
  {"SPK", 0x40, 0x22, 0x30} // Changed from STB
};
const int numRemotes = sizeof(remotes) / sizeof(remotes[0]);

int currentIndex = 0;
int CLK_state;
int prev_CLK_state;
ezButton button(SW_PIN);
ezButton btn1(BTN1_PIN);
ezButton btn2(BTN2_PIN);
bool isMenu = true;
IRsend irsend; // IRremote instance

void setup() {
  Serial.begin(115200);

  // OLED setup
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Encoder and button pins
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  button.setDebounceTime(50);
  btn1.setDebounceTime(50);
  btn2.setDebounceTime(50);

  // Read initial CLK state
  prev_CLK_state = digitalRead(CLK_PIN);

  // Enable IR sending
  irsend.begin(IR_SEND_PIN);

  Serial.println("Setup complete. Rotate M27 encoder to cycle options.");
  showMenu();
}

void loop() {
  button.loop();
  btn1.loop();
  btn2.loop();

  // Read current CLK state
  CLK_state = digitalRead(CLK_PIN);

  // Detect rising edge on CLK
  if (CLK_state != prev_CLK_state && CLK_state == HIGH) {
    if (digitalRead(DT_PIN) == HIGH) {
      currentIndex = (currentIndex - 1 + numRemotes) % numRemotes;
      Serial.print("Direction: Counter-clockwise, Index: ");
    } else {
      currentIndex = (currentIndex + 1) % numRemotes;
      Serial.print("Direction: Clockwise, Index: ");
    }
    Serial.print(currentIndex);
    Serial.print(", Remote: ");
    Serial.println(remotes[currentIndex].name);
    if (isMenu) {
      showMenu();
    }
  }
  prev_CLK_state = CLK_state;

  // Handle encoder switch press
  if (button.isPressed()) {
    if (isMenu) {
      isMenu = false;
      Serial.print("Selected: ");
      Serial.println(remotes[currentIndex].name);
      showSelected();
    } else {
      isMenu = true;
      Serial.println("Returning to menu");
      showMenu();
    }
  }

  // Handle button presses (only in selection mode)
  if (!isMenu) {
    if (btn1.isPressed()) {
      Serial.print("Sending command1 for ");
      Serial.println(remotes[currentIndex].name);
      irsend.sendNEC(remotes[currentIndex].address, remotes[currentIndex].command1, 0);
      showFeedback("Cmd1 Sent");
    }
    if (btn2.isPressed()) {
      Serial.print("Sending command2 for ");
      Serial.println(remotes[currentIndex].name);
      irsend.sendNEC(remotes[currentIndex].address, remotes[currentIndex].command2, 0);
      showFeedback("Cmd2 Sent");
    }
  }
}

void showMenu() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2); // Header
  display.println("Select:");
  display.setTextSize(4); // Remote name
  display.setCursor(0, 24);
  display.print("> ");
  display.println(remotes[currentIndex].name);
  display.display();
}

void showSelected() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2); // Header
  display.println("Selected:");
  
  // Prepare centered text with stars
  String text = String("* ") + remotes[currentIndex].name + " *";
  display.setTextSize(3); // Remote name
  display.setTextWrap(false);
  int textWidth = text.length() * 18; // 18 pixels per char at size 3
  int x = (128 - textWidth) / 2;
  if (x < 0) x = 0;
  display.setCursor(x, 24);
  display.println(text);
  display.display();
}

void showFeedback(const char* message) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println("Selected:");
  display.setTextSize(3);
  String text = String("* ") + remotes[currentIndex].name + " *";
  int textWidth = text.length() * 18;
  int x = (128 - textWidth) / 2;
  if (x < 0) x = 0;
  display.setCursor(x, 24);
  display.println(text);
  display.setTextSize(2);
  display.setCursor(0, 48);
  display.println(message);
  display.display();
  delay(1000); // Show feedback for 1 second
  showSelected(); // Return to selection screen
}