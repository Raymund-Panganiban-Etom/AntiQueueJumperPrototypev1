#include <TFT_eSPI.h>
#include <SPI.h>
#include "BLEDevice.h"

#define ctsPin 2

// Existing buzzer (key, toggle, etc.)
const int buzzerPin = 15;
const int buzzerChannel = 0;

// New ultrasonic pins
const int trigPin = 14;
const int echoPin = 13;

// New second buzzer for distance alert
const int distBuzzerPin = 12;
const int distBuzzerChannel = 1;

// Ultrasonic / distance logic
float currentDistanceCm = 0.0;
float lastDisplayedDistanceCm = -1.0;  // to track when to redraw text
unsigned long lastUltrasonicRead = 0;
const unsigned long ultrasonicInterval = 100; // 10 readings per second

// Buzzer logic for distance alert
bool alertActive = false;
unsigned long lastAlertBeep = 0;
const unsigned long alertBeepInterval = 500; // 1 sec between beeps

TFT_eSPI tft = TFT_eSPI();

// Calibration values
uint16_t calData[5] = {525, 3297, 425, 3215, 3};
#define TOUCH_THRESHOLD 70

// Keyboard layout
const char keys[3][10] = {
  {'Q','W','E','R','T','Y','U','I','O','P'},
  {'A','S','D','F','G','H','J','K','L','.'},
  {'Z','X','C','V','B','N','M',' ','?','!'}
};

String chatText = "";
bool keyboardVisible = false;
bool lastSensorState = LOW;

// BLE printer
const char* targetName = "KPrinter_a65a_BLE";
BLEClient* pClient;
BLERemoteCharacteristic* pRemoteCharacteristic;

// Counter for tickets
int ticketCounter = 1;

// Forward declaration
void drawKeyPressFeedback(uint8_t row, uint8_t col);
void drawChatBox();      // so we can call it before definition
void updateDistanceDisplay(float dist); // new lightweight redraw

// --- Beep helper for main buzzer ---
void beep(int freq, int duration) {
  ledcWriteTone(buzzerChannel, freq);
  delay(duration);
  ledcWriteTone(buzzerChannel, 0);
}

// --- Beep helper for distance buzzer (non-blocking style call) ---
void beepDistanceBuzzer(int freq, int duration) {
  // short blocking beep is okay since it's rare and short (100 ms)
  ledcWriteTone(distBuzzerChannel, freq);
  delay(duration);
  ledcWriteTone(distBuzzerChannel, 0);
}

// --- Full-screen status helper ---
void showFullStatus(const char* msg, uint16_t color = TFT_WHITE) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.setCursor(40, tft.height()/2 - 10);
  tft.print(msg);
}

// --- Ultrasonic distance measurement ---
float readUltrasonicCm() {
  // Ensure a clean LOW pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // 10us HIGH pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo pulse
  long duration = pulseIn(echoPin, HIGH, 25000); // 25ms timeout (~4m range)

  if (duration == 0) {
    // Timeout / no echo
    return -1.0;
  }

  // Distance in cm: speed of sound ~34300 cm/s
  float distance = duration * 0.0343 / 2.0;
  return distance;
}

// --- BLE send QR ---
void sendQR(String label) {
  if (!pRemoteCharacteristic) {
    Serial.println("No printer characteristic!");
    showFullStatus("Printer not ready!", TFT_RED);
    return;
  }

  // ESC/POS initialize
  uint8_t initCmd[] = {0x1B, 0x40};
  pRemoteCharacteristic->writeValue(initCmd, sizeof(initCmd));

  // Center alignment
  uint8_t centerCmd[] = {0x1B, 0x61, 0x01};
  pRemoteCharacteristic->writeValue(centerCmd, sizeof(centerCmd));

  // Build QR data: chatText + counter
  String qrData = label + "_" + String(ticketCounter++);
  int len = qrData.length();

  // Store QR data
  uint8_t storeCmd[] = {
    0x1D, 0x28, 0x6B,
    (uint8_t)((len + 3) & 0xFF), (uint8_t)(((len + 3) >> 8) & 0xFF),
    0x31, 0x50, 0x30
  };
  pRemoteCharacteristic->writeValue(storeCmd, sizeof(storeCmd));
  pRemoteCharacteristic->writeValue((uint8_t*)qrData.c_str(), len);

  // Set QR module size
  uint8_t sizeCmd[] = {0x1D,0x28,0x6B,0x03,0x00,0x31,0x43,0x06};
  pRemoteCharacteristic->writeValue(sizeCmd, sizeof(sizeCmd));

  // Set error correction level
  uint8_t eccCmd[] = {0x1D,0x28,0x6B,0x03,0x00,0x31,0x45,0x31};
  pRemoteCharacteristic->writeValue(eccCmd, sizeof(eccCmd));

  // Print QR
  uint8_t printCmd[] = {0x1D,0x28,0x6B,0x03,0x00,0x31,0x51,0x30};
  pRemoteCharacteristic->writeValue(printCmd, sizeof(printCmd));

  // Print label under QR
  pRemoteCharacteristic->writeValue("\nTicket: ");
  pRemoteCharacteristic->writeValue(qrData.c_str());
  pRemoteCharacteristic->writeValue("\n\n");

  Serial.println("QR sent: " + qrData);
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  pinMode(ctsPin, INPUT);

  // Buzzer channels
  ledcAttachPin(buzzerPin, buzzerChannel);
  ledcAttachPin(distBuzzerPin, distBuzzerChannel);

  // Ultrasonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  tft.init();
  tft.setRotation(3);
  tft.setTouch(calData);
  tft.fillScreen(TFT_BLACK);

  // Initial UI
  drawChatBox();

  // BLE init
  BLEDevice::init("ESP32_Client");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  BLEScanResults results = pBLEScan->start(5);

  BLEAdvertisedDevice targetDevice;
  bool found = false;
  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);
    if (device.getName() == targetName) {
      targetDevice = device;
      found = true;
      break;
    }
  }
  if (!found) {
    Serial.println("Printer not found!");
    showFullStatus("Printer not found!", TFT_RED);
    return;
  }

  pClient = BLEDevice::createClient();
  showFullStatus("Connecting...", TFT_YELLOW);

  if (!pClient->connect(&targetDevice)) {
    Serial.println("Failed to connect!");
    showFullStatus("Connect failed!", TFT_RED);
    return;
  }
  Serial.println("Connected!");
  showFullStatus("Connected!", TFT_GREEN);

  auto services = pClient->getServices();
  for (auto const& entry : *services) {
    BLERemoteService* service = entry.second;
    auto characteristics = service->getCharacteristics();
    for (auto const& charEntry : *characteristics) {
      BLERemoteCharacteristic* characteristic = charEntry.second;
      if (characteristic->canWrite()) {
        pRemoteCharacteristic = characteristic;
        Serial.println("Printer characteristic ready!");
        showFullStatus("Printer ready!", TFT_GREEN);
        return;
      }
    }
  }
}

void loop() {
  unsigned long now = millis();

  // --- Handle touch sensor toggle for keyboard ---
  int ctsValue = digitalRead(ctsPin);

  if (ctsValue == HIGH && lastSensorState == LOW) {
    keyboardVisible = !keyboardVisible;
    if (keyboardVisible) {
      drawKeyboard();
      drawChatBox(); // includes distance area
      beep(1200, 100); // ON tone
    } else {
      tft.fillScreen(TFT_BLACK);
      beep(800, 100);  // OFF tone
    }
  }
  lastSensorState = ctsValue;

  // --- Ultrasonic reading every 100 ms ---
  if (now - lastUltrasonicRead >= ultrasonicInterval) {
    lastUltrasonicRead = now;
    float d = readUltrasonicCm();
    if (d > 0) {
      currentDistanceCm = d;
      // Update distance display only if changed by at least 1 cm
      if (fabs(currentDistanceCm - lastDisplayedDistanceCm) >= 1.0) {
        updateDistanceDisplay(currentDistanceCm);
        lastDisplayedDistanceCm = currentDistanceCm;
      }
    }

    // Distance-based alert logic
    if (currentDistanceCm > 0 && currentDistanceCm < 30.0) {
      alertActive = true;
    } else if (currentDistanceCm >= 31.0) {
      alertActive = false;
    }
  }

  // --- Handle alert buzzer 1 beep per second when active ---
  if (alertActive && (now - lastAlertBeep >= alertBeepInterval)) {
    lastAlertBeep = now;
    // Short beep: Option 1
    beepDistanceBuzzer(3000, 100);
  }

  // --- Handle touch for keyboard only when visible ---
  if (keyboardVisible) {
    uint16_t x, y;
    if (tft.getTouch(&x, &y)) {
      if (tft.getTouchRawZ() < TOUCH_THRESHOLD) return;
      x = tft.width() - x;
      y = tft.height() - y;
      handleTouch(x, y);
      delay(50);
    }
  }
}

// --- Keyboard functions ---
void handleTouch(int16_t x, int16_t y) {
  // Keys area
  if (y >= 100) {
    uint8_t row = (y - 100) / 45;
    uint8_t col = x / 32;

    if (row < 3 && col < 10) {
      char k = keys[row][col];
      chatText += (k == ' ') ? ' ' : k;
      drawChatBox();
      drawKeyPressFeedback(row, col);
      beep(2000, 80); // key tone
    }
  }

  // Clear button (top-right)
  if (x >= 260 && y <= 60) {
    chatText = "";
    drawChatBox();
    beep(1500, 100); // clear tone
  }

  // ENTER button (bottom-right)
  if (x >= 260 && y >= 220) {
    beep(1000, 150); // enter tone

    // Sequence of notifications
    showFullStatus("Generating QR code...", TFT_YELLOW);
    delay(800);

    showFullStatus("Connecting to printer...", TFT_CYAN);
    delay(800);

    if (pRemoteCharacteristic) {
      showFullStatus("Connected!", TFT_GREEN);
      delay(500);

      showFullStatus("Printing...", TFT_GREEN);
      sendQR(chatText);
      delay(1000);
    } else {
      showFullStatus("Printer not ready!", TFT_RED);
      delay(1500);
    }

    // Return to keyboard/chat UI
    drawKeyboard();
    drawChatBox();
  }
}

void drawKeyboard() {
  tft.fillRect(0, 100, 320, 135, TFT_DARKGREY);
  for (uint8_t r = 0; r < 3; r++) {
    for (uint8_t c = 0; c < 10; c++) {
      int16_t bx = c * 32 + 2;
      int16_t by = 100 + r * 45 + 2;
      tft.drawRect(bx, by, 28, 40, TFT_WHITE);
      tft.setCursor(bx + 8, by + 12);
      tft.setTextColor(TFT_YELLOW);
      tft.setTextSize(2);
      tft.print(keys[r][c]);
    }
  }

  // ENTER button
  tft.fillRoundRect(260, 220, 50, 30, 6, TFT_GREEN);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(270, 228);
  tft.setTextSize(2);
  tft.print("ENT");
}

// Top bar with "Enter Name" + distance + CLR button
void drawChatBox() {
  // Background bar
  tft.fillRect(0, 0, 320, 100, TFT_BLACK);

  // "Enter Name"
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(10, 10);
  tft.print("Enter Name");

  // Distance initial draw (will be updated by updateDistanceDisplay as well)
  updateDistanceDisplay(currentDistanceCm);

  // Clear button
  tft.fillRoundRect(260, 10, 50, 30, 6, TFT_RED);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(272, 18);
  tft.print("CLR");

  // Text area
  tft.setTextColor(TFT_GREEN);
  tft.setCursor(10, 50);
  tft.fillRect(10, 50, 300, 24, TFT_BLACK);
  tft.print(chatText);

  // Cursor line if space available
  int px = 10 + chatText.length() * 12;
  if (px < 310) {
    tft.drawFastVLine(px, 55, 20, TFT_GREEN);
  }
}

// Lightweight distance redraw beside "Enter Name"
void updateDistanceDisplay(float dist) {
  // Draw in a fixed zone to the right of "Enter Name"
  // "Enter Name" starts at x=10, width ~ 11 chars * 12px ≈ 132px
  // So we start around x=160
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW);

  // Clear old distance area only
  tft.fillRect(160, 10, 90, 24, TFT_BLACK);

  tft.setCursor(160, 10);
  if (dist <= 0) {
    tft.print("-- cm");
  } else {
    tft.print((int)dist);
    tft.print(" cm");
  }
}

void drawKeyPressFeedback(uint8_t row, uint8_t col) {
  int16_t x = col * 32 + 2;
  int16_t y = 100 + row * 45 + 2;
  tft.fillRoundRect(x, y, 28, 40, 4, TFT_YELLOW);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(x + 8, y + 12);
  tft.setTextSize(2);
  tft.print(keys[row][col]);
  delay(100);
  tft.fillRoundRect(x, y, 28, 40, 4, TFT_DARKGREY);
  tft.drawRect(x, y, 28, 40, TFT_WHITE);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(x + 8, y + 12);
  tft.print(keys[row][col]);
}
