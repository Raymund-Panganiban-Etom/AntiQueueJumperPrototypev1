#include <TFT_eSPI.h>
#include <SPI.h>
#include "BLEDevice.h"

#define ctsPin 2
const int buzzerPin = 15;
const int buzzerChannel = 0;

TFT_eSPI tft = TFT_eSPI();

// Calibration values
uint16_t calData[5] = {525, 3297, 425, 3215, 3};
#define TOUCH_THRESHOLD 20

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

// Forward declaration so handleTouch() can call it
void drawKeyPressFeedback(uint8_t row, uint8_t col);

// --- Beep helper ---
void beep(int freq, int duration) {
  ledcWriteTone(buzzerChannel, freq);
  delay(duration);
  ledcWriteTone(buzzerChannel, 0);
}

// --- BLE send QR ---
void sendQR(String label) {
  if (!pRemoteCharacteristic) {
    Serial.println("No printer characteristic!");
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
  ledcAttachPin(buzzerPin, buzzerChannel);

  tft.init();
  tft.setRotation(3);
  tft.setTouch(calData);
  tft.fillScreen(TFT_BLACK);

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
    return;
  }

  pClient = BLEDevice::createClient();
  if (!pClient->connect(&targetDevice)) {
    Serial.println("Failed to connect!");
    return;
  }
  Serial.println("Connected!");

  auto services = pClient->getServices();
  for (auto const& entry : *services) {
    BLERemoteService* service = entry.second;
    auto characteristics = service->getCharacteristics();
    for (auto const& charEntry : *characteristics) {
      BLERemoteCharacteristic* characteristic = charEntry.second;
      if (characteristic->canWrite()) {
        pRemoteCharacteristic = characteristic;
        Serial.println("Printer characteristic ready!");
        return;
      }
    }
  }
}

void loop() {
  int ctsValue = digitalRead(ctsPin);

  if (ctsValue == HIGH && lastSensorState == LOW) {
    keyboardVisible = !keyboardVisible;
    if (keyboardVisible) {
      drawKeyboard();
      drawChatBox();
      beep(1200, 100); // ON tone
    } else {
      tft.fillScreen(TFT_BLACK);
      beep(800, 100);  // OFF tone
    }
  }
  lastSensorState = ctsValue;

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
    sendQR(chatText);
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

void drawChatBox() {
  tft.fillRect(0, 0, 320, 100, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(10, 10);
  tft.print("Enter Name");

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

  // Cursor line if space
  int px = 10 + chatText.length() * 12;
  if (px < 310) {
    tft.drawFastVLine(px, 55, 20, TFT_GREEN);
  }
}

// --- Key press visual feedback ---
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
