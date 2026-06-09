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
const char* targetPrinterName = "KPrinter_a65a_BLE";
BLEClient* pPrinterClient;
BLERemoteCharacteristic* pPrinterCharacteristic;

// BLE advertiser (ESP32-S3)
const char* targetAdvertiserName = "ESP32-S3 Peripheral";
const BLEUUID advertiserServiceUUID("12345678-1234-1234-1234-1234567890ab");
const BLEUUID advertiserCharUUID("abcdefab-1234-5678-1234-abcdefabcdef");

// Stored devices from scan
BLEAdvertisedDevice printerDevice;
BLEAdvertisedDevice advertiserDevice;
bool printerFound = false;
bool advertiserFound = false;

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

// --- BLE send QR to printer ---
void sendQRToPrinter(String label) {
  if (!pPrinterCharacteristic) {
    Serial.println("No printer characteristic!");
    showFullStatus("Printer not ready!", TFT_RED);
    return;
  }

  // Capture current ticket before increment
  int currentTicket = ticketCounter;
  String qrData = label + "_" + String(currentTicket);
  ticketCounter++;  // Increment after using

  // ESC/POS initialize
  uint8_t initCmd[] = {0x1B, 0x40};
  pPrinterCharacteristic->writeValue(initCmd, sizeof(initCmd));

  // Center alignment
  uint8_t centerCmd[] = {0x1B, 0x61, 0x01};
  pPrinterCharacteristic->writeValue(centerCmd, sizeof(centerCmd));

  // Build QR data: already in qrData
  int len = qrData.length();

  // Store QR data
  uint8_t storeCmd[] = {
    0x1D, 0x28, 0x6B,
    (uint8_t)((len + 3) & 0xFF), (uint8_t)(((len + 3) >> 8) & 0xFF),
    0x31, 0x50, 0x30
  };
  pPrinterCharacteristic->writeValue(storeCmd, sizeof(storeCmd));
  pPrinterCharacteristic->writeValue((uint8_t*)qrData.c_str(), len);

  // Set QR module size
  uint8_t sizeCmd[] = {0x1D,0x28,0x6B,0x03,0x00,0x31,0x43,0x06};
  pPrinterCharacteristic->writeValue(sizeCmd, sizeof(sizeCmd));

  // Set error correction level
  uint8_t eccCmd[] = {0x1D,0x28,0x6B,0x03,0x00,0x31,0x45,0x31};
  pPrinterCharacteristic->writeValue(eccCmd, sizeof(eccCmd));

  // Print QR
  uint8_t printCmd[] = {0x1D,0x28,0x6B,0x03,0x00,0x31,0x51,0x30};
  pPrinterCharacteristic->writeValue(printCmd, sizeof(printCmd));

  // Print label under QR
  pPrinterCharacteristic->writeValue("\nTicket: ");
  pPrinterCharacteristic->writeValue(qrData.c_str());
  pPrinterCharacteristic->writeValue("\n\n");

  Serial.println("QR sent to printer: " + qrData);
}

// --- Send data to advertiser (ESP32-S3) ---
// --- Send data to advertiser (ESP32-S3) ---
void sendDataToAdvertiser(String name, int currentTicket, String qrData) {
  BLEClient* advClient = BLEDevice::createClient();
  showFullStatus("Connecting to ESP32-S3...", TFT_YELLOW);

  if (!advClient->connect(&advertiserDevice)) {
    Serial.println("Failed to connect to advertiser!");
    showFullStatus("Connect to ESP32-S3 failed!", TFT_RED);
    delay(1500);
    delete advClient;
    return;
  }
  Serial.println("Connected to advertiser!");
  showFullStatus("Connected to ESP32-S3!", TFT_GREEN);
  delay(500);

  BLERemoteService* advService = advClient->getService(advertiserServiceUUID);
  if (!advService) {
    Serial.println("Advertiser service not found!");
    showFullStatus("ESP32-S3 service not found!", TFT_RED);
    delay(1500);
    advClient->disconnect();
    while (advClient->isConnected()) {
      delay(100);
    }
    delete advClient;
    return;
  }

  BLERemoteCharacteristic* advChar = advService->getCharacteristic(advertiserCharUUID);
  if (!advChar || !advChar->canWrite()) {
    Serial.println("Advertiser characteristic not found or not writable!");
    showFullStatus("ESP32-S3 char not found!", TFT_RED);
    delay(1500);
    advClient->disconnect();
    while (advClient->isConnected()) {
      delay(100);
    }
    delete advClient;
    return;
  }

  // Format data: number|name|qr
  String dataToSend = String(currentTicket) + "|" + name + "|" + qrData;

  // Correct overload: uint8_t* + length + response flag
  // This matches the library's signature: writeValue(uint8_t* data, size_t length, bool response = false)
  advChar->writeValue((uint8_t*)dataToSend.c_str(), dataToSend.length(), true);  // true = write with response

  Serial.println("Data sent to advertiser (with response): " + dataToSend);
  showFullStatus("Data sent to ESP32-S3!", TFT_GREEN);
  delay(1000);

  // Disconnect cleanly
  Serial.println("Disconnecting from advertiser...");
  advClient->disconnect();
  delay(500);  // Allow disconnect to process

  // Wait for full disconnect before deleting (prevents heap corruption/reset)
  unsigned long startWait = millis();
  while (advClient->isConnected() && (millis() - startWait < 5000)) {
    delay(100);
  }

  if (advClient->isConnected()) {
    Serial.println("Warning: Timed out waiting for disconnect!");
  } else {
    Serial.println("Disconnected cleanly.");
  }

  delete advClient;
  Serial.println("Advertiser client deleted.");
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
  BLEScanResults results = pBLEScan->start(10);  // Scan for 10 seconds to find both

  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);
    String devName = String(device.getName().c_str());
    if (devName == targetPrinterName) {
      printerDevice = device;
      printerFound = true;
      Serial.println("Printer found!");
    } else if (devName == targetAdvertiserName) {
      advertiserDevice = device;
      advertiserFound = true;
      Serial.println("Advertiser found!");
    }
  }

  // Show scan results
  if (printerFound && advertiserFound) {
    showFullStatus("Printer and Advertiser found!", TFT_GREEN);
    delay(1000);
  } else if (printerFound) {
    showFullStatus("Printer found, Advertiser not found!", TFT_YELLOW);
    delay(1500);
  } else if (advertiserFound) {
    showFullStatus("Advertiser found, Printer not found!", TFT_YELLOW);
    delay(1500);
  } else {
    showFullStatus("No devices found!", TFT_RED);
    delay(1500);
    return;  // Halt if nothing found, but could proceed
  }

  // Connect to printer if found
  if (printerFound) {
    pPrinterClient = BLEDevice::createClient();
    showFullStatus("Connecting to printer...", TFT_YELLOW);

    if (!pPrinterClient->connect(&printerDevice)) {
      Serial.println("Failed to connect to printer!");
      showFullStatus("Printer connect failed!", TFT_RED);
      return;
    }
    Serial.println("Connected to printer!");
    showFullStatus("Printer connected!", TFT_GREEN);
    delay(500);

    auto services = pPrinterClient->getServices();
    for (auto const& entry : *services) {
      BLERemoteService* service = entry.second;
      auto characteristics = service->getCharacteristics();
      for (auto const& charEntry : *characteristics) {
        BLERemoteCharacteristic* characteristic = charEntry.second;
        if (characteristic->canWrite()) {
          pPrinterCharacteristic = characteristic;
          Serial.println("Printer characteristic ready!");
          showFullStatus("Printer ready!", TFT_GREEN);
          delay(500);
          break;
        }
      }
    }
  } else {
    showFullStatus("Printer not found!", TFT_RED);
    return;
  }

  // Advertiser connection deferred to later
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

    if (pPrinterCharacteristic) {
      showFullStatus("Connected!", TFT_GREEN);
      delay(500);

      showFullStatus("Printing...", TFT_GREEN);
      sendQRToPrinter(chatText);
      delay(1000);

      showFullStatus("Printing finished!", TFT_GREEN);
      delay(800);
    } else {
      showFullStatus("Printer not ready!", TFT_RED);
      delay(1500);
    }

    // Now handle advertiser if found
    if (advertiserFound) {
      // Capture data for advertiser
      int currentTicket = ticketCounter - 1;  // Since incremented in sendQRToPrinter
      String qrData = chatText + "_" + String(currentTicket);

      sendDataToAdvertiser(chatText, currentTicket, qrData);
    } else {
      showFullStatus("No advertiser, skipping...", TFT_YELLOW);
      delay(1000);
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