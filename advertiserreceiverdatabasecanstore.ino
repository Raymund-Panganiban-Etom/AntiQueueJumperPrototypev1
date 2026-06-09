#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "FS.h"
#include "SPIFFS.h"

#define DB_FILE "/people_db.txt"

// UUIDs
#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcdefab-1234-5678-1234-abcdefabcdef"

// Forward declarations
void insertRecord(String number, String name, String qr_text);
void readAllRecords();

// BLE characteristic pointer
BLECharacteristic* pCharacteristic;

// Server callbacks for connect/disconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("Client connected!");
  };

  void onDisconnect(BLEServer* pServer) {
    Serial.println("Client disconnected! Restarting advertising...");
    BLEDevice::startAdvertising();  // Restart advertising to allow reconnections
  }
};

// Characteristic callbacks for writes
class DataCallback: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      Serial.print("Received via BLE: ");
      Serial.println(rxValue.c_str());

      // Parse string: expected format "number|name|qr"
      String data = String(rxValue.c_str());
      int firstSep = data.indexOf('|');
      int secondSep = data.indexOf('|', firstSep + 1);

      if (firstSep > 0 && secondSep > firstSep) {
        String number = data.substring(0, firstSep);
        String name   = data.substring(firstSep + 1, secondSep);
        String qr     = data.substring(secondSep + 1);

        insertRecord(number, name, qr);
      } else {
        Serial.println("Invalid format. Expected: number|name|qr");
      }
    }
  }
};

void setup() {
  Serial.begin(115200);  // Lower baud for reliability
  delay(1000);
  Serial.println("Starting ESP32-S3 BLE + SPIFFS...");

  // Mount SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed!");
    return;
  }

  // Init BLE
  BLEDevice::init("ESP32-S3 Peripheral");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Characteristic for receiving data
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());  // For notify support
  pCharacteristic->setValue("Send data as number|name|qr");
  pCharacteristic->setCallbacks(new DataCallback());

  pService->start();

  // Advertising setup
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // Helps with iPhone connections
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("ESP32-S3 is now advertising!");
}

void loop() {
  Serial.println("=== Database Records ===");
  readAllRecords();
  delay(5000);
}

// Insert record into SPIFFS
void insertRecord(String number, String name, String qr_text) {
  File db = SPIFFS.open(DB_FILE, FILE_APPEND);
  if (!db) {
    Serial.println("Failed to open DB file for writing");
    return;
  }
  db.println(number + "|" + name + "|" + qr_text);
  db.close();

  Serial.println("Inserted: " + number + " | " + name + " | " + qr_text);
}

// Read all records from SPIFFS
void readAllRecords() {
  File db = SPIFFS.open(DB_FILE, FILE_READ);
  if (!db) {
    Serial.println("Failed to open DB file for reading");
    return;
  }
  while (db.available()) {
    String line = db.readStringUntil('\n');
    Serial.println(line);
  }
  db.close();
}