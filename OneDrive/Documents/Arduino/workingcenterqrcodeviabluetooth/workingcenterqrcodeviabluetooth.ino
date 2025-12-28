#include "BLEDevice.h"

const char* targetName = "KPrinter_a65a_BLE";

BLEClient* pClient;
BLERemoteCharacteristic* pRemoteCharacteristic;

void setup() {
  Serial.begin(115200);
  BLEDevice::init("ESP32_Client");

  // Scan for printer
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

  // Connect
  pClient = BLEDevice::createClient();
  if (!pClient->connect(&targetDevice)) {
    Serial.println("Failed to connect!");
    return;
  }
  Serial.println("Connected!");

  // Find writable characteristic
  auto services = pClient->getServices();
  for (auto const& entry : *services) {
    BLERemoteService* service = entry.second;
    auto characteristics = service->getCharacteristics();
    for (auto const& charEntry : *characteristics) {
      BLERemoteCharacteristic* characteristic = charEntry.second;
      if (characteristic->canWrite()) {
        pRemoteCharacteristic = characteristic;

        // ESC/POS initialize
        uint8_t initCmd[] = {0x1B, 0x40};
        characteristic->writeValue(initCmd, sizeof(initCmd));

        // Center alignment
        uint8_t centerCmd[] = {0x1B, 0x61, 0x01}; // 0=left, 1=center, 2=right
        characteristic->writeValue(centerCmd, sizeof(centerCmd));

        // Generate unique 6-digit code
        int code = random(100000, 999999);
        char codeStr[7];
        sprintf(codeStr, "%06d", code);
        Serial.print("Generated code: ");
        Serial.println(codeStr);

        // --- ESC/POS QR sequence ---
        int len = strlen(codeStr);

        // Store QR data
        uint8_t storeCmd[] = {
          0x1D, 0x28, 0x6B,
          (len + 3) & 0xFF, ((len + 3) >> 8) & 0xFF,
          0x31, 0x50, 0x30
        };
        characteristic->writeValue(storeCmd, sizeof(storeCmd));
        characteristic->writeValue((uint8_t*)codeStr, len);

        // Set QR module size (try 6)
        uint8_t sizeCmd[] = {0x1D, 0x28, 0x6B, 0x03, 0x00, 0x31, 0x43, 0x06};
        characteristic->writeValue(sizeCmd, sizeof(sizeCmd));

        // Set error correction level (M)
        uint8_t eccCmd[] = {0x1D, 0x28, 0x6B, 0x03, 0x00, 0x31, 0x45, 0x31};
        characteristic->writeValue(eccCmd, sizeof(eccCmd));

        // Print QR
        uint8_t printCmd[] = {0x1D, 0x28, 0x6B, 0x03, 0x00, 0x31, 0x51, 0x30};
        characteristic->writeValue(printCmd, sizeof(printCmd));

        // Print label under QR (still centered)
        characteristic->writeValue("\nTicket: ");
        characteristic->writeValue(codeStr);
        characteristic->writeValue("\n\n");

        Serial.println("Centered QR + label sent!");
        return;
      }
    }
  }
}

void loop() {}
