#include "BLEDevice.h"

const char* targetName = "KPrinter_a65a_BLE";

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE scan...");

  BLEDevice::init("ESP32_Client");

  // Scan for devices
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  BLEScanResults results = pBLEScan->start(5);

  BLEAdvertisedDevice targetDevice;
  bool found = false;

  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);
    Serial.print("Found device: ");
    Serial.println(device.getName().c_str());

    if (device.getName() == targetName) {
      Serial.println("Target printer found!");
      targetDevice = device;
      found = true;
      break;
    }
  }

  if (!found) {
    Serial.println("Printer not found!");
    return;
  }

  // Connect to printer
  BLEClient* pClient = BLEDevice::createClient();
  Serial.println("Connecting...");
  if (!pClient->connect(&targetDevice)) {
    Serial.println("Failed to connect!");
    return;
  }
  Serial.println("Connected!");

  // Enumerate services
  std::map<std::string, BLERemoteService*>* services = pClient->getServices();
  for (auto const& entry : *services) {
    Serial.print("Service UUID: ");
    Serial.println(entry.first.c_str());

    BLERemoteService* service = entry.second;
    auto characteristics = service->getCharacteristics();
    for (auto const& charEntry : *characteristics) {
      Serial.print("  Characteristic UUID: ");
      Serial.println(charEntry.first.c_str());

      BLERemoteCharacteristic* characteristic = charEntry.second;
      if (characteristic->canWrite()) {
        Serial.println("  --> Writable characteristic found, sending test print...");

        // ESC/POS initialize
        uint8_t initCmd[] = {0x1B, 0x40};
        characteristic->writeValue(initCmd, sizeof(initCmd));

        // Print text
        characteristic->writeValue("Hello World\n");

        // Cut paper
        uint8_t cutCmd[] = {0x1D, 0x56, 0x00};
        characteristic->writeValue(cutCmd, sizeof(cutCmd));

        Serial.println("Print commands sent!");
      }
    }
  }
}

void loop() {
  // nothing here
}
