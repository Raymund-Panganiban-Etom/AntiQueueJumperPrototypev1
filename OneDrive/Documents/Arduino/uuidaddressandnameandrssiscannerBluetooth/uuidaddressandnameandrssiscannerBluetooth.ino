#include "BLEDevice.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE scan...");

  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan(); // create scanner
  pBLEScan->setActiveScan(true);            // active scan = more data
  BLEScanResults foundDevices = pBLEScan->start(10); // scan for 10 seconds

  for (int i = 0; i < foundDevices.getCount(); i++) {
    BLEAdvertisedDevice device = foundDevices.getDevice(i);
    Serial.print("Device: ");
    Serial.println(device.toString().c_str());
  }
}

void loop() {
  // nothing here
}
