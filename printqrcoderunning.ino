#include "BLEDevice.h"
#include "qrcodegen.hpp"  // Nayuki QR Code Generator (patched with LOW_ECC etc.)

using namespace qrcodegen;

const char* targetName = "KPrinter_a65a_BLE";

BLEClient* pClient;
BLERemoteCharacteristic* pRemoteCharacteristic;

void sendRasterImage(BLERemoteCharacteristic* characteristic, uint8_t* data, int width, int height) {
  int widthBytes = (width + 7) / 8;
  uint8_t xL = widthBytes & 0xFF;
  uint8_t xH = (widthBytes >> 8) & 0xFF;
  uint8_t yL = height & 0xFF;
  uint8_t yH = (height >> 8) & 0xFF;

  uint8_t header[] = {0x1D, 0x76, 0x30, 0x00, xL, xH, yL, yH};
  characteristic->writeValue(header, sizeof(header));
  characteristic->writeValue(data, widthBytes * height);
}

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
        uint8_t centerCmd[] = {0x1B, 0x61, 0x01};
        characteristic->writeValue(centerCmd, sizeof(centerCmd));

        // Generate unique 6-digit code
        int code = random(100000, 999999);
        char codeStr[7];
        sprintf(codeStr, "%06d", code);
        Serial.print("Generated code: ");
        Serial.println(codeStr);

        // Generate QR with Nayuki (patched enum)
        QrCode qr = QrCode::encodeText(codeStr, QrCode::Ecc::MEDIUM_ECC);

        // Scale QR to fit 58mm paper
        int scale = 5; // adjust 4–6 for size
        int qrSize = qr.getSize() * scale;
        int widthBytes = (qrSize + 7) / 8;
        int height = qrSize;
        std::vector<uint8_t> bitmap(widthBytes * height, 0);

        for (int y = 0; y < qr.getSize(); y++) {
          for (int x = 0; x < qr.getSize(); x++) {
            if (qr.getModule(x, y)) {
              for (int dy = 0; dy < scale; dy++) {
                for (int dx = 0; dx < scale; dx++) {
                  int px = x * scale + dx;
                  int py = y * scale + dy;
                  int byteIndex = (py * widthBytes) + (px >> 3);
                  bitmap[byteIndex] |= (0x80 >> (px & 7));
                }
              }
            }
          }
        }

        // Send raster QR
        sendRasterImage(characteristic, bitmap.data(), qrSize, qrSize);

        // Print label
        characteristic->writeValue("\nTicket: ");
        characteristic->writeValue(codeStr);
        characteristic->writeValue("\n\n");

        Serial.println("QR bitmap + label sent!");
        return;
      }
    }
  }
}

void loop() {}
