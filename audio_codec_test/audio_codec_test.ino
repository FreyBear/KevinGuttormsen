// Audio Codec Detection and Test for ESP32-S3
// Attempts to detect and initialize the built-in audio codec
// Scans I2C bus for common audio codec chips

#include <Wire.h>

// I2C pins for audio codec
#define I2C_SCL 10
#define I2C_SDA 11

// Common audio codec I2C addresses
#define ES8388_ADDR 0x10    // ES8388 codec
#define ES8311_ADDR 0x18    // ES8311 codec
#define AC101_ADDR 0x15     // AC101 codec
#define WM8960_ADDR 0x1A    // WM8960 codec

// Codec detection results
struct CodecInfo {
  const char* name;
  uint8_t address;
  bool found;
};

CodecInfo codecs[] = {
  {"ES8388", ES8388_ADDR, false},
  {"ES8311", ES8311_ADDR, false},
  {"AC101", AC101_ADDR, false},
  {"WM8960", WM8960_ADDR, false}
};
const int numCodecs = sizeof(codecs) / sizeof(codecs[0]);

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== Audio Codec Detection Test ===");
  Serial.println("Scanning I2C bus for audio codec chips...\n");
  
  // Initialize I2C with custom pins
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // 100kHz for codec communication
  
  Serial.println("I2C initialized on GPIO10 (SCL) and GPIO11 (SDA)");
  Serial.println("Scanning for audio codecs...\n");
  
  // Scan for known codecs
  scanForCodecs();
  
  // Print results
  Serial.println("\n=== Scan Results ===");
  bool foundCodec = false;
  for (int i = 0; i < numCodecs; i++) {
    if (codecs[i].found) {
      Serial.printf("✓ Found: %s at address 0x%02X\n", codecs[i].name, codecs[i].address);
      foundCodec = true;
    } else {
      Serial.printf("✗ Not found: %s (0x%02X)\n", codecs[i].name, codecs[i].address);
    }
  }
  
  if (!foundCodec) {
    Serial.println("\n⚠ No known audio codec found!");
    Serial.println("Possible reasons:");
    Serial.println("  1. Codec is on a different I2C address");
    Serial.println("  2. Codec requires different initialization");
    Serial.println("  3. Audio codec may be integrated differently on this board");
  }
  
  // Scan entire I2C bus for any devices
  Serial.println("\n=== Full I2C Bus Scan ===");
  Serial.println("Scanning all I2C addresses for any devices...\n");
  scanI2CBus();
  
  Serial.println("\n=== Test Complete ===");
  Serial.println("Check the results above to identify the audio codec");
}

void loop() {
  delay(10000);  // Just wait, test is in setup
}

// Scan for known audio codecs
void scanForCodecs() {
  for (int i = 0; i < numCodecs; i++) {
    Wire.beginTransmission(codecs[i].address);
    int result = Wire.endTransmission();
    
    if (result == 0) {
      codecs[i].found = true;
      Serial.printf("Codec detected: %s at 0x%02X\n", codecs[i].name, codecs[i].address);
      
      // Try to read a register to confirm
      Wire.beginTransmission(codecs[i].address);
      Wire.write(0x00);  // Read from register 0
      Wire.endTransmission();
      
      Wire.requestFrom(codecs[i].address, 1);
      if (Wire.available()) {
        uint8_t value = Wire.read();
        Serial.printf("  Register 0x00 value: 0x%02X\n", value);
      }
    }
  }
}

// Scan entire I2C bus
void scanI2CBus() {
  int deviceCount = 0;
  
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    int result = Wire.endTransmission();
    
    if (result == 0) {
      Serial.printf("Device found at address 0x%02X (decimal %d)\n", address, address);
      deviceCount++;
      
      // Try to identify the device
      identifyDevice(address);
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found on the bus!");
  } else {
    Serial.printf("\nTotal devices found: %d\n", deviceCount);
  }
}

// Try to identify I2C device by reading common registers
void identifyDevice(uint8_t address) {
  // Try to read chip ID registers (common locations)
  uint8_t chipID = 0;
  bool foundID = false;
  
  // Try common chip ID register addresses
  uint8_t idRegisters[] = {0x00, 0x01, 0x02, 0x03, 0xFE, 0xFF};
  
  for (int i = 0; i < sizeof(idRegisters); i++) {
    Wire.beginTransmission(address);
    Wire.write(idRegisters[i]);
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(address, 1);
      if (Wire.available()) {
        chipID = Wire.read();
        Serial.printf("  Register 0x%02X: 0x%02X\n", idRegisters[i], chipID);
        foundID = true;
      }
    }
  }
}
