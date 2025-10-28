// Simple I2C Scanner for ESP32-S3
// Scans I2C bus with proper error handling
// Tests communication with known devices (accelerometer, RTC, etc.)

#include <Wire.h>

// I2C pins
#define I2C_SCL 10
#define I2C_SDA 11

// Known device addresses
#define MPU6050_ADDR 0x68    // Accelerometer (default address)
#define MPU6050_ALT_ADDR 0x69 // Accelerometer (alternate address)
#define PCF85063_ADDR 0x51   // RTC chip (from documentation)

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== I2C Scanner Test ===");
  Serial.println("Testing I2C communication on GPIO10 (SCL) and GPIO11 (SDA)\n");
  
  // Initialize I2C
  Serial.println("Initializing I2C...");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // 100kHz
  Serial.println("I2C initialized\n");
  
  // Test known devices
  Serial.println("=== Testing Known Devices ===\n");
  
  testDevice("MPU-6050 Accelerometer", MPU6050_ADDR);
  testDevice("MPU-6050 (Alt Address)", MPU6050_ALT_ADDR);
  testDevice("PCF85063 RTC", PCF85063_ADDR);
  
  // Full I2C scan
  Serial.println("\n=== Full I2C Bus Scan ===\n");
  scanI2CBus();
  
  Serial.println("\n=== Scan Complete ===");
}

void loop() {
  delay(10000);
}

// Test a specific I2C device
void testDevice(const char* name, uint8_t address) {
  Serial.printf("Testing %s at 0x%02X... ", name, address);
  
  Wire.beginTransmission(address);
  int error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("✓ FOUND");
    
    // Try to read first register
    Wire.beginTransmission(address);
    Wire.write(0x00);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Wire.requestFrom(address, 1);
      if (Wire.available()) {
        uint8_t value = Wire.read();
        Serial.printf("  Register 0x00: 0x%02X\n", value);
      }
    }
  } else {
    Serial.print("✗ Not found (error: ");
    switch(error) {
      case 1: Serial.println("data too long)"); break;
      case 2: Serial.println("NACK on address)"); break;
      case 3: Serial.println("NACK on data)"); break;
      case 4: Serial.println("other error)"); break;
      default: Serial.printf("%d)\n", error);
    }
  }
}

// Scan entire I2C bus
void scanI2CBus() {
  int deviceCount = 0;
  
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    int error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("Device found at 0x%02X (decimal %d)\n", address, address);
      deviceCount++;
      
      // Try to read register 0
      Wire.beginTransmission(address);
      Wire.write(0x00);
      Wire.endTransmission();
      
      Wire.requestFrom(address, 1);
      if (Wire.available()) {
        uint8_t value = Wire.read();
        Serial.printf("  Register 0x00: 0x%02X\n", value);
      }
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found!");
    Serial.println("\nPossible issues:");
    Serial.println("  1. I2C pins (GPIO10/11) may not be correct");
    Serial.println("  2. I2C bus may not be properly connected");
    Serial.println("  3. Devices may require different initialization");
  } else {
    Serial.printf("\nTotal devices found: %d\n", deviceCount);
  }
}
