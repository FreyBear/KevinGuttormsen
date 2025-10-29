// I2C Scanner for ESP32-S3 Board
// Scans for all I2C devices and displays their addresses

#include <Wire.h>

#define I2C_SDA 11
#define I2C_SCL 10

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== I2C Scanner ===");
  Wire.begin(I2C_SDA, I2C_SCL);
  
  Serial.println("Scanning I2C bus...");
  scanI2C();
}

void loop() {
  delay(5000);
  Serial.println("\nScanning again...");
  scanI2C();
}

void scanI2C() {
  byte error, address;
  int devices = 0;
  
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      devices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if (devices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("Scan complete\n");
}
