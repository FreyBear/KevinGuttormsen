// Serial Debug Test - Minimal test to identify where the hang occurs

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== Serial Debug Test ===");
  Serial.println("Step 1: Serial initialized");
  Serial.flush();
  delay(500);
  
  Serial.println("Step 2: About to initialize I2C with default pins...");
  Serial.flush();
  delay(500);
  
  // Try I2C with default pins (no custom pins)
  Wire.begin();
  
  Serial.println("Step 3: I2C initialized with default pins");
  Serial.flush();
  delay(500);
  
  Serial.println("Step 4: Testing I2C communication...");
  Serial.flush();
  delay(500);
  
  // Try to communicate with a device
  Wire.beginTransmission(0x68);  // MPU6050 default address
  int error = Wire.endTransmission();
  
  Serial.printf("Step 5: I2C transmission result: %d\n", error);
  Serial.flush();
  delay(500);
  
  Serial.println("Step 6: Setup complete!");
  Serial.flush();
}

void loop() {
  Serial.println("Loop running...");
  delay(5000);
}
