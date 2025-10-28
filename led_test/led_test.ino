// LED Pin Test for DeepSeek ESP32-S3 Board
// This program tests common GPIO pins to find the RGB LED ring

#include <Adafruit_NeoPixel.h>

// Extended list of possible GPIO pins for WS2812 LEDs on ESP32-S3
// Excluding pins already used for I2S (12-16), I2C (10-11), SD card (40-42)
// GPIO18 caused crash - skipping problematic pins
const int testPins[] = {
  48, 47, 21, 38, 39, 45, 46,  // Most common
  1, 2, 8, 9, 17,              // Available GPIOs (skipped 18)
  19, 20, 35, 36, 37,          // Additional GPIOs
  3, 4, 5, 6, 7                // GPIO 3-7 (for buttons but worth testing)
};
const int numTestPins = sizeof(testPins) / sizeof(testPins[0]);
const int NUM_LEDS = 7;  // 7 RGB LEDs in a ring according to docs

Adafruit_NeoPixel* strip = NULL;
int currentPinIndex = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== RGB LED Pin Test ===");
  Serial.println("This program will test different GPIO pins to find the LEDs");
  Serial.println("Watch the LED ring on your board!");
  Serial.println("\nTesting pins in order:");
  for (int i = 0; i < numTestPins; i++) {
    Serial.printf("  - GPIO%d\n", testPins[i]);
  }
  Serial.println("\nStarting test...\n");
  
  testNextPin();
}

void loop() {
  // Quick color test on current pin
  if (strip != NULL) {
    // Flash red, green, blue quickly
    for (int i = 0; i < NUM_LEDS; i++) {
      strip->setPixelColor(i, strip->Color(255, 0, 0));  // Red
    }
    strip->show();
    delay(300);
    
    for (int i = 0; i < NUM_LEDS; i++) {
      strip->setPixelColor(i, strip->Color(0, 255, 0));  // Green
    }
    strip->show();
    delay(300);
    
    for (int i = 0; i < NUM_LEDS; i++) {
      strip->setPixelColor(i, strip->Color(0, 0, 255));  // Blue
    }
    strip->show();
    delay(300);
  }
  
  // Move to next pin
  currentPinIndex++;
  if (currentPinIndex >= numTestPins) {
    currentPinIndex = 0;
    Serial.println("\n--- Cycling back to first pin ---\n");
  }
  
  testNextPin();
}

void testNextPin() {
  // Clean up previous strip
  if (strip != NULL) {
    strip->clear();
    strip->show();
    delay(10);
    delete strip;
    strip = NULL;
  }
  
  // Create new strip on next pin
  int pin = testPins[currentPinIndex];
  Serial.printf("\n[%d/%d] Testing GPIO%d...\n", currentPinIndex + 1, numTestPins, pin);
  
  // Add watchdog reset to prevent crashes
  delay(100);
  
  strip = new Adafruit_NeoPixel(NUM_LEDS, pin, NEO_GRB + NEO_KHZ800);
  if (strip == NULL) {
    Serial.println("ERROR: Failed to create NeoPixel object!");
    return;
  }
  
  strip->begin();
  strip->setBrightness(50);  // 50/255 brightness
  
  // Flash white to indicate new pin test
  for (int i = 0; i < NUM_LEDS; i++) {
    strip->setPixelColor(i, strip->Color(255, 255, 255));
  }
  strip->show();
  delay(500);
  
  strip->clear();
  strip->show();
  delay(200);
}

void rainbowCycle(uint8_t wait) {
  for (int j = 0; j < 256; j++) {
    for (int i = 0; i < NUM_LEDS; i++) {
      strip->setPixelColor(i, Wheel((i * 256 / NUM_LEDS + j) & 255));
    }
    strip->show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip->Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip->Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip->Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
