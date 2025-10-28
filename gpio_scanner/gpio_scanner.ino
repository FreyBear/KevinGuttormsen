// GPIO Scanner Test for ESP32-S3
// Tests various GPIO pins to identify EXIO9 (PA_EN - Speaker Amplifier Enable)
// Watch for LED brightness changes or speaker behavior when each pin is toggled

#include <Adafruit_NeoPixel.h>

// LED configuration
#define LED_PIN 38
#define NUM_LEDS 7

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// GPIO pins to test (excluding reserved pins)
const int testPins[] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
  17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 39, 43, 44, 45, 46, 47, 48
};
const int numPins = sizeof(testPins) / sizeof(testPins[0]);

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== GPIO Scanner Test ===");
  Serial.println("Testing GPIO pins to identify EXIO9 (PA_EN)");
  Serial.println("Watch the LED ring for brightness changes or listen for speaker behavior");
  Serial.println("When you notice a change, note the GPIO number shown here\n");
  
  // Initialize LED ring
  strip.begin();
  strip.setBrightness(100);
  strip.clear();
  strip.show();
  
  // Set all test pins as outputs
  for (int i = 0; i < numPins; i++) {
    pinMode(testPins[i], OUTPUT);
    digitalWrite(testPins[i], LOW);
  }
  
  Serial.println("All GPIO pins initialized as outputs (LOW)");
  Serial.println("Starting GPIO test sequence...\n");
}

void loop() {
  // Test each GPIO pin
  for (int i = 0; i < numPins; i++) {
    int pin = testPins[i];
    
    // Set this pin HIGH
    digitalWrite(pin, HIGH);
    
    Serial.printf("GPIO%d: HIGH - Watch for changes (LED brightness, speaker behavior, etc.)\n", pin);
    
    // Show on LED ring which pin is being tested
    strip.clear();
    strip.setPixelColor(i % NUM_LEDS, strip.Color(0, 100, 0));  // Green
    strip.show();
    
    delay(2000);  // 2 seconds to observe
    
    // Set this pin LOW
    digitalWrite(pin, LOW);
    Serial.printf("GPIO%d: LOW\n\n", pin);
    
    strip.clear();
    strip.show();
    
    delay(500);
  }
  
  Serial.println("=== Cycle complete, repeating... ===\n");
  delay(2000);
}
