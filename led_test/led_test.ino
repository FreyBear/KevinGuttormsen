// Individual LED Test for ESP32-S3 RGB LED Ring
// Tests each LED individually with Red, Green, Blue on GPIO38

#include <Adafruit_NeoPixel.h>

// LED configuration
#define LED_PIN 38
#define NUM_LEDS 7

// Test colors
const uint32_t testColors[] = {
  0xFF0000, // Red
  0x00FF00, // Green
  0x0000FF  // Blue
};
const char* colorNames[] = {
  "RED", "GREEN", "BLUE"
};
const int numColors = sizeof(testColors) / sizeof(testColors[0]);

Adafruit_NeoPixel* strip = NULL;
int currentLED = 0;
int currentColor = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n=== Individual LED Test ===");
  Serial.println("Testing each LED individually with Red, Green, Blue");
  Serial.println("Watch which LED lights up and tell me what color you see:");
  Serial.println("This will help determine the correct color order for your LEDs");
  Serial.println("\nStarting test...\n");

  // Initialize NeoPixel strip with GRB (most common)
  strip = new Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
  if (strip == NULL) {
    Serial.println("ERROR: Failed to create NeoPixel object!");
    return;
  }

  strip->begin();
  strip->setBrightness(100);  // Brighter for better visibility
  strip->clear();
  strip->show();
}

void loop() {
  if (strip != NULL) {
    // Clear all LEDs first
    strip->clear();

    // Light up only the current LED with the current color
    strip->setPixelColor(currentLED, testColors[currentColor]);
    strip->show();

    // Display info
    Serial.printf("LED %d showing %s - Tell me what color you actually see!\n",
                  currentLED, colorNames[currentColor]);

    delay(3000);  // Show for 3 seconds

    // Move to next color
    currentColor++;
    if (currentColor >= numColors) {
      currentColor = 0;
      // Move to next LED
      currentLED++;
      if (currentLED >= NUM_LEDS) {
        currentLED = 0;
        Serial.println("\n--- Cycling back to LED 0 ---\n");
      }
    }
  }
}
