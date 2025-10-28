// Ultra-minimal test - no includes, just Serial

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("HELLO WORLD - Setup started");
  delay(1000);
}

void loop() {
  Serial.println("Loop running");
  delay(1000);
}
