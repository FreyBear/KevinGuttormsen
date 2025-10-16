// Plush Toy Security Device (PTSD) - ESP32-S3 Based System
// Modular structure for easy maintenance and updates
// Supports multiple operation modes: Armed, Social, Sleeping
// Accelerometer (MPU-6050) power management for energy efficiency

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD_MMC.h>
#include <ESP8266Audio.h>
#include <AudioFileSourceSD.h>
#include <AudioGeneratorWAV.h>
#include <AudioOutputI2S.h>

// Enum for operation modes
enum OperationMode {
  ARMED,
  SOCIAL,
  SLEEPING,
  KIDNAPPED
};

// Global variables
Adafruit_MPU6050 mpu;
OperationMode currentMode = ARMED;  // Default mode
bool accelerometerActive = false;

// Kidnapped mode variables
WebServer server(80);
AudioGeneratorWAV *wav;
AudioFileSourceSD *file;
AudioOutputI2S *out;
bool isPlayingAlarm = false;
unsigned long alarmStartTime = 0;
unsigned long lastMovementTime = 0;
int currentAlarmFileIndex = 0;
const unsigned long MOVEMENT_TIMEOUT = 3000;  // 3 seconds
const unsigned long MAX_ALARM_DURATION = 600000;  // 10 minutes

// Wake word recording variables
AudioInputI2S *audioInput;
AudioFileSinkSD *audioSink;
bool isRecordingWakeWord = false;
unsigned long recordingStartTime = 0;
const unsigned long RECORDING_DURATION = 2500;  // 2.5 seconds
int wakeWordSampleCount = 0;

// WiFi AP settings for kidnapped mode
const char* apSSID = "JegErPingnappa";
const char* apPassword = "Kevin";

// Pin definitions (adjust based on board)
const int MODE_BUTTON_PIN = 0;  // General mode switching button
const int LEFT_LEG_BUTTON_PIN = 3;   // Left leg button (pull-up, connected to GND)
const int RIGHT_LEG_BUTTON_PIN = 4;  // Right leg button (pull-up, connected to GND)
const int LEFT_WING_BUTTON_PIN = 5;  // Left wing button (pull-up, connected to GND)
const int RIGHT_WING_BUTTON_PIN = 6; // Right wing button (pull-up, connected to GND)
const int BEAK_BUTTON_PIN = 7;       // Beak button (pull-up, connected to GND)

// Thresholds for movement detection (tune these values)
const float MOVEMENT_THRESHOLD = 2.0;  // m/s^2
const float FREE_FALL_THRESHOLD = 0.5; // m/s^2

// Function prototypes for modularity
void initializeHardware();
void setAccelerometerState(bool active);
void handleModeSwitching();
void handleButtonInputs();
void armedModeBehavior();
void socialModeBehavior();
void sleepingModeBehavior();
void detectMovement();
void playSound(const char* soundFile);  // Placeholder for audio playback

void setup() {
  Serial.begin(115200);
  initializeHardware();
  Serial.println("PTSD System Initialized. Current Mode: Armed");
}

void loop() {
  // Handle wake word recording if active
  if (isRecordingWakeWord) {
    unsigned long currentTime = millis();
    if (currentTime - recordingStartTime >= RECORDING_DURATION) {
      stopWakeWordRecording();
    }
    delay(10);  // Short delay during recording
    return;  // Don't process other logic during recording
  }

  handleModeSwitching();
  handleButtonInputs();

  switch (currentMode) {
    case ARMED:
      armedModeBehavior();
      break;
    case SOCIAL:
      socialModeBehavior();
      break;
    case SLEEPING:
      sleepingModeBehavior();
      break;
    case KIDNAPPED:
      kidnappedModeBehavior();
      break;
  }

  delay(100);  // Small delay to prevent overwhelming the loop
}

void initializeHardware() {
  // Initialize I2C on ESP32-S3: SCL=GPIO10, SDA=GPIO11
  Wire.begin(10, 11);

  // Initialize MPU-6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set accelerometer to sleep initially for power saving
  setAccelerometerState(false);

  // Initialize all buttons as pull-up inputs (connected to GND)
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LEFT_LEG_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_LEG_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LEFT_WING_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_WING_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BEAK_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("All buttons initialized as pull-up inputs");
}

void setAccelerometerState(bool active) {
  if (active && !accelerometerActive) {
    mpu.setSleepEnabled(false);
    accelerometerActive = true;
    Serial.println("Accelerometer activated");
  } else if (!active && accelerometerActive) {
    mpu.setSleepEnabled(true);
    accelerometerActive = false;
    Serial.println("Accelerometer deactivated for power saving");
  }
}

void handleModeSwitching() {
  // Simple mode switching via button press (debounce not implemented for simplicity)
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(MODE_BUTTON_PIN);

  if (lastButtonState == HIGH && currentButtonState == LOW) {
    // Button pressed, cycle through modes
    currentMode = static_cast<OperationMode>((currentMode + 1) % 3);
    Serial.print("Mode switched to: ");
    switch (currentMode) {
      case ARMED: Serial.println("Armed"); break;
      case SOCIAL: Serial.println("Social"); break;
      case SLEEPING: Serial.println("Sleeping"); break;
    }
  }
  lastButtonState = currentButtonState;

  // Alternatively, you could use serial input or other triggers for mode switching
  // Example: if (Serial.available()) { /* read and set mode */ }
}

void handleButtonInputs() {
  // Read all button states (LOW = pressed, HIGH = not pressed due to pull-up)
  bool leftLegPressed = (digitalRead(LEFT_LEG_BUTTON_PIN) == LOW);
  bool rightLegPressed = (digitalRead(RIGHT_LEG_BUTTON_PIN) == LOW);
  bool leftWingPressed = (digitalRead(LEFT_WING_BUTTON_PIN) == LOW);
  bool rightWingPressed = (digitalRead(RIGHT_WING_BUTTON_PIN) == LOW);
  bool beakPressed = (digitalRead(BEAK_BUTTON_PIN) == LOW);

  // Check for wake word recording trigger: hold left leg button for 2 seconds
  static unsigned long leftLegPressStart = 0;
  static bool leftLegHeld = false;

  if (leftLegPressed && !leftLegHeld) {
    leftLegPressStart = millis();
    leftLegHeld = true;
  } else if (!leftLegPressed && leftLegHeld) {
    leftLegHeld = false;
    unsigned long pressDuration = millis() - leftLegPressStart;

    // If held for 2+ seconds, start wake word recording
    if (pressDuration >= 2000 && !isRecordingWakeWord) {
      Serial.println("WAKE WORD RECORDING MODE ACTIVATED!");
      startWakeWordRecording();
      return;  // Don't process other button logic during recording
    }
  }

  // Check for disarm sequence: both leg buttons pressed simultaneously
  static bool disarmSequenceActive = false;
  if (leftLegPressed && rightLegPressed && currentMode == ARMED) {
    if (!disarmSequenceActive) {
      disarmSequenceActive = true;
      Serial.println("DISARM SEQUENCE ACTIVATED: Both legs pressed!");
      // Switch to social mode when disarmed
      currentMode = SOCIAL;
      Serial.println("Kevin disarmed! Switched to Social mode.");
      playSound("disarmed.wav");  // Placeholder
    }
  } else {
    disarmSequenceActive = false;
  }

  // Handle individual button presses based on current mode
  if (currentMode == SOCIAL) {
    if (leftWingPressed) {
      Serial.println("Left wing touched - playful response");
      playSound("wing_flap.wav");  // Placeholder
    }
    if (rightWingPressed) {
      Serial.println("Right wing touched - playful response");
      playSound("wing_flap.wav");  // Placeholder
    }
    if (beakPressed) {
      Serial.println("Beak touched - affectionate response");
      playSound("kiss.wav");  // Placeholder
    }
  }

  // In armed mode, individual button presses might trigger alerts
  if (currentMode == ARMED && !disarmSequenceActive) {
    if (leftWingPressed || rightWingPressed || beakPressed) {
      Serial.println("WARNING: Touch detected in armed mode!");
      playSound("warning.wav");  // Placeholder
    }
  }
}

void armedModeBehavior() {
  // Activate accelerometer for movement detection
  setAccelerometerState(true);

  // Detect unexpected movements
  detectMovement();

  // Add armed-specific logic here (e.g., alarm sounds, LED indicators)
  // For now, just print status
  Serial.println("Armed mode: Monitoring for unauthorized movement");
}

void socialModeBehavior() {
  // Activate accelerometer for interaction detection
  setAccelerometerState(true);

  // Detect playful interactions
  detectMovement();

  // Add social-specific logic here (e.g., happy sounds, responses to touch)
  Serial.println("Social mode: Ready for play and interaction");
}

void sleepingModeBehavior() {
  // Deactivate accelerometer to save power
  setAccelerometerState(false);

  // Minimal activity: perhaps periodic checks or deep sleep
  Serial.println("Sleeping mode: Battery saving active");

  // You could implement deep sleep here for extended battery life
  // esp_deep_sleep_start();  // Uncomment and configure as needed
}

void detectMovement() {
  if (!accelerometerActive) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate total acceleration magnitude
  float totalAccel = sqrt(a.acceleration.x * a.acceleration.x +
                          a.acceleration.y * a.acceleration.y +
                          a.acceleration.z * a.acceleration.z);

  // Check for significant movement
  if (totalAccel > MOVEMENT_THRESHOLD) {
    Serial.print("Movement detected! Magnitude: ");
    Serial.println(totalAccel);
    lastMovementTime = millis();  // Update last movement time

    // React based on mode
    if (currentMode == ARMED) {
      // Trigger kidnapped mode immediately
      Serial.println("KIDNAPPED MODE ACTIVATED!");
      currentMode = KIDNAPPED;
      alarmStartTime = millis();
      startKidnappedMode();
    } else if (currentMode == SOCIAL) {
      // Trigger playful response
      playSound("giggle.wav");  // Placeholder
    }
  }

  // Optional: Detect free-fall or other events
  if (totalAccel < FREE_FALL_THRESHOLD) {
    Serial.println("Free-fall detected!");
    // Handle free-fall event
  }
}

void kidnappedModeBehavior() {
  // Handle web server requests
  server.handleClient();

  // Check if alarm should continue or stop
  unsigned long currentTime = millis();

  // Check if movement has stopped for 3 seconds
  if (currentTime - lastMovementTime > MOVEMENT_TIMEOUT && isPlayingAlarm) {
    Serial.println("Movement stopped - advancing to next alarm file");
    currentAlarmFileIndex++;
    playNextAlarmFile();
  }

  // Check if 10 minutes have passed
  if (currentTime - alarmStartTime > MAX_ALARM_DURATION) {
    Serial.println("Maximum alarm duration reached - stopping alarm");
    stopAlarm();
    return;
  }

  // Continue playing current alarm file
  if (wav && isPlayingAlarm) {
    if (!wav->loop()) {
      // Current file finished, play next one
      currentAlarmFileIndex++;
      playNextAlarmFile();
    }
  }
}

void startKidnappedMode() {
  Serial.println("Starting kidnapped mode...");

  // Initialize SD card
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    return;
  }

  // Initialize audio output
  out = new AudioOutputI2S();
  out->SetPinout(12, 13, 14);  // BCLK, LRCLK, DOUT pins (adjust as needed)

  // Start WiFi AP
  WiFi.softAP(apSSID, apPassword);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/activate_alarm", handleActivateAlarm);
  server.on("/disarm", handleDisarm);
  server.begin();

  // Start playing first alarm file
  currentAlarmFileIndex = 0;
  playNextAlarmFile();
}

void playNextAlarmFile() {
  if (wav) {
    wav->stop();
    delete wav;
    wav = nullptr;
  }
  if (file) {
    delete file;
    file = nullptr;
  }

  // Create filename for next alarm file
  char filename[32];
  sprintf(filename, "/SecurityResponse/%d.wav", currentAlarmFileIndex + 1);

  Serial.print("Playing alarm file: ");
  Serial.println(filename);

  file = new AudioFileSourceSD(filename);
  if (!file->isOpen()) {
    Serial.println("Failed to open alarm file");
    // If file doesn't exist, loop back to first file
    currentAlarmFileIndex = 0;
    sprintf(filename, "/SecurityResponse/1.wav", currentAlarmFileIndex + 1);
    file = new AudioFileSourceSD(filename);
    if (!file->isOpen()) {
      Serial.println("No alarm files found!");
      return;
    }
  }

  wav = new AudioGeneratorWAV();
  wav->begin(file, out);
  isPlayingAlarm = true;
}

void stopAlarm() {
  if (wav) {
    wav->stop();
    delete wav;
    wav = nullptr;
  }
  if (file) {
    delete file;
    file = nullptr;
  }
  if (out) {
    delete out;
    out = nullptr;
  }

  isPlayingAlarm = false;
  WiFi.softAPdisconnect(true);
  server.stop();
  Serial.println("Alarm stopped - Kevin rescued!");
}

void handleRoot() {
  String html = "<html><body style='font-family: Arial, sans-serif; text-align: center; padding: 20px;'>";
  html += "<h1 style='color: #ff4444;'>Kevin has been kidnapped!</h1>";
  html += "<p>Kevin needs your help! Choose an option below:</p>";

  // Activate Alarm button
  html += "<form action='/activate_alarm' method='POST' style='margin: 20px;'>";
  html += "<input type='submit' value='Activate Alarm' style='background-color: #ff4444; color: white; padding: 15px 30px; font-size: 18px; border: none; border-radius: 5px; cursor: pointer;'>";
  html += "</form>";

  // Rescue Kevin form with password
  html += "<form action='/disarm' method='POST' style='margin: 20px;'>";
  html += "<p>Enter password to rescue Kevin:</p>";
  html += "<input type='password' name='password' placeholder='Enter password' style='padding: 10px; font-size: 16px; margin: 10px;' required>";
  html += "<br>";
  html += "<input type='submit' value='Rescue Kevin & Reset to Social Mode' style='background-color: #44aa44; color: white; padding: 15px 30px; font-size: 18px; border: none; border-radius: 5px; cursor: pointer;'>";
  html += "</form>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleActivateAlarm() {
  Serial.println("Manual alarm activation requested via web interface!");
  // Play a loud alarm sound immediately
  playImmediateAlarm();
  server.send(200, "text/html", "<h1 style='color: #ff4444;'>ALARM ACTIVATED!</h1><p>Kevin is now playing a loud alarm sound.</p><p><a href='/'>Back to main menu</a></p>");
}

void handleDisarm() {
  if (server.hasArg("password") && server.arg("password") == "Kevin") {
    Serial.println("Kevin has been rescued via web interface with correct password!");
    currentMode = SOCIAL;  // Reset to social mode as requested
    stopAlarm();
    server.send(200, "text/html", "<h1 style='color: #44aa44;'>Kevin has been rescued!</h1><p>The alarm has been stopped and Kevin is now in Social mode.</p>");
  } else {
    Serial.println("Incorrect password attempt for Kevin rescue!");
    server.send(200, "text/html", "<h1 style='color: #ff4444;'>Incorrect Password!</h1><p>Please try again.</p><p><a href='/'>Back to main menu</a></p>");
  }
}

void playImmediateAlarm() {
  // Play a loud immediate alarm sound (different from the escalating sequence)
  Serial.println("Playing immediate loud alarm!");
  // This could play a specific alarm file or generate a tone
  // For now, we'll play the first security response file immediately
  if (wav) {
    wav->stop();
    delete wav;
    wav = nullptr;
  }
  if (file) {
    delete file;
    file = nullptr;
  }

  file = new AudioFileSourceSD("/SecurityResponse/1.wav");
  if (file->isOpen()) {
    wav = new AudioGeneratorWAV();
    wav->begin(file, out);
    isPlayingAlarm = true;
    Serial.println("Immediate alarm started!");
  } else {
    Serial.println("Could not play immediate alarm - file not found");
  }
}

void startWakeWordRecording() {
  Serial.println("Initializing wake word recording...");

  // Initialize SD card if not already done
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed for wake word recording!");
    return;
  }

  // Create wakeword_samples directory if it doesn't exist
  if (!SD_MMC.exists("/wakeword_samples")) {
    SD_MMC.mkdir("/wakeword_samples");
    Serial.println("Created /wakeword_samples directory");
  }

  // Initialize audio input from microphones
  audioInput = new AudioInputI2S();
  audioInput->SetPinout(15, 16, 17);  // Adjust pins based on ESP32-S3 I2S microphone pins

  // Generate filename for new sample
  wakeWordSampleCount++;
  char filename[32];
  sprintf(filename, "/wakeword_samples/sample_%03d.wav", wakeWordSampleCount);

  // Initialize audio sink for WAV file recording
  audioSink = new AudioFileSinkSD(filename);

  // Play "pling" sound to indicate recording start
  Serial.println("Playing recording cue sound...");
  // For now, just print - you would play a short tone here
  Serial.println("PLING! Start saying 'Hei Kevin'...");

  // Start recording
  recordingStartTime = millis();
  isRecordingWakeWord = true;

  Serial.print("Recording to: ");
  Serial.println(filename);
}

void stopWakeWordRecording() {
  if (audioInput) {
    delete audioInput;
    audioInput = nullptr;
  }
  if (audioSink) {
    delete audioSink;
    audioSink = nullptr;
  }

  isRecordingWakeWord = false;
  Serial.println("Wake word recording completed and saved!");
}

void playSound(const char* soundFile) {
  // Placeholder for audio playback implementation
  // Use ESP8266Audio library or similar for WAV playback via I2S
  Serial.print("Playing sound: ");
  Serial.println(soundFile);
  // Actual implementation would involve AudioFileSourceSD, AudioGeneratorWAV, etc.
}
