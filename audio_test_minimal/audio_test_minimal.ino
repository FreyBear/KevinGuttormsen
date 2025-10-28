// Minimal Microphone and Speaker Test for ESP32-S3
// Diagnostic version to identify where the crash occurs

#include <driver/i2s.h>

// I2S Pin Configuration (from documentation)
#define I2S_MCLK 12
#define I2S_SCLK 13
#define I2S_LRCK 14
#define I2S_DOUT 15  // Speaker output
#define I2S_DIN  16  // Microphone input

// Audio Configuration
#define SAMPLE_RATE 16000
#define RECORD_TIME_SEC 2
#define RECORD_SAMPLES (SAMPLE_RATE * RECORD_TIME_SEC)
#define BEEP_FREQUENCY 1000  // Hz
#define BEEP_DURATION 500    // ms

// Audio buffer
int16_t* audioBuffer = NULL;
size_t bufferSize = 0;
bool bufferValid = false;

// I2S port
i2s_port_t i2s_port = I2S_NUM_0;
bool i2sInstalled = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== Minimal Audio Test ===");
  Serial.println("Step 1: Serial initialized");
  
  // Allocate audio buffer - try PSRAM first, then SRAM
  bufferSize = RECORD_SAMPLES * sizeof(int16_t);
  Serial.printf("Step 2: Attempting to allocate %d bytes for audio buffer...\n", bufferSize);
  
  // Try PSRAM first
  audioBuffer = (int16_t*)heap_caps_malloc(bufferSize, MALLOC_CAP_SPIRAM);
  if (audioBuffer != NULL) {
    Serial.println("Step 3: Audio buffer allocated in PSRAM");
    bufferValid = true;
  } else {
    // Fall back to SRAM
    Serial.println("Step 3: PSRAM allocation failed, trying SRAM...");
    audioBuffer = (int16_t*)malloc(bufferSize);
    if (audioBuffer != NULL) {
      Serial.println("Step 4: Audio buffer allocated in SRAM");
      bufferValid = true;
    } else {
      Serial.println("Step 4: ERROR - Failed to allocate audio buffer!");
      Serial.printf("Available heap: %d bytes\n", ESP.getFreeHeap());
      bufferValid = false;
    }
  }
  
  // Initialize I2S for playback (TX mode)
  Serial.println("Step 5: Initializing I2S for playback...");
  initI2SPlayback();
  
  Serial.println("Step 6: Setup complete. Starting test loop...\n");
}

void loop() {
  // Step 1: Beep
  Serial.println(">>> Playing beep...");
  playBeep();
  delay(500);
  
  // Step 2: Record
  Serial.println(">>> Recording 2 seconds...");
  recordAudio();
  delay(500);
  
  // Step 3: Beep
  Serial.println(">>> Playing beep...");
  playBeep();
  delay(500);
  
  // Step 4: Playback
  Serial.println(">>> Playing back recorded audio...");
  playbackAudio();
  delay(500);
  
  Serial.println(">>> Cycle complete. Repeating...\n");
  delay(1000);
}

// Initialize I2S for playback (speaker output)
void initI2SPlayback() {
  Serial.println("  - Uninstalling any existing I2S driver...");
  
  // Stop any existing I2S if it was installed
  if (i2sInstalled) {
    i2s_driver_uninstall(i2s_port);
    i2sInstalled = false;
  }
  
  Serial.println("  - Creating I2S configuration...");
  
  // I2S configuration for playback
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  
  // Pin configuration
  i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_MCLK,
    .bck_io_num = I2S_SCLK,
    .ws_io_num = I2S_LRCK,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  
  Serial.println("  - Installing I2S driver...");
  esp_err_t err = i2s_driver_install(i2s_port, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("  - ERROR: Failed to install I2S driver: %d\n", err);
    return;
  }
  
  Serial.println("  - Setting I2S pins...");
  err = i2s_set_pin(i2s_port, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("  - ERROR: Failed to set I2S pins: %d\n", err);
    return;
  }
  
  i2sInstalled = true;
  Serial.println("  - I2S initialized for playback");
}

// Initialize I2S for recording (microphone input)
void initI2SRecording() {
  Serial.println("  - Uninstalling existing I2S driver...");
  
  // Stop any existing I2S if it was installed
  if (i2sInstalled) {
    i2s_driver_uninstall(i2s_port);
    i2sInstalled = false;
  }
  
  Serial.println("  - Creating I2S configuration for recording...");
  
  // I2S configuration for recording
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .fixed_mclk = 0
  };
  
  // Pin configuration
  i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_MCLK,
    .bck_io_num = I2S_SCLK,
    .ws_io_num = I2S_LRCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DIN
  };
  
  Serial.println("  - Installing I2S driver for recording...");
  esp_err_t err = i2s_driver_install(i2s_port, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("  - ERROR: Failed to install I2S driver: %d\n", err);
    return;
  }
  
  Serial.println("  - Setting I2S pins for recording...");
  err = i2s_set_pin(i2s_port, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("  - ERROR: Failed to set I2S pins: %d\n", err);
    return;
  }
  
  i2sInstalled = true;
  Serial.println("  - I2S initialized for recording");
}

// Generate and play a beep tone
void playBeep() {
  initI2SPlayback();
  
  // Generate sine wave for beep
  int16_t* beepBuffer = (int16_t*)malloc(SAMPLE_RATE / 10 * sizeof(int16_t));  // 100ms buffer
  if (beepBuffer == NULL) {
    Serial.println("ERROR: Failed to allocate beep buffer!");
    return;
  }
  
  int beepSamples = SAMPLE_RATE / 10;
  
  // Generate sine wave
  for (int i = 0; i < beepSamples; i++) {
    float angle = 2.0 * M_PI * BEEP_FREQUENCY * i / SAMPLE_RATE;
    beepBuffer[i] = (int16_t)(32767 * sin(angle) * 0.5);  // 50% amplitude
  }
  
  // Play beep for BEEP_DURATION
  int repetitions = (BEEP_DURATION * SAMPLE_RATE) / (beepSamples * 1000);
  size_t bytesWritten = 0;
  
  for (int rep = 0; rep < repetitions; rep++) {
    i2s_write(i2s_port, beepBuffer, beepSamples * sizeof(int16_t), &bytesWritten, portMAX_DELAY);
  }
  
  free(beepBuffer);
  Serial.println("Beep played");
}

// Record audio from microphone
void recordAudio() {
  if (!bufferValid) {
    Serial.println("ERROR: Audio buffer not allocated!");
    return;
  }
  
  initI2SRecording();
  
  size_t bytesRead = 0;
  int samplesRead = 0;
  
  // Record for RECORD_TIME_SEC seconds
  while (samplesRead < RECORD_SAMPLES) {
    size_t toRead = (RECORD_SAMPLES - samplesRead) * sizeof(int16_t);
    esp_err_t err = i2s_read(i2s_port, &audioBuffer[samplesRead], toRead, &bytesRead, portMAX_DELAY);
    if (err != ESP_OK) {
      Serial.printf("ERROR: I2S read failed: %d\n", err);
      break;
    }
    samplesRead += bytesRead / sizeof(int16_t);
  }
  
  Serial.printf("Recorded %d samples (%d bytes)\n", samplesRead, bytesRead);
}

// Playback recorded audio
void playbackAudio() {
  if (!bufferValid) {
    Serial.println("ERROR: Audio buffer not allocated!");
    return;
  }
  
  initI2SPlayback();
  
  size_t bytesWritten = 0;
  esp_err_t err = i2s_write(i2s_port, audioBuffer, bufferSize, &bytesWritten, portMAX_DELAY);
  if (err != ESP_OK) {
    Serial.printf("ERROR: I2S write failed: %d\n", err);
    return;
  }
  
  Serial.printf("Played back %d bytes\n", bytesWritten);
}
