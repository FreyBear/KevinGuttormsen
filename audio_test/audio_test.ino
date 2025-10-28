// Audio Test Program for ESP32-S3 AI Smart Speaker Board
// Tests microphone input and speaker output
// Sequence: PLING -> Record 3 seconds -> PLING -> Playback

#include <driver/i2s.h>

// I2S pins based on ESP32-S3 board documentation
#define I2S_MCLK 12
#define I2S_SCLK 13
#define I2S_LRCK 14
#define I2S_DOUT 15
#define I2S_DIN  16

// Audio parameters
#define SAMPLE_RATE 16000
#define BITS_PER_SAMPLE 16
#define RECORDING_TIME 3  // seconds
#define BUFFER_SIZE 1024
#define AUDIO_BUFFER_SIZE (SAMPLE_RATE * RECORDING_TIME * 2)  // 2 bytes per sample

// Global audio buffer
int16_t* audioBuffer = NULL;
size_t audioBufferIndex = 0;

void setup() {
  Serial.begin(115200);
  
  // Wait for Serial to be ready (important for ESP32-S3 with USB CDC)
  while (!Serial) {
    delay(10);
  }
  delay(1000);
  
  Serial.println("\n=== ESP32-S3 Audio Test ===");
  Serial.println("Testing microphone and speaker...");
  
  // Allocate memory for audio recording
  audioBuffer = (int16_t*)malloc(AUDIO_BUFFER_SIZE);
  if (audioBuffer == NULL) {
    Serial.println("ERROR: Failed to allocate audio buffer!");
    while(1) delay(1000);
  }
  Serial.println("Audio buffer allocated successfully");
  
  // Initialize I2S for playback (speaker)
  initI2SOutput();
  
  // Play first PLING
  Serial.println("\n1. Playing first PLING...");
  playPlingSound();
  delay(500);
  
  // Switch to recording mode
  Serial.println("\n2. Starting 3-second recording...");
  Serial.println("SPEAK NOW!");
  i2s_driver_uninstall(I2S_NUM_0);
  initI2SInput();
  recordAudio();
  
  // Switch back to playback mode
  Serial.println("\n3. Recording complete. Playing second PLING...");
  i2s_driver_uninstall(I2S_NUM_0);
  initI2SOutput();
  delay(200);
  playPlingSound();
  delay(500);
  
  // Play back recorded audio
  Serial.println("\n4. Playing back recorded audio...");
  playbackRecording();
  
  Serial.println("\n=== Audio Test Complete ===");
  Serial.println("If you heard two plings and your recorded voice, the audio system works!");
}

void loop() {
  // Test complete - do nothing
  delay(1000);
}

void initI2SOutput() {
  Serial.println("Initializing I2S for speaker output...");
  
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
  
  i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_MCLK,
    .bck_io_num = I2S_SCLK,
    .ws_io_num = I2S_LRCK,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  
  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("ERROR: I2S driver install failed: %d\n", err);
    return;
  }
  
  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: I2S pin config failed: %d\n", err);
    return;
  }
  
  Serial.println("I2S output initialized successfully");
}

void initI2SInput() {
  Serial.println("Initializing I2S for microphone input...");
  
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
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  
  i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_MCLK,
    .bck_io_num = I2S_SCLK,
    .ws_io_num = I2S_LRCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DIN
  };
  
  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("ERROR: I2S driver install failed: %d\n", err);
    return;
  }
  
  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: I2S pin config failed: %d\n", err);
    return;
  }
  
  Serial.println("I2S input initialized successfully");
}

void playPlingSound() {
  // Generate a simple "pling" sound (1000 Hz tone for 150ms)
  const int duration_ms = 150;
  const int frequency = 1000;
  const int samples = (SAMPLE_RATE * duration_ms) / 1000;
  
  int16_t* plingBuffer = (int16_t*)malloc(samples * sizeof(int16_t));
  if (plingBuffer == NULL) {
    Serial.println("ERROR: Failed to allocate pling buffer");
    return;
  }
  
  // Generate sine wave with envelope
  for (int i = 0; i < samples; i++) {
    float t = (float)i / SAMPLE_RATE;
    float envelope = 1.0 - ((float)i / samples);  // Fade out
    float sample = sin(2.0 * PI * frequency * t) * envelope * 10000.0;
    plingBuffer[i] = (int16_t)sample;
  }
  
  // Play the sound
  size_t bytes_written;
  i2s_write(I2S_NUM_0, plingBuffer, samples * sizeof(int16_t), &bytes_written, portMAX_DELAY);
  
  free(plingBuffer);
  Serial.printf("PLING! (%d samples played)\n", samples);
}

void recordAudio() {
  audioBufferIndex = 0;
  size_t totalSamples = SAMPLE_RATE * RECORDING_TIME;
  size_t samplesRecorded = 0;
  
  int16_t tempBuffer[BUFFER_SIZE];
  
  unsigned long startTime = millis();
  
  while (samplesRecorded < totalSamples) {
    size_t bytes_read = 0;
    
    // Read from microphone
    esp_err_t result = i2s_read(I2S_NUM_0, tempBuffer, sizeof(tempBuffer), &bytes_read, portMAX_DELAY);
    
    if (result == ESP_OK && bytes_read > 0) {
      size_t samples_read = bytes_read / sizeof(int16_t);
      
      // Copy to main buffer
      for (size_t i = 0; i < samples_read && audioBufferIndex < (AUDIO_BUFFER_SIZE / sizeof(int16_t)); i++) {
        audioBuffer[audioBufferIndex++] = tempBuffer[i];
      }
      
      samplesRecorded += samples_read;
      
      // Progress indicator
      if (samplesRecorded % SAMPLE_RATE == 0) {
        Serial.printf("Recording... %d seconds\n", samplesRecorded / SAMPLE_RATE);
      }
    }
  }
  
  unsigned long duration = millis() - startTime;
  Serial.printf("Recording complete: %d samples in %lu ms\n", samplesRecorded, duration);
}

void playbackRecording() {
  if (audioBufferIndex == 0) {
    Serial.println("ERROR: No audio recorded!");
    return;
  }
  
  Serial.printf("Playing back %d samples...\n", audioBufferIndex);
  
  size_t bytes_written;
  i2s_write(I2S_NUM_0, audioBuffer, audioBufferIndex * sizeof(int16_t), &bytes_written, portMAX_DELAY);
  
  Serial.printf("Playback complete (%d bytes written)\n", bytes_written);
}
