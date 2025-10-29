// Microphone and Speaker Test for ESP32-S3
// Sequence: Beep -> Record 2 sec -> Beep -> Playback -> Repeat
// Uses I2S for audio I/O

#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <driver/i2c.h>
#include <es8311.h>
#include <es7210.h>

// I2S Pin Configuration (from documentation)
#define I2S_MCLK 12
#define I2S_SCLK 13
#define I2S_LRCK 14
// Per board docs:
//  - ES8311 (Speaker DAC) DSDIN <= ESP32 I2S DOUT on GPIO16
//  - ES7210 (Mic ADC) ASDOUT => ESP32 I2S DIN on GPIO15
#define I2S_DOUT 16  // Speaker output -> ES8311 DSDIN
#define I2S_DIN  15  // Microphone input <- ES7210 ASDOUT

// I2C Configuration for codecs and expander
#define I2C_SDA 11
#define I2C_SCL 10
#define I2C_PORT I2C_NUM_0

// Device addresses (from scan and docs)
#define ES8311_ADDR 0x18
#define ES7210_ADDR 0x40
#define TCA9555_ADDR 0x20

// Audio Configuration
#define SAMPLE_RATE 16000
#define RECORD_TIME_SEC 2
// Stereo frames (L+R), 16-bit samples
#define CHANNELS 2
#define RECORD_FRAMES (SAMPLE_RATE * RECORD_TIME_SEC)
#define BEEP_FREQUENCY 1000  // Hz
#define BEEP_DURATION 500    // ms

// Audio buffer (stored in PSRAM or SRAM)
int16_t* audioBuffer = NULL;  // interleaved stereo (L,R)
size_t bufferSize = 0;        // bytes
bool bufferValid = false;

// I2S port
i2s_port_t i2s_port = I2S_NUM_0;
bool i2sInstalled = false;

// Codec handles
es8311_handle_t es8311_handle_ = NULL;
es7210_dev_handle_t es7210_handle_ = NULL;
// Track legacy I2C init (old driver)
static bool i2c_old_inited = false;

// Forward decls
bool init_codecs_and_amp();
void initI2SPlayback();
void initI2SRecording();
void playBeep();
void recordAudio();
void playbackAudio();
esp_err_t ensure_i2c_old_driver();
esp_err_t i2c_write_reg8(uint8_t addr, uint8_t reg, uint8_t data);
esp_err_t i2c_read_reg8(uint8_t addr, uint8_t reg, uint8_t *data);

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== Microphone and Speaker Test ===");
  Serial.println("Sequence: Beep -> Record 2s -> Beep -> Playback -> Repeat");
  
  // Initialize legacy ESP-IDF I2C master driver (avoid mixing with new driver)
  if (ensure_i2c_old_driver() != ESP_OK) {
    Serial.println("ERROR: Failed to init legacy I2C driver");
  }
  
  // Initialize codecs and enable amplifier
  Serial.println("\nInitializing codecs (ES8311/ES7210) and enabling amplifier via TCA9555...");
  if (!init_codecs_and_amp()) {
    Serial.println("ERROR: Codec/amplifier init failed - sound may be muted");
  }
  
  // Allocate audio buffer - try PSRAM first, then SRAM
  bufferSize = (size_t)RECORD_FRAMES * CHANNELS * sizeof(int16_t);
  Serial.printf("Attempting to allocate %d bytes for audio buffer...\n", bufferSize);
  
  // Try PSRAM first
  audioBuffer = (int16_t*)heap_caps_malloc(bufferSize, MALLOC_CAP_SPIRAM);
  if (audioBuffer != NULL) {
    Serial.println("Audio buffer allocated in PSRAM");
    bufferValid = true;
  } else {
    // Fall back to SRAM
    Serial.println("PSRAM allocation failed, trying SRAM...");
    audioBuffer = (int16_t*)malloc(bufferSize);
    if (audioBuffer != NULL) {
      Serial.println("Audio buffer allocated in SRAM");
      bufferValid = true;
    } else {
      Serial.println("ERROR: Failed to allocate audio buffer in both PSRAM and SRAM!");
      Serial.printf("Available heap: %d bytes\n", ESP.getFreeHeap());
      bufferValid = false;
    }
  }
  
  // Initialize I2S for playback (TX mode)
  initI2SPlayback();
  
  Serial.println("Setup complete. Starting test loop...\n");
}

// Initialize codecs and enable amplifier (PA_EN)
bool init_codecs_and_amp() {
  // TCA9555: set P10 (EXIO8) as output and drive HIGH
  // Registers: 0x07 = Configuration Port1 (1=input, 0=output); 0x03 = Output Port1
  uint8_t cfg1 = 0xFF;
  if (i2c_read_reg8(TCA9555_ADDR, 0x07, &cfg1) != ESP_OK) cfg1 = 0xFF;
  cfg1 &= ~(1 << 0); // P10 as output
  i2c_write_reg8(TCA9555_ADDR, 0x07, cfg1);
  uint8_t out1 = 0x00;
  if (i2c_read_reg8(TCA9555_ADDR, 0x03, &out1) != ESP_OK) out1 = 0x00;
  out1 |= (1 << 0); // P10 HIGH
  if (i2c_write_reg8(TCA9555_ADDR, 0x03, out1) == ESP_OK) {
    Serial.println("  PA_EN set HIGH via TCA9555 (EXIO8)");
  } else {
    Serial.println("  ERROR: Failed to set PA_EN high on TCA9555");
  }

  // Initialize ES8311 (DAC / speaker)
  es8311_handle_ = es8311_create(I2C_PORT, ES8311_ADDR);
  if (!es8311_handle_) {
    Serial.println("  ERROR: es8311_create failed");
    return false;
  }
  es8311_clock_config_t clk_cfg = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = SAMPLE_RATE * 256,
    .sample_frequency = SAMPLE_RATE
  };
  if (es8311_init(es8311_handle_, &clk_cfg, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16) != ESP_OK) {
    Serial.println("  ERROR: es8311_init failed");
    return false;
  }
  es8311_voice_mute(es8311_handle_, false);
  es8311_voice_volume_set(es8311_handle_, 80, NULL); // 0..100
  es8311_microphone_config(es8311_handle_, false);
  Serial.println("  ES8311 initialized (DAC)");

  // Initialize ES7210 (ADC / microphones), TDM enabled as in example
  es7210_i2c_config_t i2c_conf = {
    .i2c_port = I2C_PORT,
    .i2c_addr = ES7210_ADDR
  };
  if (es7210_new_codec(&i2c_conf, &es7210_handle_) != ESP_OK) {
    Serial.println("  ERROR: es7210_new_codec failed");
    return false;
  }
  es7210_codec_config_t adc_cfg = {
    .sample_rate_hz = SAMPLE_RATE,
    .mclk_ratio = 256,
    .i2s_format = ES7210_I2S_FMT_I2S,
    .bit_width = ES7210_I2S_BITS_16B,
    .mic_bias = ES7210_MIC_BIAS_2V87,
    .mic_gain = ES7210_MIC_GAIN_30DB,
  };
  adc_cfg.flags.tdm_enable = 1;
  if (es7210_config_codec(es7210_handle_, &adc_cfg) != ESP_OK) {
    Serial.println("  ERROR: es7210_config_codec failed");
    return false;
  }
  es7210_config_volume(es7210_handle_, 0);
  Serial.println("  ES7210 initialized (ADC)");

  return true;
}

// Ensure legacy (old) I2C driver is installed on I2C_PORT
esp_err_t ensure_i2c_old_driver() {
  if (i2c_old_inited) return ESP_OK;
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)I2C_SDA;
  conf.scl_io_num = (gpio_num_t)I2C_SCL;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 100000; // 100kHz
  esp_err_t err = i2c_param_config(I2C_PORT, &conf);
  if (err != ESP_OK) return err;
  err = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
  if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
    i2c_old_inited = true;
    return ESP_OK;
  }
  return err;
}

// Old-driver helper: write 8-bit register
esp_err_t i2c_write_reg8(uint8_t addr, uint8_t reg, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

// Old-driver helper: read 8-bit register
esp_err_t i2c_read_reg8(uint8_t addr, uint8_t reg, uint8_t *data) {
  if (!data) return ESP_ERR_INVALID_ARG;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
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
  // Stop any existing I2S if it was installed
  if (i2sInstalled) {
    i2s_driver_uninstall(i2s_port);
    i2sInstalled = false;
  }
  
  // I2S configuration for playback
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // stereo frames
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll = true,              // use APLL for accurate audio clock
    .tx_desc_auto_clear = true,
    .fixed_mclk = SAMPLE_RATE * 256 // common MCLK ratio for codecs
  };
  
  // Pin configuration
  i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_MCLK,
    .bck_io_num = I2S_SCLK,
    .ws_io_num = I2S_LRCK,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  
  esp_err_t err = i2s_driver_install(i2s_port, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Failed to install I2S driver for playback: %d\n", err);
    return;
  }
  
  err = i2s_set_pin(i2s_port, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Failed to set I2S pins: %d\n", err);
    return;
  }
  
  i2sInstalled = true;
  Serial.printf("I2S playback init: MCLK=%d, BCLK=%d, LRCK=%d, DOUT=%d\n", I2S_MCLK, I2S_SCLK, I2S_LRCK, I2S_DOUT);
}

// Initialize I2S for recording (microphone input)
void initI2SRecording() {
  // Stop any existing I2S if it was installed
  if (i2sInstalled) {
    i2s_driver_uninstall(i2s_port);
    i2sInstalled = false;
  }
  
  // I2S configuration for recording
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll = true,
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
  
  esp_err_t err = i2s_driver_install(i2s_port, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Failed to install I2S driver for recording: %d\n", err);
    return;
  }
  
  err = i2s_set_pin(i2s_port, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Failed to set I2S pins: %d\n", err);
    return;
  }
  
  i2sInstalled = true;
  Serial.printf("I2S record init: MCLK=%d, BCLK=%d, LRCK=%d, DIN=%d\n", I2S_MCLK, I2S_SCLK, I2S_LRCK, I2S_DIN);
}

// Generate and play a beep tone
void playBeep() {
  initI2SPlayback();
  
  // Generate sine wave for beep (interleaved stereo)
  int frames = SAMPLE_RATE / 10;  // 100ms worth of frames
  int16_t* beepBuffer = (int16_t*)malloc(frames * CHANNELS * sizeof(int16_t));
  
  if (beepBuffer == NULL) {
    Serial.println("ERROR: Failed to allocate beep buffer!");
    return;
  }
  
  // Generate sine wave with higher amplitude
  for (int i = 0; i < frames; i++) {
    float angle = 2.0f * (float)M_PI * (float)BEEP_FREQUENCY * (float)i / (float)SAMPLE_RATE;
    int16_t s = (int16_t)(30000.0f * sinf(angle));  // near full-scale
    // interleave L,R
    beepBuffer[i * 2 + 0] = s;
    beepBuffer[i * 2 + 1] = s;
  }
  
  // Play beep for BEEP_DURATION
  int repetitions = (BEEP_DURATION * SAMPLE_RATE) / (frames * 1000);
  size_t bytesWritten = 0;
  
  Serial.printf("Playing %d repetitions of %d frames stereo (%.1f Hz tone)...\n", 
                repetitions, frames, (float)BEEP_FREQUENCY);
  
  for (int rep = 0; rep < repetitions; rep++) {
    i2s_write(i2s_port, beepBuffer, frames * CHANNELS * sizeof(int16_t), &bytesWritten, portMAX_DELAY);
  }
  
  free(beepBuffer);
  Serial.printf("Beep played (%d bytes written per repetition)\n", bytesWritten);
}

// Record audio from microphone
void recordAudio() {
  if (!bufferValid) {
    Serial.println("ERROR: Audio buffer not allocated!");
    return;
  }
  
  initI2SRecording();
  
  size_t bytesRead = 0;
  size_t bytesTotal = 0;
  
  // Record for RECORD_TIME_SEC seconds
  while (bytesTotal < bufferSize) {
    size_t toRead = bufferSize - bytesTotal;
    esp_err_t err = i2s_read(i2s_port, ((uint8_t*)audioBuffer) + bytesTotal, toRead, &bytesRead, portMAX_DELAY);
    if (err != ESP_OK) {
      Serial.printf("ERROR: I2S read failed: %d\n", err);
      break;
    }
    bytesTotal += bytesRead;
  }
  
  Serial.printf("Recorded %d bytes (stereo interleaved)\n", (int)bytesTotal);
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
