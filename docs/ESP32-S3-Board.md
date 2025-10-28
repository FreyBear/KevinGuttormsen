# DeepSeek AI Voice Chat ESP32-S3 AUDIO Board

## Product Features
- **DeepSeek AI smart speaker development board** based on ESP32-S3
- Supports **2.4GHz Wi-Fi** (802.11 b/g/n) and **Bluetooth 5 (LE)**
- Integrates large-capacity **Flash** and **PSRAM**
- **Dual-microphone array**, onboard speaker with amplifier
- **RGB surround lighting** (7 programmable ring lights)
- Multiple extension interfaces for rapid development

## Arduino IDE Configuration (REQUIRED)

To use this board with Arduino IDE, you **must** configure these settings:

### Critical Settings:
- **Board:** ESP32S3 Dev Module
- **USB CDC On Boot:** `Enabled` ⚠️ **REQUIRED**
- **USB DFU On Boot:** `Enabled` ⚠️ **REQUIRED**
- **Upload Mode:** `UART0 / Hardware CDC`
- **USB Mode:** `Hardware CDC and JTAG` (or `USB-OTG (TinyUSB)`)

### Recommended Settings:
- **CPU Frequency:** `240MHz (WiFi)`
- **Flash Mode:** `QIO 80MHz`
- **Flash Size:** `16MB (128Mb)`
- **Partition Scheme:** `16M Flash (3MB APP/9.9MB FATFS)`
- **PSRAM:** `OPI PSRAM`
- **Upload Speed:** `921600` (or `115200` if issues occur)

### First Time Setup:
1. Install ESP32 board support via Board Manager
2. Set the above configuration
3. Upload your sketch
4. Open Serial Monitor (115200 baud)
5. Press RESET button on board to see output

## Technical Specifications
- **Controller:** ESP32-S3R8, Xtensa 32-bit LX7 dual-core, up to 240MHz
- **Wireless:** 2.4GHz Wi-Fi, Bluetooth 5 (LE), onboard antenna
- **Storage:** 512KB SRAM, 384KB ROM, 8MB PSRAM, 16MB Flash
- **Voice:** Dual digital microphones, noise reduction, echo cancellation, near/far-field wake-up
- **RTC:** PCF85063 RTC chip, supports power-down retention, alarm/timed tasks
- **Lighting:** 7 RGB ring lights, fully programmable
- **Interaction:** Multiple buttons, battery switches for custom functions

## Extension Interfaces
- **SPI LCD screen interface** (FPC / pin header)
- **DVP camera interface** (24-pin header)
- **USB, I2C, and IO pinouts** (compatible with screen interface)
- **Audio decoding chip**, MIC and speaker interface
- **Micro SD card slot** for audio file storage
- **Battery charging management**, multiple power modes

## Audio Features
- High-quality audio input/output module
- Supports offline voice model and shortcut commands

## Available GPIO Pins for External Components

### General Purpose I/O:
- **GPIO3** to **GPIO11** - Available for buttons, sensors, etc.
- **GPIO19, GPIO20** - Additional GPIOs
- **EXIO1 to EXIO3** - Extended I/O pins

### Reserved/In-Use Pins (Do NOT use for other purposes):
- **GPIO10, GPIO11** - I2C (SCL, SDA) for accelerometer or other I2C devices
- **GPIO12-16** - I2S Audio (MCLK, SCLK, LRCK, DOUT, DIN)
- **GPIO40-42** - SD Card (CLK, CMD, DATA)
- **EXIO9** - PA_EN (Speaker amplifier enable)

### Recommended Pin Assignments for Kevin Project:
- **GPIO3** - Left leg button (INPUT_PULLUP)
- **GPIO4** - Right leg button (INPUT_PULLUP)
- **GPIO5** - Left wing button (INPUT_PULLUP)
- **GPIO6** - Right wing button (INPUT_PULLUP)
- **GPIO7** - Beak button (INPUT_PULLUP)
- **GPIO0** - Mode switching button (INPUT_PULLUP) (or use BOOT button)
- **GPIO10, GPIO11** - MPU-6050 accelerometer (I2C_SCL, I2C_SDA)

### Power Pins:
- **3.3V** - Logic level power output
- **5V** - USB power output
- **GND** - Ground

## SD Card Interface (SDMMC)

### Pin Configuration:
- **GPIO42:** SDMMC_CMD (Command line)
- **GPIO40:** SDMMC_CLK (Clock)
- **GPIO41:** SDMMC_DATA (Data line)
- **EXIO4:** SD_CD_PIN (Card detect - optional)

### Arduino Usage:
```cpp
#include <SD_MMC.h>

void setup() {
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    return;
  }
  Serial.println("SD Card Mounted Successfully");
}
```

## I2S Audio Interface (Microphones & Speaker)

### Pin Configuration:
- **GPIO12:** I2S_MCLK (Master Clock)
- **GPIO13:** I2S_SCLK (Serial Clock / Bit Clock)
- **GPIO14:** I2S_LRCK (Left-Right Clock / Word Select)
- **GPIO15:** I2S_DOUT (Data Out to Speaker)
- **GPIO16:** I2S_DIN (Data In from Microphones)
- **EXIO9:** PA_EN (Power Amplifier Enable)

### Audio Specifications:
- **Dual Digital Microphones** - Stereo input via I2S
- **Built-in Speaker** - Mono output via I2S with amplifier
- **Recommended Sample Rate:** 16kHz (voice) or 44.1kHz (music)
- **Bits Per Sample:** 16-bit

### Arduino I2S Configuration Example:
```cpp
#include <driver/i2s.h>

#define I2S_MCLK 12
#define I2S_SCLK 13
#define I2S_LRCK 14
#define I2S_DOUT 15
#define I2S_DIN  16

i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = 16000,
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
  .data_in_num = I2S_PIN_NO_CHANGE  // or I2S_DIN for recording
};

i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
i2s_set_pin(I2S_NUM_0, &pin_config);
```

## I2C Interface

### Pin Configuration:
- **GPIO10:** I2C_SCL (Clock)
- **GPIO11:** I2C_SDA (Data)

### Arduino Usage (for MPU-6050 Accelerometer):
```cpp
#include <Wire.h>
#include <Adafruit_MPU6050.h>

Wire.begin(10, 11);  // SDA=11, SCL=10
```

## RGB Ring Lights

- **7 programmable RGB LEDs** arranged in a ring
- **GPIO38:** LED data control pin (WS2812B addressable LEDs)
- **Color Order:** GRB (Green, Red, Blue) - NOT RGB
  - When setting color 0xFF0000 (red), the LED displays as green
  - When setting color 0x00FF00 (green), the LED displays as red
  - Use `NEO_GRB + NEO_KHZ800` in Adafruit_NeoPixel initialization

### Arduino Usage Example:
```cpp
#include <Adafruit_NeoPixel.h>

#define LED_PIN 38
#define NUM_LEDS 7

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.setBrightness(100);
  
  // Set LED 0 to red (appears as green due to GRB order)
  strip.setPixelColor(0, 0xFF0000);
  strip.show();
}
```

## Onboard Buttons

- **BOOT Button** - Can be used for mode switching or entering bootloader
- **RESET Button** - Hardware reset

## Power Management

- **USB-C Power Input** - 5V
- **Battery Connector** - For portable operation
- **Charging Circuit** - Built-in battery management

---

## Additional Resources

- [ESP32-S3 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- [ESP32-S3 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
- [Arduino ESP32 Documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/)
- [ESP-SR Speech Recognition Framework](https://github.com/espressif/esp-sr)
