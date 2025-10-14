# ESP32-S3 AI Smart Speaker Development Board

## Product Features
- **AI smart speaker development board** based on ESP32-S3
- Supports **2.4GHz Wi-Fi** (802.11 b/g/n) and **Bluetooth 5 (LE)**
- Integrates large-capacity **Flash** and **PSRAM**
- **Dual-microphone array**, onboard speaker
- **RGB surround lighting** (7 programmable ring lights)
- Multiple extension interfaces for rapid development

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

## Available GPIO Pins
- GPIO19
- GPIO20
- GPIO3 to GPIO11
- EXIO1 to EXIO3
- 3.3V, 5V, GND

## SD Card Pinout
- GPIO42: SDMMC_CMD
- GPIO40: SDMMC_CLK
- GPIO41: SDMMC_DATA
- EXIO4: SD_CD_PIN

## Sound Interface Pinout
- GPIO10: I2C_SCL
- GPIO11: I2C_SDA
- GPIO12: I2S_MCLK
- GPIO13: I2S_SCLK
- GPIO14: I2S_LRCK
- GPIO15: I2S_DOUT
- GPIO16: I2S_DIN
- EXIO9: PA_EN

---

For more details, refer to the official ESP32-S3 documentation and board schematics.