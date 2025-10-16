# Plush Toy Security Device (PTSD)
Plush Toy Security Device
# Plush Toy Security Device (PTSD) - Plush Toy Protection & Interaction System

Welcome to the Plush Toy Security Device (PTSD) project—a playful yet professionally engineered solution to protect our beloved plush penguin, Kevin Guttormsen, from unauthorized capture and add interactive features to his personality.

## Project Overview

The Plush Toy Security Device (PTSD) is an embedded system designed to fit inside a plush toy, leveraging an ESP32-S3 development board with built-in microphone and speakers. The system features four distinct operation modes (Armed, Social, Sleeping and Kidnapped) with intelligent accelerometer power management for energy efficiency. It detects unauthorized movement, responds to pressure pushbuttons placed around Kevin's body, and interacts through sound and AI-driven responses.

## Operation Modes

- **Armed Mode:** Monitors for unauthorized movement and triggers security alerts when Kevin is unexpectedly handled.
- **Social Mode:** Expects playful interaction and responds with friendly sounds and behaviors to encourage play.
- **Sleeping Mode:** Battery-saving mode with accelerometer deactivated to conserve power during inactivity.
- **Kidnapped Mode:** Emergency mode activated when Kevin detects movement in armed mode. Creates WiFi hotspot "JegErPingnappa" (password: "Kevin") and plays escalating alarm sequence from SD card SecurityResponse folder. Web interface at 192.168.1.1 offers "Activate Alarm" (immediate loud alarm) and "Rescue Kevin & Reset to Social Mode" (requires password "Kevin").

## AI Features & Speech Recognition

### Wake Word Detection
- **Custom Wake Word:** "Hei Kevin" (Norwegian) - trained specifically for Kevin's personality
- **Technology:** Hybrid approach using ESP-SR WakeNet (training) + Arduino-compatible libraries (runtime)
- **Activation:** Puts Kevin into command-listening mode when detected

### Voice Commands (Post Wake-Word)
- **"Syng en sang"** → Plays random WAV file from `/songs/` directory on SD card
- **"God natt"** → Switches Kevin to Sleeping mode for power conservation
- **"Fortell en vits"** → Plays random WAV file from `/jokes/` directory on SD card
- **Technology:** Arduino-compatible keyword spotting libraries (Snowboy, Edge Impulse, or Picovoice)
- **Language:** Norwegian support through compatible libraries

### Implementation Requirements
- **Framework:** Arduino IDE (primary) with ESP-IDF (wake word training only)
- **Wake Word Training:** One-time ESP-IDF process to create custom "Hei Kevin" model
- **Runtime Libraries:** Arduino-compatible speech recognition libraries
- **Audio Storage:** SD card with organized WAV file directories
- **Memory:** ~50-200KB for wake word model + library overhead

## Features

- **Movement Detection:** Advanced accelerometer-based detection with configurable thresholds for movement and free-fall events.
- **Intelligent Power Management:** Accelerometer automatically activates/deactivates based on current mode to extend battery life.
- **Mode Switching:** Easy cycling between operation modes via button press or other triggers.
- **Interactive Pushbuttons:** Multiple pressure sensors allow Kevin to respond to touch and interaction.
- **Audio Interaction:** Kevin can play sounds, respond to voice commands, and express emotions through the built-in speaker.
- **Modular Software Architecture:** Clean, maintainable code structure for easy updates and feature expansion.
- **Expandable Functionality:** Designed for easy addition of new sensors and interactive features.

## Hardware

- **ESP32-S3 AI Development Board** (with microphone and speakers)
- **MPU-6050 Accelerometer** (for movement detection, connected via I2C)
- **5 Pressure Pushbuttons** (left leg, right leg, left wing, right wing, beak - all pull-up inputs connected to GND)
- **Power Supply** (battery and USB-power brick)

## Software

- **Primary Framework:** Arduino IDE with C++ for all development
- **AI Features:** Hybrid approach - ESP-IDF for wake word training, Arduino libraries for runtime
- **Wake Word Libraries:** Snowboy, Edge Impulse, or Picovoice (Arduino-compatible)
- **Audio Playback:** ESP8266Audio library for WAV file playback via I2S
- **Modular Architecture:** Clean, maintainable code structure for easy updates and feature expansion

## Getting Started

1. **Clone the repository**
2. **Open the project in Arduino IDE**
3. **Connect your ESP32-S3 board**
4. **Upload the firmware and start protecting Kevin!**

## Project Progress

- **Accelerometer Selected:** MPU-6050 module chosen for movement, free-fall, and crash detection. Wired via I2C (GPIO10=SCL, GPIO11=SDA).
- **Arduino IDE Setup:** Sketch folder `kevin` with starter `kevin.ino` created. Includes MPU-6050 initialization and serial output.
- **Audio System Research:**
	- SD card access via built-in `SD_MMC` library (SDMMC interface)
	- WAV file playback and speaker output using `ESP8266Audio` library (`AudioFileSourceSD`, `AudioGeneratorWAV`, `AudioOutputI2S`)
	- I2S pins used for speaker communication

## On-Device Wake Word Recording System

### Recording Trigger
- **Activation:** Hold left leg button for 2 seconds
- **Audio Cue:** Kevin plays "pling" sound to indicate recording start
- **Duration:** Records for 2.5 seconds through built-in microphones
- **Storage:** Automatic WAV file storage in `/wakeword_samples/` directory on SD card

### Recording Process
1. **Hold left leg button** for 2+ seconds
2. **Kevin signals** with "pling" sound and "Start saying 'Hei Kevin'..." message
3. **2.5 second recording** captures audio through ESP32-S3 microphones
4. **Automatic saving** as numbered WAV files (sample_001.wav, sample_002.wav, etc.)
5. **Confirmation** message when recording completes

### Benefits of On-Device Recording
- **Authentic audio environment** - captures real device acoustics and enclosure effects
- **Easy data collection** - record from multiple people without external equipment
- **Better training data** - includes actual usage distance and environmental factors
- **Simplified workflow** - no need to transfer files between devices

### Training Data Collection Workflow
1. **Collect samples:** Use Kevin to record 1000+ samples from different speakers
2. **Transfer files:** Copy WAV files from SD card to computer
3. **ESP-IDF training:** Use collected samples to train custom "Hei Kevin" model
4. **Model deployment:** Flash trained model back to Kevin for runtime use

## AI Implementation Roadmap (Arduino IDE Compatible)

### Phase 1: Current Arduino Code Enhancement
- [x] Basic button and accelerometer functionality implemented
- [x] On-device wake word recording system implemented
- [ ] Add SD card support for audio file storage
- [ ] Implement random audio file selection from directories
- [ ] Integrate ESP8266Audio library for WAV playback

### Phase 2: Wake Word Development (One-time ESP-IDF Process)
- [ ] Install ESP-IDF development environment (separate from Arduino)
- [ ] Record 1000+ samples of "Hei Kevin" using on-device recording system
- [ ] Train custom wake word model using ESP-SR WakeNet tools
- [ ] Export trained model for Arduino use
- [ ] Test wake word detection accuracy

### Phase 3: Arduino AI Integration
- [ ] Choose Arduino-compatible wake word library (Snowboy recommended)
- [ ] Install wake word library via Arduino Library Manager
- [ ] Integrate custom "Hei Kevin" model into Arduino sketch
- [ ] Implement wake word detection in main loop
- [ ] Add command listening mode after wake word detection

### Phase 4: Command Recognition Setup
- [ ] Research Arduino-compatible keyword spotting libraries
- [ ] Implement keyword detection for: "syng", "god natt", "fortell"
- [ ] Add command timeout and fallback handling
- [ ] Test Norwegian language compatibility

### Phase 5: Audio Response System
- [ ] Create SD card directory structure: `/songs/`, `/jokes/`, `/wakeword_samples/`
- [ ] Implement random file selection algorithms
- [ ] Add audio playback interruption for new commands
- [ ] Test audio quality and synchronization

### Phase 6: System Integration & Testing
- [ ] Combine wake word, commands, and audio playback
- [ ] Implement mode transitions via voice commands
- [ ] Add visual/audio feedback for command confirmation
- [ ] Comprehensive testing with Norwegian speakers
- [ ] Performance optimization and battery life testing

### Phase 7: Advanced Features (Future)
- [ ] Gesture recognition using accelerometer patterns
- [ ] Activity learning and adaptive behavior
- [ ] Voice activity detection for battery optimization
- [ ] Multi-language support expansion

## Required Tools & Dependencies
- **Arduino IDE** with ESP32 board support
- **ESP-IDF v5.0+** (for wake word training only)
- **Arduino Libraries:** ESP8266Audio, Snowboy (or alternative wake word library)
- **Python 3.8+** for model training scripts (optional)
- **Audio recording software** for wake word dataset
- **WAV audio files** for songs and jokes (16-bit, 16kHz recommended)
- **SD card** formatted as FAT32
- **Arduino-Compatible Wake Word Options:**
  - Snowboy (free, open-source)
  - Edge Impulse (free tier available)
  - Picovoice (commercial, most accurate)

Next steps: Integrate audio playback, add movement detection logic, and expand interactive features.

## License

MIT License

---

*Protect Kevin. Make him interactive. Have fun!*

## Documentation

- [ESP32-S3 Board Features & Pinout](docs/ESP32-S3-Board.md)
