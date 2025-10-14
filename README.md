# KevinGuttormsen
Plush Toy protection and interaction system
# KevinGuttormsen Plush Toy Protection & Interaction System

Welcome to the KevinGuttormsen project—a playful yet professionally engineered solution to protect our beloved plush penguin, Kevin Guttormsen, from unauthorized capture and add interactive features to his personality.

## Project Overview

KevinGuttormsen is an embedded system designed to fit inside a plush toy, leveraging an ESP32-S3 development board with built-in microphone and speakers. The system detects unauthorized movement, responds to pressure pushbuttons placed around Kevin’s body, and interacts with the world through sound and AI-driven responses.

## Features

- **Movement Detection:** Alerts and reacts when Kevin is moved without authorization.
- **Interactive Pushbuttons:** Multiple pressure sensors allow Kevin to respond to touch and interaction.
- **Audio Interaction:** Kevin can play sounds, respond to voice commands, and express emotions.
- **Expandable Functionality:** Designed for easy addition of new sensors and interactive features.

## Hardware

- **ESP32-S3 AI Development Board** (with microphone and speakers)
- **Pressure Pushbuttons** (strategically placed around the plush toy)
- **Power Supply** (battery and USB-power brick)

## Software

- Developed using the **Arduino IDE**
- Written in **C++** for embedded systems
- Modular codebase for easy maintenance and feature expansion

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

Next steps: Integrate audio playback, add movement detection logic, and expand interactive features.

## License

MIT License

---

*Protect Kevin. Make him interactive. Have fun!*

## Documentation

- [ESP32-S3 Board Features & Pinout](docs/ESP32-S3-Board.md)
