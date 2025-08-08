
---

```markdown
# Pico2 Firmware

This guide walks you through installing dependencies, setting up the environment, and building the project.

## 📦 Required Packages

Install the following packages for Ubuntu:

```bash
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential git
```

Optional but recommended:

```bash
sudo apt install python3 python3-pip
```

---

## 📁 Folder Setup

Create a working folder and clone the necessary repositories:

```bash
# Clone the Pico SDK
git clone -b master https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init

```

---

## 🔧 Environment Setup

Before building, set the following environment variables:

```bash
export PICO_SDK_PATH=~/pico-sdk
```

To make this permanent, you can add them to your `~/.bashrc` or `~/.zshrc`.

---

## 🛠️ Building the Project

Navigate to the project folder and build:

## Pico2 firmware (RP2350) — build and flash

This folder builds two UF2s for Raspberry Pi Pico 2 (RP2350):
- rfm69_sender.uf2 — transmits test messages
- rfm69_receiver.uf2 — listens and prints received messages

Project structure:
- CMakeLists.txt creates a small common library (rfm69_lib) and two apps
- common/rfm69.[ch] talks to RFM69 over SPI
- sender/main_sender.c and receiver/main_receiver.c are the apps

Pin mapping (GPIO on Pico2):
- SPI0 SCK 18, MOSI 19, MISO 16, CS 17
- RESET 20, DIO0 21, onboard LED 25

Prerequisites
- macOS (Homebrew):
	- brew install cmake ninja git
	- Install an ARM GCC toolchain providing arm-none-eabi-gcc. For Homebrew this is often arm-none-eabi-gcc. If your brew doesn’t have it, install Arm’s toolchain from Arm or use xPack GCC.
- Ubuntu/Debian:
	- sudo apt update && sudo apt install cmake ninja-build gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential git

Get the Pico SDK (choose one)
1) Clone SDK yourself (recommended for speed/offline):
	 - git clone -b master https://github.com/raspberrypi/pico-sdk.git
	 - cd pico-sdk && git submodule update --init && cd -
	 - export PICO_SDK_PATH="$(pwd)/pico-sdk"
2) Or let CMake auto-fetch it on first configure:
	 - export PICO_SDK_FETCH_FROM_GIT=1

Build (Pico2 target)
- From this folder (boards/pico2):

```bash
# Pick ONE of these depending on your SDK choice above
export PICO_SDK_PATH=/absolute/path/to/pico-sdk    # option 1
# export PICO_SDK_FETCH_FROM_GIT=1                 # option 2

# Target the Pico 2 board
export PICO_BOARD=pico2

# Configure + build
cmake -S . -B build -G Ninja
cmake --build build -j
```

Outputs
- build/rfm69_sender.uf2
- build/rfm69_receiver.uf2

Flash to the board
1) Hold BOOTSEL, plug in the Pico 2 via USB; a drive (RPI-RP2) appears.
2) Drag-and-drop the desired .uf2 onto the drive.
3) Board reboots and runs your program.

See logs over USB
- Both apps enable stdio over USB. After flashing, a USB serial device appears.
- On macOS you can use, e.g.:

```bash
# Replace with your actual device path
screen /dev/tty.usbmodem* 115200
```

Troubleshooting
- “arm-none-eabi-gcc not found”: install an ARM GCC toolchain and ensure arm-none-eabi-gcc is on PATH.
- “SDK location was not specified”: set PICO_SDK_PATH or set PICO_SDK_FETCH_FROM_GIT=1 before configuring.
- Wrong board selected: ensure export PICO_BOARD=pico2 before running the first cmake configure.
- No serial output: give it a few seconds after power-up; ensure USB stdio isn’t disabled and try another cable/port.

Notes on how it’s written
- CMake links pico_stdlib, hardware_spi, hardware_gpio and defines two apps.
- rfm69_init configures RFM69 for 433 MHz, variable-length packets, CRC, addressing, and maps DIO0 for PacketSent/PayloadReady.
- Sender uses rfm69_send_with_ack for unicast reliability (ACK/retry, simple CSMA) and rfm69_send_packet_addressed for broadcasts.
- Receiver filters node+broadcast addresses and auto-ACKs unicast frames that request it.

License
MIT