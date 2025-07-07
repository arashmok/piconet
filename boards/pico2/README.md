
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

```bash
export PICO_BOARD=pico2
mkdir build && cd build
cmake ..
make -j4
```

If everything is set up correctly, this will generate a `.uf2` file you can flash to the board.

---

## 🚀 Flashing to the Board

1. Plug your `pico2` board into your computer while holding the **BOOTSEL** button.
2. It will appear as a USB mass storage device.
3. Copy the generated `.uf2` file to it.

---

## 💬 Troubleshooting

- Make sure you’ve exported `PICO_SDK_PATH` before running `cmake`.
- If `cmake ..` fails, verify your Pico SDK path and that the SDK was cloned with submodules.
- If your board is custom (`pico2`), ensure it is properly defined in your CMake project.

---

## 🧩 Custom Board Notes

If you're targeting a custom board (`pico2`), make sure you have a matching `board` configuration file in your project, or that your `CMakeLists.txt` and `CMake` toolchain knows how to handle it.

---

## 📄 License

MIT
```