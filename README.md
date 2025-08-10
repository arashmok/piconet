# Raspberry Pi Pico 2 W Development in VS Code (Proxmox VM)

Set up a full dev workflow for the Pico 2 W inside a Proxmox VM, using the Pico SDK, picotool, and the VS Code Pico extension. Includes USB passthrough, build/flash, and serial logging.

---

## 1) Proxmox USB passthrough (host → VM)

**On the Proxmox host:**
- Put the board in **BOOTSEL**: hold **BOOTSEL** while plugging in USB.
- Check it's seen:
  ```bash
  lsusb
  ```

**Attach to the VM:**
- Proxmox UI → VM → Hardware → Add → USB Device

## 2) VM setup (Ubuntu/Debian)

**Install tools:**
```bash
sudo apt update
sudo apt install -y git cmake ninja-build build-essential python3-pip picocom
```

**USB permissions for picotool (no sudo):**
```bash
sudo tee /etc/udev/rules.d/99-pico.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="2e8a", MODE="0666", GROUP="plugdev", TAG+="uaccess"
KERNEL=="hidraw*", ATTRS{idVendor}=="2e8a", MODE="0666", GROUP="plugdev", TAG+="uaccess"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG plugdev,dialout $USER
# log out/in so groups apply
```

## 3) VS Code

**Install extensions:**
- Raspberry Pi Pico
- CMake Tools
- C/C++ (MS)

**Optional setting:**
```json
"cmake.generator": "Ninja"
```

## 4) Example project (USB serial "hello")

Check the Hello_World folder!

## 5) Build, flash, and run (two ways)

### A) Terminal

#### Clean build
```bash
rm -rf build
cmake -S . -B build -G Ninja -DPICO_BOARD=pico2_w -DPICO_PLATFORM=rp2350-arm-s
cmake --build build -j
```

#### Put board in BOOTSEL, then flash and reboot
```bash
picotool load build/blank.elf -fx
```

#### Open the serial log (requires picocom)

**Find the port:**
```bash
dmesg | grep -i ttyACM
```

**Connect (115200 baud is typical):**
```bash
picocom -b 115200 /dev/ttyACM0
```

**How to exit picocom:**
- Press **Ctrl-A**, then **X** → quit cleanly
- (If keys echo weirdly, press **Ctrl-A**, then **E** to toggle local echo.)

### B) VS Code (Pico extension)

- Click **Run Project** (status bar) → it builds and flashes using picotool.
- Then open a terminal and run `picocom -b 115200 /dev/ttyACM0` to view logs.
- **Tip:** you can add a VS Code task to launch picocom automatically after flash if you like.

---

## 6) Tips & tricks

- Pass both USB IDs to the VM (BOOTSEL `2e8a:000f` + runtime `2e8a:xxxx`) so the board stays attached after reboot.
- **-fx vs -f with picotool:**
  - `-fx` = flash then reboot now.
  - `-f` = flash only; later run `picotool reboot`.
- If the board falls back to BOOTSEL after flashing, the app likely crashed or was linked wrong. Try the SDK's hello_usb and verify `PICO_BOARD=pico2_w`, `PICO_PLATFORM=rp2350-arm-s`.
- **USB picky?** A cheap USB 2.0 hub between host and board often helps with enumeration on VMs.

---

## 7) Common errors

| Error | Meaning | Fix |
|-------|---------|-----|
| `spawn arm-none-eabi-gcc ENOENT` | Compiler not on PATH | Use full paths in CMake kit, or add toolchain bin to PATH |
| `CMake was unable to find Ninja` | Ninja missing | `sudo apt install ninja-build` or use "Unix Makefiles" |
| `No accessible RP-series devices` | Permissions | Add udev rules, groups, replug in BOOTSEL |
| `picoboot::connection_error` after flash | Device rebooted mid-connection | Add both USB IDs to VM, or use `-f` then `picotool reboot` |

---

## 8) Handy commands

```bash
# Show device/app info (BOOTSEL)
picotool info -a

# Reboot from BOOTSEL into your app
picotool reboot

# Reboot into BOOTSEL from running app (if USB stdio enabled)
picotool reboot -u
```

---

## 9) Links

- **Pico SDK:** https://github.com/raspberrypi/pico-sdk
- **Pico Examples:** https://github.com/raspberrypi/pico-examples
- **Picotool:** https://github.com/raspberrypi/picotool
