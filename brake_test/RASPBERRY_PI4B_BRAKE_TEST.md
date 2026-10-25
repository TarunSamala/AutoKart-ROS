# AutoKart Brake Test — Raspberry Pi 4B Handoff

This document contains everything needed to transfer and run the AutoKart
blue-cone brake test on a Raspberry Pi 4B.

## System layout

```text
HP60C camera ──USB──┐
                    ├── Raspberry Pi 4B ──Ethernet/Wi-Fi── TP-Link WR850N
ESP32-S3 ──USB──────┘                                  ├── Laptop browser
                                                       └── Mobile browser
```

- The Pi runs camera perception, metric cone localization, the dashboard and
  ESP32 communication.
- The Pi communicates with the ESP32-S3 through USB serial, not
  `192.168.4.1`.
- Phones and laptops connected to the WR850N open the dashboard through the
  Pi's LAN address.
- A qualified blue cone inside the selected bumper gap commands full BLDC
  brake/lock.

## Supported Raspberry Pi operating systems

Use a **64-bit** operating system:

- Ubuntu 22.04 64-bit — recommended
- Raspberry Pi OS 12 64-bit

Check before installation:

```bash
uname -m
```

Required result:

```text
aarch64
```

Raspberry Pi OS 11 and 32-bit installations are not supported by this package.

## Package on the development laptop

Archive:

```text
/home/satvara/Github/AutoKart-ROS/autonomous_kart/dist/AutoKart-Brake-Test-Ubuntu22.04-Pi4B.tar.gz
```

Checksum file:

```text
/home/satvara/Github/AutoKart-ROS/autonomous_kart/dist/AutoKart-Brake-Test-Ubuntu22.04-Pi4B.tar.gz.sha256
```

Always verify with the accompanying `.sha256` file because the checksum changes
whenever the package is rebuilt.

## 1. Connect the Pi to the router

Connect the Raspberry Pi to the TP-Link WR850N using Ethernet when possible.
Wi-Fi also works.

On the Pi, find its address:

```bash
ip -o -4 addr show scope global
```

Example Pi address:

```text
192.168.0.100
```

Recommended router settings:

1. Reserve the Pi's DHCP address so it does not change.
2. Disable AP isolation/client isolation.
3. Use WPA2 and allow only trusted test devices.
4. Do not forward TCP port 8781 to the internet.

## 2. Transfer the package from the laptop

Replace the values below with the Pi's actual username and IP address:

```bash
PI_USER=pi
PI_IP=192.168.0.100

scp \
  /home/satvara/Github/AutoKart-ROS/autonomous_kart/dist/AutoKart-Brake-Test-Ubuntu22.04-Pi4B.tar.gz \
  /home/satvara/Github/AutoKart-ROS/autonomous_kart/dist/AutoKart-Brake-Test-Ubuntu22.04-Pi4B.tar.gz.sha256 \
  "$PI_USER@$PI_IP:/home/$PI_USER/"
```

For Ubuntu, the username may be `ubuntu` instead of `pi`.

## 3. Verify and extract on the Pi

Run on the Pi:

```bash
cd "$HOME"

sha256sum -c AutoKart-Brake-Test-Ubuntu22.04-Pi4B.tar.gz.sha256

tar -xzf AutoKart-Brake-Test-Ubuntu22.04-Pi4B.tar.gz

cd AutoKart-Brake-Test-Ubuntu22.04-Pi4B
```

The checksum command must report:

```text
AutoKart-Brake-Test-Ubuntu22.04-Pi4B.tar.gz: OK
```

## 4. First installation

The Pi needs internet access during the first installation for operating-system
packages and Python dependencies.

Run:

```bash
cd "$HOME/AutoKart-Brake-Test-Ubuntu22.04-Pi4B"
bash brake_test/portable/launch.sh
```

Enter the sudo password when requested. The installer will:

- install OpenCV, NumPy, SciPy, PyYAML and PySerial;
- install the ARM64 ONNX Runtime;
- verify the YOLO26s model checksum;
- compile the HP60C native camera bridge against the included ARM64 SDK;
- install HP60C USB permissions;
- install ESP32-S3 USB serial permissions;
- add the user to the `dialout` group;
- install desktop launchers.

After the first installation, reboot once:

```bash
sudo reboot
```

The reboot activates the new serial-port group membership.

## 5. Hardware connections

After reboot:

1. Connect the HP60C camera directly to a Raspberry Pi USB port.
2. Connect the ESP32-S3 USB-C port directly to another Pi USB port using a
   data-capable cable.
3. Keep the kart jacked, restrained and its drive wheels clear.
4. Keep the independent physical power cut accessible.

Check USB serial enumeration:

```bash
find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -print
```

Expected ESP32-S3 result is normally:

```text
/dev/ttyACM0
```

## 6. ESP32 firmware

The matching MicroPython files are included under the package's `esp32/`
directory.

Flash them once if the ESP32 does not already run the USB-serial brake-test
firmware:

```bash
cd "$HOME/AutoKart-Brake-Test-Ubuntu22.04-Pi4B/esp32"

PORT=/dev/ttyACM0

VENV="$HOME/AutoKart-Brake-Test-Ubuntu22.04-Pi4B/.brake-test-runtime/venv"

"$VENV/bin/python3" -m pip install mpremote

"$VENV/bin/python3" -m mpremote connect "$PORT" fs cp \
  bldc_bench_config.py :bldc_bench_config.py

"$VENV/bin/python3" -m mpremote connect "$PORT" fs cp \
  main.py :main.py

"$VENV/bin/python3" -m mpremote connect "$PORT" soft-reset
```

The firmware uses framed USB serial commands. The Pi dashboard automatically
discovers and validates the controller protocol before enabling START.

## 7. Start the dashboard

Run:

```bash
cd "$HOME/AutoKart-Brake-Test-Ubuntu22.04-Pi4B"
bash brake_test/portable/launch.sh
```

On a Pi desktop, the installed **AutoKart Brake Test** launcher may be used
instead.

The launcher prints addresses similar to:

```text
Dashboard addresses for phones/laptops on this router:
  http://192.168.0.100:8781/
```

## 8. Open from a laptop or mobile device

Connect the phone or laptop to the same TP-Link WR850N network and open:

```text
http://PI_IP:8781/
```

Example:

```text
http://192.168.0.100:8781/
```

The remote device does not need ROS, Python, camera drivers or ESP32 drivers.
It only needs a modern web browser.

## 9. Firewall

The dashboard listens on all Pi network interfaces at TCP port 8781.

If UFW is enabled:

```bash
sudo ufw allow 8781/tcp
sudo ufw status
```

Do not expose this port through router port forwarding.

## 10. Safe operating sequence

Before START, verify:

- kart is jacked and restrained;
- drive wheels are clear;
- physical power cut is accessible;
- ESP32 is connected by USB serial;
- HP60C camera is connected;
- dashboard reports the camera healthy;
- dashboard reports the ESP32 online;
- GP8630N is healthy;
- AS5047P reports `MAGNET OK`;
- controller reports `WHEEL LOCKED`;
- START is enabled only after all required checks pass.

Select a stop gap of 0.40, 0.70, 1.00 or 1.20 metres and press **START**.
START performs the fixed test sequence:

```text
zero encoder → arm → forward → 0.40 V
```

A qualified blue cone at or inside the selected bumper gap commands full brake
and wheel lock. Orange cones are detected and displayed but do not trigger the
brake.

## 11. Important network safety

There is currently no user login on the private dashboard. Anyone connected to
the router who can open the Pi URL can press START or STOP.

Therefore:

- use a dedicated test router;
- connect only trusted operator devices;
- do not expose the dashboard to the internet;
- do not configure port forwarding;
- retain the physical emergency power cut;
- do not use browser access as the only emergency-stop mechanism.

## 12. Troubleshooting

### Dashboard does not open

Check that it is listening:

```bash
ss -ltnp | grep 8781
```

Expected:

```text
0.0.0.0:8781
```

Check the Pi address and use that address from remote devices, not
`127.0.0.1`.

### ESP32 shows offline

```bash
find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -print
groups
```

If `dialout` is missing, run:

```bash
sudo usermod -aG dialout "$USER"
sudo reboot
```

Also verify the USB-C cable supports data.

### Camera shows offline

```bash
find /dev -maxdepth 1 -name 'video*' -print
```

Reconnect the HP60C after installation and ensure no other camera program is
using it.

### Remote device cannot connect

1. Confirm the phone/laptop and Pi are on the same router.
2. Disable AP/client isolation in the router.
3. Verify the Pi IP address.
4. Check TCP port 8781 in the Pi firewall.
5. Do not use `127.0.0.1` on the remote device.

### Logs

Application logs are stored under:

```text
$HOME/AutoKart-Brake-Test-Ubuntu22.04-Pi4B/.brake-test-runtime/logs/
```

Show the newest log:

```bash
ls -1t "$HOME/AutoKart-Brake-Test-Ubuntu22.04-Pi4B/.brake-test-runtime/logs" | head
```

## Validation boundary

The package structure, ARM64 vendor libraries, ARM64 inference wheels, serial
transport, LAN binding and archive checksum have been verified on the build
laptop. Final Raspberry Pi camera FPS, inference latency, USB stability and
physical braking distance must be measured on the actual Pi 4B and kart before
ground operation.
