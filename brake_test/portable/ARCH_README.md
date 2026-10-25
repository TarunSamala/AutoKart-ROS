# AutoKart Brake Test for the Arch development laptop

This package keeps the host on Arch Linux and runs the qualified Ubuntu 22.04
runtime inside Distrobox. It uses the existing `kart-ros2-humble` container or
creates it on first launch.

## First launch

1. Extract the archive to a permanent directory.
2. Double-click `AutoKart Brake Test.desktop` and select **Trust and Launch**.
3. If the Ubuntu runtime is not installed yet, remain connected to the internet
   while dependencies are installed. Reconnect the HP60C afterward.
4. Connect the ESP32-S3 to the laptop by USB.
5. Double-click the launcher again. The browser opens automatically.

The dashboard no longer exits when the ESP32 is temporarily unreachable. It
stays open, retries automatically, and leaves START disabled until safe live
telemetry is verified. The laptop may remain on normal internet Wi-Fi because
motor control uses framed USB serial, not the ESP32 access point. Do not unplug
USB during a motor run; retain the independent physical power cut.

The matching ESP32 firmware is under `esp32/`. Routine operation remains:
choose the stop gap, click START, and observe the verified blue-cone full-brake
response. Keep the kart jacked, restrained, and the physical power cut ready.
