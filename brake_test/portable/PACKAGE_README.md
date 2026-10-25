# AutoKart Brake Test portable package

Qualified targets:

- Ubuntu 22.04 x86_64 laptop/workstation
- Ubuntu 22.04 64-bit (`aarch64`) on Raspberry Pi 4B
- 64-bit Raspberry Pi OS 12 on Raspberry Pi 4B

Raspberry Pi OS 11 is not supported because its default Python 3.9 has no
compatible wheel for the pinned ONNX Runtime. The recommended Pi target is
64-bit Ubuntu 22.04.

The archive contains the HP60C SDK libraries for both targets, the native
camera bridge source, the pinned YOLO26s model, the blue-cone dashboard, and a
first-run installer. It also contains the matching 0.40 V ESP32-S3 files under
`esp32/`; flash those once as described in `esp32/README.md`.

## First launch

1. Extract the archive to a permanent folder. Do not run it from inside the
   archive viewer.
   Keep the accompanying `.sha256` file and verify it before extraction with
   `sha256sum -c AutoKart-Brake-Test-Ubuntu22.04-Pi4B.tar.gz.sha256`.
2. Connect the HP60C and ESP32 by USB. The dashboard auto-discovers the
   AutoKart serial protocol; normal internet Wi-Fi may remain connected.
3. Double-click **AutoKart Brake Test.desktop**, choose **Trust and Launch** if
   Ubuntu asks, and enter the sudo password for the one-time dependency and
   USB-rule installation.
4. Unplug and reconnect the HP60C after the first installation, then launch the
   icon again.
5. Keep the kart jacked, restrained, and the physical power cut within reach.
   Choose a stop gap, then click **START**. START commands Forward at 0.40 V.

The default blue-cone bumper gap is 1.20 m. The UI permits 0.40, 0.70, 1.00,
or 1.20 m only, and permits changing it only while the wheel is locked.

The dashboard may be opened while the ESP32 is offline. It remains running,
shows **ESP32 OFFLINE - RETRYING**, and keeps START disabled until the ESP32,
GP8630N, AS5047P, camera, and locked zero-output state are verified. If the
	controller serial link drops after it has connected, the dashboard disables all
motion controls and issues a verified full-brake command as soon as the link
returns.

The ESP32 control channel is USB serial and does not compete with internet
Wi-Fi. The old `192.168.4.1` controller link is not used by this package.

Important: this bench firmware deliberately has its control-link timeout
disabled. Keeping the dashboard alive prevents a host application crash, but
cannot transmit the brake command while USB is physically disconnected. Never
unplug USB during a motor run; retain the independent physical power cut.

The ESP32 must run the matching AutoKart firmware where `TEST_START` is fixed
to 0.40 V and the blue-cone no-heartbeat bench profile is installed. Closing
the launcher or dashboard attempts a serial `LOCK`, but the operator remains
responsible for the independent physical power cut.

Logs are written under `.brake-test-runtime/logs/` inside the extracted folder.

## Raspberry Pi 4B router dashboard

Use a 64-bit Ubuntu 22.04 or Raspberry Pi OS 12 image. Connect the HP60C and
ESP32-S3 directly to the Pi by USB, and connect the Pi to the dedicated TP-Link
router. The ESP32 control channel remains USB serial; the router carries only
the browser dashboard traffic.

After first installation, reboot the Pi once so its `dialout` membership is
active. Launch the application on the Pi and use the printed LAN address from
any phone or laptop on the same router, for example:

```text
http://192.168.0.100:8781/
```

The dashboard server listens on all Pi interfaces. If a firewall is enabled,
allow inbound TCP port 8781. Disable wireless client/AP isolation on the router,
and reserve the Pi's DHCP address so the URL remains stable.

Anyone who can reach this private URL can press START or STOP. Use a dedicated,
WPA2-protected test router with no untrusted clients, keep the kart jacked and
restrained, and retain the independent physical power cut. Do not expose port
8781 to the internet or configure router port forwarding.
