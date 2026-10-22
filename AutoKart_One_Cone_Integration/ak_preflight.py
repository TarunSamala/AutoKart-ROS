#!/usr/bin/env python3
"""AutoKart laptop preflight. Pure diagnostics; never writes to an actuator."""
import glob
import os
import shutil
import subprocess
import sys
from pathlib import Path

CAMERA_ID = '3482:6723'
ESP_ID = '303a:1001'

def run(*cmd):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=8, check=False)
        return p.stdout.strip() or p.stderr.strip() or '(no output)'
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return f'(unavailable: {e})'

def tty_usb_ids(port):
    props = run('udevadm', 'info', '--query=property', f'--name={port}')
    v = None
    product = None
    for line in props.splitlines():
        if line.startswith('ID_VENDOR_ID='):
            v = line.split('=',1)[1].lower()
        elif line.startswith('ID_MODEL_ID='):
            product = line.split('=',1)[1].lower()
    return (v,product)

def main():
    print('=== AUTOKART USB PREFLIGHT (NO MOTOR COMMANDS) ===')
    print('USB enumeration:')
    devices = run('lsusb')
    print(devices)
    print('HP60C USB detected:', CAMERA_ID in devices)
    print('ESP32-S3 USB detected:', ESP_ID in devices)
    print('\nSerial ports:')
    ports = sorted(set(glob.glob('/dev/ttyACM*')+glob.glob('/dev/ttyUSB*')))
    for port in ports:
        vid, pid = tty_usb_ids(port)
        print(f'  {port}: VID:PID={vid}:{pid}' + (' <ESP32-S3 candidate>' if (vid,pid) == ('303a','1001') else ''))
    if not ports: print('  No serial ports. Check dmesg -T and USB interface permissions.')
    print('\nStable serial port names:')
    ids = sorted(glob.glob('/dev/serial/by-id/*'))
    for p in ids: print(f'  {p} -> {os.path.realpath(p)}')
    if not ids: print('  No /dev/serial/by-id entries.')
    print('\nVideo nodes:')
    print('  ' + (' '.join(sorted(glob.glob('/dev/video*'))) or '(none)'))
    if shutil.which('v4l2-ctl'):
        print('\nV4L2 camera enumeration:\n' + run('v4l2-ctl','--list-devices'))
    else:
        print('\nTo enumerate video devices: sudo apt install v4l-utils')
    print('\nNOTE: Camera USB enumeration does not prove a working DEPTH stream.')
    print('NOTE: No firmware flashed, no commands transmitted, no GPIO actuated.')
    return 0 if CAMERA_ID in devices and ESP_ID in devices else 2

if __name__ == '__main__':
    sys.exit(main())
