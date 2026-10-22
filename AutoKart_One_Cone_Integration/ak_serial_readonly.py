#!/usr/bin/env python3
"""Read existing ESP32 output for a few seconds; NEVER send commands.

Use only with the BLDC power electrically isolated. Opening a USB serial
port may reset some ESP32 boards. This script does not flash firmware.
"""
import argparse
import glob
import os
import sys
import time

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--port', help='Use a /dev/serial/by-id path or /dev/ttyACM0')
    p.add_argument('--baud',type=int,default=115200)
    p.add_argument('--seconds',type=float,default=5.0)
    args=p.parse_args()
    try:
        import serial
    except ImportError:
        print('Install dependency: python3 -m pip install pyserial',file=sys.stderr)
        return 2
    ports=sorted(glob.glob('/dev/serial/by-id/*'))
    if not args.port:
        s3=[x for x in ports if 'Espressif' in x or '303a' in x]
        args.port=s3[0] if len(s3)==1 else None
    if not args.port:
        print('Provide --port /dev/ttyACM0 (verify using ak_preflight.py).',file=sys.stderr)
        return 2
    if not os.path.exists(args.port):
        print(f'Serial port not found: {args.port}',file=sys.stderr)
        return 2
    device=serial.Serial(port=None,baudrate=args.baud,timeout=0.2)
    device.dtr=False
    device.rts=False
    device.port=args.port
    try:
        device.open()
        print(f'Opened {args.port} read-only at {args.baud}. No bytes transmitted.')
        end=time.monotonic()+args.seconds
        n=0
        while time.monotonic()<end:
            raw=device.read(512)
            if raw:
                n+=len(raw)
                sys.stdout.write(raw.decode('utf-8',errors='replace'))
                sys.stdout.flush()
        print(f'\nReceived {n} bytes. Zero transmitted.')
    except (OSError,serial.SerialException) as ex:
        print(f'Serial error: {ex}',file=sys.stderr)
        return 1
    finally:
        if device.is_open:device.close()
    return 0
if __name__=='__main__':
    sys.exit(main())
