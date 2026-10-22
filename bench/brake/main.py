"""Standalone open-loop brake actuator bench test for MicroPython."""

import time
from machine import Pin


FIRMWARE_NAME = "autokart-brake-bench-test"
FIRMWARE_VERSION = "1.0.0"

BRAKE_STEP_PIN = 10
BRAKE_DIR_PIN = 11
BRAKE_EN_PIN = 12

PULSES_PER_MOTOR_REV = 3200
BRAKE_GEAR_RATIO = 1.0
PULSES_PER_DEG = (
    PULSES_PER_MOTOR_REV * BRAKE_GEAR_RATIO
) / 360.0

# The latest documented build uses DIR LOW to apply the brake.
APPLY_DIR_LEVEL = 0
RELEASE_DIR_LEVEL = 1 - APPLY_DIR_LEVEL
TB_ENABLE_LEVEL = 0
TB_DISABLE_LEVEL = 1

# Latest documented mechanical travel. Revalidate after any linkage change.
BRAKE_MIN_DEG = 0.0
BRAKE_MAX_DEG = 15.0

STEP_HIGH_US = 10
START_PERIOD_US = 1500
FAST_PERIOD_US = 700
ACCEL_US_PER_STEP = 20

step_pin = Pin(BRAKE_STEP_PIN, Pin.OUT, value=0)
dir_pin = Pin(BRAKE_DIR_PIN, Pin.OUT, value=APPLY_DIR_LEVEL)
enable_pin = Pin(BRAKE_EN_PIN, Pin.OUT, value=TB_DISABLE_LEVEL)

# Open-loop estimate. The mechanism must physically start released.
current_steps = 0
driver_enabled = False


def current_angle():
    return current_steps / PULSES_PER_DEG


def enable_driver():
    global driver_enabled
    enable_pin.value(TB_ENABLE_LEVEL)
    driver_enabled = True


def disable_driver():
    global driver_enabled
    step_pin.value(0)
    enable_pin.value(TB_DISABLE_LEVEL)
    driver_enabled = False


def move_to_angle(target_deg):
    global current_steps

    target_deg = max(BRAKE_MIN_DEG, min(BRAKE_MAX_DEG, float(target_deg)))
    target_steps = round(target_deg * PULSES_PER_DEG)
    delta = target_steps - current_steps

    if delta == 0:
        print("Brake already at", round(current_angle(), 2), "deg")
        return

    enable_driver()
    dir_pin.value(APPLY_DIR_LEVEL if delta > 0 else RELEASE_DIR_LEVEL)

    period_us = START_PERIOD_US
    for _ in range(abs(delta)):
        step_pin.value(1)
        time.sleep_us(STEP_HIGH_US)
        step_pin.value(0)
        time.sleep_us(period_us)
        period_us = max(FAST_PERIOD_US, period_us - ACCEL_US_PER_STEP)

    current_steps = target_steps
    print("Brake position:", round(current_angle(), 2), "deg")


def print_status():
    print("Firmware:", FIRMWARE_NAME, FIRMWARE_VERSION)
    print("Driver:", "ENABLED" if driver_enabled else "DISABLED")
    print("Estimated brake angle:", round(current_angle(), 2), "deg")
    print("Range:", BRAKE_MIN_DEG, "to", BRAKE_MAX_DEG, "deg")


print("AutoKart standalone brake test")
print("BENCH ONLY: physically release the brake before power-up.")
print("Commands: apply | release | move <0..15> | status | disable | quit")
print_status()

try:
    while True:
        command = input("BRAKE > ").strip().lower()

        if command == "apply":
            move_to_angle(BRAKE_MAX_DEG)
        elif command == "release":
            move_to_angle(BRAKE_MIN_DEG)
        elif command.startswith("move "):
            move_to_angle(float(command.split()[1]))
        elif command == "status":
            print_status()
        elif command == "disable":
            disable_driver()
            print("Driver disabled")
        elif command in ("quit", "exit"):
            break
        else:
            print("Commands: apply | release | move <0..15> | status | disable | quit")
except KeyboardInterrupt:
    print("Interrupted")
finally:
    disable_driver()
    print("Brake test stopped; TB6600 disabled")
