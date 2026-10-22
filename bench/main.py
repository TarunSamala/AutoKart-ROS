"""AutoKart ESP32-S3 MicroPython RC dynamics prototype (v1.0).

LoRa: NOT PRESENT. No GPIO E-stop input. External independent hardwired stop
is required. The dashboard WEB STOP is a best-effort software stop, not a
substitute for independent hardware stopping. Test wheels lifted first.

Files required on ESP32: main.py, web_ui.py.
GPIO mapping is deliberately frozen to the user's confirmed values.
"""
import gc
import network
import socket
import time
import machine
from machine import Pin, I2C
from web_ui import PAGE

FIRMWARE_NAME = 'AutoKart-MicroPython-R86mini-WebStop'
FIRMWARE_VERSION = '1.1.0-400PPR'

# ---- ESP32-S3 pins (exact confirmed mapping) ----
STEER_STEP_PIN = 4
STEER_DIR_PIN = 5
STEER_EN_PIN = 6
BRAKE_STEP_PIN = 10
BRAKE_DIR_PIN = 11
BRAKE_EN_PIN = 12
SDA_PIN = 8
SCL_PIN = 9
BLD_EN_PIN = 1
BLD_BRK_PIN = 2
BLD_FR_PIN = 7

PIN_MAP = (STEER_STEP_PIN, STEER_DIR_PIN, STEER_EN_PIN,
           BRAKE_STEP_PIN, BRAKE_DIR_PIN, BRAKE_EN_PIN,
           SDA_PIN, SCL_PIN, BLD_EN_PIN, BLD_BRK_PIN, BLD_FR_PIN)
if len(set(PIN_MAP)) != len(PIN_MAP):
    raise ValueError('GPIO collision')

# ---- Configuration: validate against your exact build before movement ----
SSID = 'AutoKart-RC'
PASSWORD = 'CHANGE_ME_2026'  # CHANGE BEFORE TEST; WPA2 AP, not Internet-exposed
COMMAND_TIMEOUT_MS = 1200           # control owner's browser heartbeat
FORWARD_HOLD_TIMEOUT_MS = 450       # separate dead-man for held forward
CONTROL_LEASE_TIMEOUT_MS = 3000
HTTP_RX_LIMIT = 1400
HTTP_TX_CHUNK = 768
HTTP_PEER_TIMEOUT_MS = 1600

# Rtelligent R86mini: SW5 ON, SW6 ON, SW7 ON, SW8 ON = 400 pulses/rev.
# Verified against the manufacturer R86mini microstepping table.
# Bench setup used 1:1 motor->steering shaft; change ratio if mechanically different.
STEER_PULSES_PER_REV = 400
STEER_GEAR_RATIO = 1.0
STEER_LIMIT_DEG = 20.0               # KEEP until hard-stops/clearance verified
STEER_POSITIVE_DIR = 1               # RIGHT. Check wheels-lifted.
STEER_START_PERIOD_US = 3500
STEER_FAST_PERIOD_US = 2200
STEER_ACCEL_US_PER_STEP = 65
STEER_DECEL_STEPS = 10

# NEMA23 + TB6600 mechanical brake: physically RELEASED = zero.
BRAKE_PULSES_PER_REV = 3200
BRAKE_GEAR_RATIO = 1.0
BRAKE_LIMIT_DEG = 15.0               # APPLY travel: check mechanism lifted
BRAKE_POSITIVE_DIR = 0               # APPLY. Check unloaded.
BRAKE_START_PERIOD_US = 1600
BRAKE_FAST_PERIOD_US = 850
BRAKE_ACCEL_US_PER_STEP = 12
BRAKE_DECEL_STEPS = 55

# Wiring assumption: common cathode signals, ENA+ GPIO and ENA- GND.
# R86mini default: optocoupler ON = drive disabled, OFF = motor energized.
# Requires verification with R86mini configuration. TB6600 assumed same.
STEER_ENABLE_LEVEL = 0
STEER_DISABLE_LEVEL = 1
BRAKE_ENABLE_LEVEL = 0
BRAKE_DISABLE_LEVEL = 1
STEP_PULSE_US = 12
DIRECTION_SETUP_US = 25
BRAKE_RELEASE_SETTLE_MS = 150

GP_ADDR = 0x58
REG_MODE = 0x01
REG_DAC = 0x02
MODE_0_10V = 0x1C
THROTTLE_MAX_V = 0.70             # Bench cap only (prior BLD-750 fault)
DEFAULT_DRIVE_V = 0.40
THROTTLE_UPDATE_MS = 15
THROTTLE_RISE_STEP_V = 0.02
THROTTLE_FALL_STEP_V = 0.10
BLD_FORWARD_GPIO_LEVEL = 0       # NPN F/R transistor OFF means forward ASSUMED

# Assumes three NPN low-side transistor interfaces, not direct GPIO to BLD-750:
# EN GPIO 0 = BLD disabled; EN GPIO 1 = BLD run
# BRK GPIO 0 = BLD quick brake; BRK GPIO 1 = release
ENABLE_WDT = True
WDT_TIMEOUT_MS = 4000


def now_ms():
    return time.ticks_ms()


def elapsed(now, before):
    return time.ticks_diff(now, before)


def clamp(x, lo, hi):
    return min(max(x, lo), hi)


class Stepper:
    """Nonblocking pulse scheduler; OPEN LOOP. No end-stop or encoder feedback."""
    def __init__(self, step_pin, dir_pin, enable_pin, ppd, low, high,
                 positive_dir, enable_level, disable_level,
                 start_period, fast_period, acceleration, decel_steps):
        self.step = Pin(step_pin, Pin.OUT, value=0)
        self.direction = Pin(dir_pin, Pin.OUT, value=0)
        self.enable = Pin(enable_pin, Pin.OUT, value=disable_level)
        self.enable_level = enable_level
        self.disable_level = disable_level
        self.ppd = ppd
        self.low = low
        self.high = high
        self.positive_dir = positive_dir
        self.start_period = start_period
        self.fast_period = fast_period
        self.acceleration = acceleration
        self.decel_steps = decel_steps
        self.position = 0
        self.target = 0
        self.period = start_period
        self.prev_direction = 0
        self.prev_step_us = time.ticks_us()
        self.dir_changed_us = time.ticks_us()
        self.is_enabled = False

    def motor_enable(self):
        self.enable.value(self.enable_level)
        self.is_enabled = True
        self.prev_step_us = time.ticks_us()

    def motor_disable(self):
        self.enable.value(self.disable_level)
        self.is_enabled = False
        self.hold()

    def reference_zero(self):
        """CALL ONLY AFTER PHYSICALLY CENTERING / RELEASING mechanisms."""
        self.position = 0
        self.target = 0
        self.period = self.start_period
        self.prev_direction = 0
        self.prev_step_us = time.ticks_us()

    def position_deg(self):
        return self.position / self.ppd

    def target_deg(self):
        return self.target / self.ppd

    def set_target_deg(self, degrees):
        degrees = clamp(float(degrees), self.low, self.high)
        self.target = int(round(degrees * self.ppd))

    def hold(self):
        self.target = self.position
        self.prev_direction = 0
        self.period = self.start_period

    def nudge(self, delta):
        self.set_target_deg(self.position_deg() + float(delta))

    def at_target(self):
        return self.position == self.target

    def update(self):
        if not self.is_enabled or self.position == self.target:
            self.period = self.start_period
            self.prev_direction = 0
            return
        move_dir = 1 if self.target > self.position else -1
        now = time.ticks_us()
        if move_dir != self.prev_direction:
            self.direction.value(self.positive_dir if move_dir > 0 else 1 - self.positive_dir)
            self.dir_changed_us = now
            self.prev_direction = move_dir
            self.period = self.start_period
            return
        if time.ticks_diff(now, self.dir_changed_us) < DIRECTION_SETUP_US:
            return
        if time.ticks_diff(now, self.prev_step_us) < self.period:
            return
        self.step.value(1)
        time.sleep_us(STEP_PULSE_US)
        self.step.value(0)
        self.prev_step_us = time.ticks_us()
        self.position += move_dir
        remaining = abs(self.target - self.position)
        if remaining < self.decel_steps:
            # Gradually slow down when approaching target.
            near_target = self.fast_period + ((self.start_period - self.fast_period)
                                              * (self.decel_steps - remaining)
                                              // self.decel_steps)
        else:
            near_target = self.fast_period
        self.period = max(near_target, self.period - self.acceleration)


class Vehicle:
    def __init__(self):
        # Digital STOP outputs first (requires physical base pulldowns for power-on).
        self.bld_enable = Pin(BLD_EN_PIN, Pin.OUT, value=0)
        self.bld_brake = Pin(BLD_BRK_PIN, Pin.OUT, value=0)
        self.bld_forward = Pin(BLD_FR_PIN, Pin.OUT, value=BLD_FORWARD_GPIO_LEVEL)
        self.bld_run = False
        self.bld_release = False
        self.steer = Stepper(
            STEER_STEP_PIN, STEER_DIR_PIN, STEER_EN_PIN,
            STEER_PULSES_PER_REV * STEER_GEAR_RATIO / 360.0,
            -STEER_LIMIT_DEG, STEER_LIMIT_DEG, STEER_POSITIVE_DIR,
            STEER_ENABLE_LEVEL, STEER_DISABLE_LEVEL, STEER_START_PERIOD_US,
            STEER_FAST_PERIOD_US, STEER_ACCEL_US_PER_STEP, STEER_DECEL_STEPS)
        self.brake = Stepper(
            BRAKE_STEP_PIN, BRAKE_DIR_PIN, BRAKE_EN_PIN,
            BRAKE_PULSES_PER_REV * BRAKE_GEAR_RATIO / 360.0,
            0.0, BRAKE_LIMIT_DEG, BRAKE_POSITIVE_DIR,
            BRAKE_ENABLE_LEVEL, BRAKE_DISABLE_LEVEL, BRAKE_START_PERIOD_US,
            BRAKE_FAST_PERIOD_US, BRAKE_ACCEL_US_PER_STEP, BRAKE_DECEL_STEPS)
        self.reference_confirmed = False
        self.armed = False
        self.estop = False
        self.estop_reason = ''
        self.drive_held = False
        self.drive_pending = False
        self.brake_commanded = False
        self.settle_started = None
        self.throttle_target = 0.0
        self.throttle_current = 0.0
        self.drive_voltage = DEFAULT_DRIVE_V
        self.last_throttle_tick = now_ms()
        self.last_contact = now_ms()
        self.last_forward_hold = now_ms()
        self.steer_seq = -1
        self.lease_token = None
        self.lease_last_seen = None
        # GP8630N initialization; on failure digital paths are already STOP.
        self.i2c = I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=100000)
        seen = self.i2c.scan()
        print('I2C:', [hex(x) for x in seen])
        if GP_ADDR not in seen:
            raise OSError('GP8630N not found at 0x58. Traction remains stopped.')
        self.i2c.writeto_mem(GP_ADDR, REG_MODE, bytes((MODE_0_10V,)))
        time.sleep_ms(100)
        self.write_dac(0.0)

    def write_dac(self, voltage):
        voltage = clamp(float(voltage), 0.0, THROTTLE_MAX_V)
        code = int((voltage / 10.0) * 65535)
        self.i2c.writeto_mem(GP_ADDR, REG_DAC, bytes((code & 255, (code >> 8) & 255)))
        self.throttle_current = voltage

    def stop_digital(self):
        self.bld_enable.value(0)
        self.bld_brake.value(0)
        self.bld_run = False
        self.bld_release = False

    def halt(self):
        """Priority: GPIO stop first, then best-effort DAC, then stepper brake."""
        self.drive_held = False
        self.drive_pending = False
        self.settle_started = None
        self.throttle_target = 0.0
        self.stop_digital()
        try:
            self.write_dac(0.0)
        except Exception as exc:
            print('DAC zero failed (digital stop asserted):', exc)
        if self.reference_confirmed:
            self.brake_commanded = True
            self.brake.set_target_deg(BRAKE_LIMIT_DEG)
        self.steer.hold()

    def latch(self, reason):
        if not self.estop:
            print('!!! WEB/CONTROL STOP LATCHED !!!', reason)
            self.estop_reason = reason
        self.estop = True
        self.armed = False
        self.halt()

    def disarm(self):
        self.armed = False
        self.halt()

    def reference(self):
        # Browser operator explicitly verifies physical steering center and
        # brake fully released before declaring software zero. Not a sensor.
        if self.armed or self.estop or self.drive_held or self.bld_run:
            return False, 'DISARM_OR_RESET_FIRST'
        self.stop_digital()
        self.steer.motor_disable()
        self.brake.motor_disable()
        self.steer.reference_zero()
        self.brake.reference_zero()
        self.reference_confirmed = True
        self.steer.motor_enable()
        self.brake.motor_enable()
        # Apply brake after explicit verification of RELEASED position.
        self.brake_commanded = True
        self.brake.set_target_deg(BRAKE_LIMIT_DEG)
        self.steer_seq = -1
        return True, 'REFERENCE_CONFIRMED_BRAKE_APPLYING'

    def arm(self):
        if self.estop:
            return False, 'ESTOP_LATCHED'
        if not self.reference_confirmed:
            return False, 'CONFIRM_PHYSICAL_REFERENCE_FIRST'
        if not self.brake.at_target() or not self.brake_commanded:
            return False, 'WAIT_BRAKE_APPLIED'
        self.halt()
        self.armed = True
        self.last_contact = now_ms()
        self.steer_seq = -1
        return True, 'ARMED'

    def drive_on(self):
        if not self.armed or self.estop or not self.reference_confirmed:
            return False, 'NOT_READY'
        # Idempotent command. Repeated press never restarts brake release.
        if self.drive_held:
            self.last_forward_hold = now_ms()
            return True, 'ALREADY_HELD'
        self.drive_held = True
        self.last_forward_hold = now_ms()
        self.drive_pending = True
        self.settle_started = None
        self.throttle_target = 0.0
        self.stop_digital()
        self.bld_forward.value(BLD_FORWARD_GPIO_LEVEL)
        self.brake_commanded = False
        self.brake.set_target_deg(0.0)
        return True, 'BRAKE_RELEASING'

    def drive_off(self):
        # User-requested behavior: release forward -> DAC 0 only (coast).
        self.drive_held = False
        self.drive_pending = False
        self.settle_started = None
        self.throttle_target = 0.0
        try:
            self.write_dac(0.0)
        except Exception:
            self.latch('DAC ERROR ON FORWARD RELEASE')
        # Exception: if motor has not reached RUN, keep traction disabled.
        return 'THROTTLE_ZERO'

    def brake_now(self):
        self.drive_held = False
        self.drive_pending = False
        self.settle_started = None
        self.throttle_target = 0.0
        self.stop_digital()  # BRAKE is guaranteed to command BLD STOP
        try:
            self.write_dac(0.0)
        except Exception:
            self.latch('DAC ERROR ON BRAKE')
        if self.reference_confirmed:
            self.brake_commanded = True
            self.brake.set_target_deg(BRAKE_LIMIT_DEG)
        return 'BRAKED'

    def update(self):
        now = now_ms()
        if self.armed and elapsed(now, self.last_contact) > COMMAND_TIMEOUT_MS:
            self.latch('RC HEARTBEAT TIMEOUT')
        if self.armed and self.drive_held and elapsed(now, self.last_forward_hold) > FORWARD_HOLD_TIMEOUT_MS:
            self.latch('FORWARD HOLD TIMEOUT')
        if self.drive_pending and self.armed and not self.estop and self.drive_held:
            if self.brake.at_target():
                if self.settle_started is None:
                    self.settle_started = now
                elif elapsed(now, self.settle_started) >= BRAKE_RELEASE_SETTLE_MS:
                    self.bld_forward.value(BLD_FORWARD_GPIO_LEVEL)
                    self.bld_brake.value(1)
                    self.bld_release = True
                    self.bld_enable.value(1)
                    self.bld_run = True
                    self.drive_pending = False
            else:
                self.settle_started = None
        if self.armed and self.drive_held and not self.drive_pending and self.bld_run and self.bld_release:
            self.throttle_target = self.drive_voltage
        else:
            self.throttle_target = 0.0
        if elapsed(now, self.last_throttle_tick) >= THROTTLE_UPDATE_MS:
            self.last_throttle_tick = now
            goal = self.throttle_target
            if abs(goal - self.throttle_current) > 0.0001:
                if goal > self.throttle_current:
                    new_voltage = min(goal, self.throttle_current + THROTTLE_RISE_STEP_V)
                else:
                    new_voltage = max(goal, self.throttle_current - THROTTLE_FALL_STEP_V)
                try:
                    self.write_dac(new_voltage)
                except Exception as exc:
                    print('GP8630N error:', exc)
                    self.latch('GP8630N I2C ERROR')
        self.steer.update()
        self.brake.update()

    def state(self):
        if self.estop:
            return 'ESTOP'
        if not self.reference_confirmed:
            return 'REFERENCE_REQUIRED'
        if not self.armed:
            return 'SAFE'
        if self.drive_pending:
            return 'RELEASING_BRAKE'
        if self.drive_held and self.bld_run:
            return 'DRIVE'
        if self.brake_commanded:
            return 'BRAKED'
        return 'ARMED'

    def status(self):
        reason = self.estop_reason.replace('|', '/') if self.estop_reason else '-'
        return '|'.join((self.state(), '1' if self.armed else '0',
                         '1' if self.estop else '0',
                         '1' if self.reference_confirmed else '0',
                         '%.1f' % self.steer.position_deg(),
                         '%.1f' % self.brake.position_deg(),
                         '%.2f' % self.throttle_current,
                         '%.2f' % self.drive_voltage,
                         '1' if self.drive_held else '0', reason))

    def owns(self, token):
        if not token or not self.lease_token or token != self.lease_token or self.lease_last_seen is None:
            return False
        return elapsed(now_ms(), self.lease_last_seen) <= CONTROL_LEASE_TIMEOUT_MS

    def claim(self, supplied):
        now = now_ms()
        if self.owns(supplied):
            self.lease_last_seen = now
            return self.lease_token
        if not self.armed and (self.lease_token is None or
                               self.lease_last_seen is None or
                               elapsed(now, self.lease_last_seen) > CONTROL_LEASE_TIMEOUT_MS):
            try:
                import os
                raw = os.urandom(12)
                token = ''.join('%02x' % b for b in raw)
            except Exception:
                token = '%08x%08x' % (machine.unique_id()[0], time.ticks_us())
            self.lease_token = token
            self.lease_last_seen = now
            return token
        return None


def query_param(path, key, default=None):
    try:
        params = path.split('?', 1)[1].split('&')
    except IndexError:
        return default
    for item in params:
        k, sep, value = item.partition('=')
        if sep and k == key:
            return value
    return default


def response(body, ctype='text/plain; charset=utf-8', code='200 OK'):
    if isinstance(body, str):
        body = body.encode('utf-8')
    header = ('HTTP/1.1 ' + code + '\r\n'
              'Content-Type: ' + ctype + '\r\n'
              'Cache-Control: no-store\r\n'
              'X-Content-Type-Options: nosniff\r\n'
              'Content-Length: ' + str(len(body)) + '\r\n'
              'Connection: close\r\n\r\n').encode('ascii')
    return header + body


def handle(vehicle, path):
    route = path.split('?', 1)[0]
    if route == '/' or route == '/index.html':
        return response(PAGE, 'text/html; charset=utf-8')
    if route == '/status':
        return response(vehicle.status())
    if route == '/estop':
        # Anyone authenticated to this Wi-Fi AP can request a safer state.
        vehicle.latch('WEB E-STOP PRESSED')
        return response('ESTOP_LATCHED')
    if route == '/claim':
        token = vehicle.claim(query_param(path, 'token', ''))
        if token is None:
            return response('CONTROL_BUSY', code='409 Conflict')
        return response('TOKEN:' + token)
    token = query_param(path, 'token', '')
    if not vehicle.owns(token):
        return response('CONTROL_DENIED', code='403 Forbidden')
    vehicle.lease_last_seen = now_ms()
    if route == '/heartbeat':
        if vehicle.armed:
            vehicle.last_contact = now_ms()
        return response(vehicle.status())
    if route == '/estop_reset':
        if vehicle.armed:
            return response('DISARM_FIRST', code='409 Conflict')
        vehicle.halt()
        vehicle.estop = False
        vehicle.estop_reason = ''
        return response('RESET_OK_DISARMED')
    if route == '/disarm':
        vehicle.disarm()
        return response('DISARMED')
    if vehicle.estop:
        return response('ESTOP_LATCHED', code='409 Conflict')
    if route == '/reference':
        good, message = vehicle.reference()
        return response(message, code=('200 OK' if good else '409 Conflict'))
    if route == '/arm':
        good, message = vehicle.arm()
        return response(message, code=('200 OK' if good else '409 Conflict'))
    if route == '/drive_off':
        return response(vehicle.drive_off())
    if route == '/brake_now':
        if vehicle.reference_confirmed:
            return response(vehicle.brake_now())
        vehicle.stop_digital()
        return response('REFERENCE_REQUIRED', code='409 Conflict')
    if not vehicle.armed:
        return response('NOT_ARMED', code='409 Conflict')
    if route == '/drive_on':
        good, message = vehicle.drive_on()
        return response(message, code=('200 OK' if good else '409 Conflict'))
    if route == '/drive_keep':
        if not vehicle.drive_held:
            return response('NOT_HELD', code='409 Conflict')
        vehicle.last_forward_hold = now_ms()
        return response('OK')
    if route == '/speed_set':
        try:
            voltage = float(query_param(path, 'v', ''))
            # NaN is invalid:  comparisons otherwise evade clamp.
            if not (0.0 <= voltage <= THROTTLE_MAX_V):
                return response('INVALID_VOLTAGE', code='400 Bad Request')
        except Exception:
            return response('INVALID_VOLTAGE', code='400 Bad Request')
        vehicle.drive_voltage = voltage
        return response('OK')
    if route in ('/steer_left', '/steer_right', '/steer_hold', '/steer_center', '/steer_nudge'):
        try:
            seq = int(query_param(path, 'seq', '-1'))
        except Exception:
            return response('BAD_SEQUENCE', code='400 Bad Request')
        if seq < 0 or seq <= vehicle.steer_seq:
            return response('STALE_SEQUENCE', code='409 Conflict')
        vehicle.steer_seq = seq
        if route == '/steer_left':
            vehicle.steer.set_target_deg(-STEER_LIMIT_DEG)
        elif route == '/steer_right':
            vehicle.steer.set_target_deg(STEER_LIMIT_DEG)
        elif route == '/steer_hold':
            vehicle.steer.hold()
        elif route == '/steer_center':
            vehicle.steer.set_target_deg(0.0)
        else:
            try:
                nudge = float(query_param(path, 'v', ''))
                if not (-5.0 <= nudge <= 5.0):
                    return response('INVALID_NUDGE', code='400 Bad Request')
            except Exception:
                return response('INVALID_NUDGE', code='400 Bad Request')
            vehicle.steer.nudge(nudge)
        return response('OK')
    return response('NOT_FOUND', code='404 Not Found')


class HttpServer:
    """One nonblocking peer at a time, chunked I/O, bounded client lifetime."""
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.listener = socket.socket()
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(('0.0.0.0', 80))
        self.listener.listen(4)
        self.listener.setblocking(False)
        self.peer = None
        self.rx = bytearray()
        self.tx = None
        self.tx_pos = 0
        self.deadline = None

    def close_peer(self):
        if self.peer is not None:
            try:
                self.peer.close()
            except Exception:
                pass
        self.peer = None
        self.rx = bytearray()
        self.tx = None
        self.tx_pos = 0
        self.deadline = None

    def serve_once(self):
        if self.peer is None:
            try:
                peer, _ = self.listener.accept()
                peer.setblocking(False)
                self.peer = peer
                self.deadline = time.ticks_add(now_ms(), HTTP_PEER_TIMEOUT_MS)
            except OSError:
                return
        if time.ticks_diff(now_ms(), self.deadline) > 0:
            self.close_peer()
            return
        if self.tx is None:
            try:
                b = self.peer.recv(512)
                if b:
                    self.rx.extend(b)
                else:
                    self.close_peer()
                    return
            except OSError:
                pass
            if len(self.rx) > HTTP_RX_LIMIT:
                self.tx = response('REQUEST_TOO_LARGE', code='413 Payload Too Large')
            elif b'\r\n\r\n' in self.rx:
                line = bytes(self.rx).split(b'\r\n', 1)[0].decode('utf-8', 'ignore')
                bits = line.split(' ')
                if len(bits) < 3 or bits[0] != 'GET' or len(bits[1]) > 450:
                    self.tx = response('BAD_REQUEST', code='400 Bad Request')
                else:
                    try:
                        self.tx = handle(self.vehicle, bits[1])
                    except Exception as exc:
                        # A routing crash must also terminate traction.
                        print('route exception:', exc)
                        self.vehicle.latch('HTTP ROUTING FAULT')
                        self.tx = response('SERVER_ERROR', code='500 Internal Server Error')
                self.rx = bytearray()
                self.tx_pos = 0
        if self.tx is not None:
            try:
                last = min(self.tx_pos + HTTP_TX_CHUNK, len(self.tx))
                sent = self.peer.send(self.tx[self.tx_pos:last])
                if sent:
                    self.tx_pos += sent
                    self.deadline = time.ticks_add(now_ms(), HTTP_PEER_TIMEOUT_MS)
                if self.tx_pos >= len(self.tx):
                    self.close_peer()
            except OSError:
                pass

    def close(self):
        self.close_peer()
        try:
            self.listener.close()
        except Exception:
            pass


def launch():
    vehicle = None
    server = None
    wdt = None
    try:
        vehicle = Vehicle()
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        try:
            ap.config(essid=SSID, password=PASSWORD)
        except Exception:
            # Newer ports may use 'ssid'; older ones use 'essid'.
            ap.config(ssid=SSID, key=PASSWORD)
        if not ap.active():
            raise RuntimeError('Wi-Fi AP did not start')
        ip = ap.ifconfig()[0]
        server = HttpServer(vehicle)
        if ENABLE_WDT:
            wdt = machine.WDT(timeout=WDT_TIMEOUT_MS)
        print('\nAUTOKART MICRO-PYTHON RC', FIRMWARE_VERSION)
        print('Wi-Fi:', SSID, ' URL: http://' + ip)
        print('Steering: R86mini @ GPIO4/5/6, 400 pulses/rev (SW5-8 ON), limit +/-%.1f deg' % STEER_LIMIT_DEG)
        print('Brake: TB6600 @ GPIO10/11/12')
        print('GP8630N @ GPIO8/9; BLD EN/BRK/F-R @ GPIO1/2/7')
        print('LoRa: NONE; hardware E-stop: external and independent')
        print('Manual reference required; motors WILL NOT auto-move on boot.')
        cycles = 0
        while True:
            vehicle.update()
            server.serve_once()
            if wdt is not None:
                wdt.feed()
            time.sleep_us(150)
            cycles += 1
            if cycles >= 3000:
                gc.collect()
                cycles = 0
    except KeyboardInterrupt:
        print('KeyboardInterrupt: stopping')
    except Exception as exc:
        print('AUTOKART FAULT:', exc)
        if vehicle:
            vehicle.latch('MAIN LOOP FAILURE')
    finally:
        if vehicle:
            vehicle.disarm()
            # A brief best-effort brake stepper attempt only if referenced.
            until = time.ticks_add(now_ms(), 400)
            while vehicle.reference_confirmed and time.ticks_diff(until, now_ms()) > 0:
                try:
                    vehicle.brake.update()
                    if vehicle.brake.at_target():
                        break
                    time.sleep_us(200)
                except Exception:
                    break
            vehicle.steer.motor_disable()
            vehicle.brake.motor_disable()
            vehicle.stop_digital()
        if server:
            server.close()
        print('AUTOKART HALTED - TRACTION COMMAND STOP')


if __name__ == '__main__':
    launch()
