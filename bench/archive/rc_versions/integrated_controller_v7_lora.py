import network
import socket
import time
from machine import Pin, I2C, SoftSPI


# ============================================================
# AUTOKART RC - INTEGRATED ESP32-S3 CONTROLLER
# ============================================================
#
# Combines:
#   1) Steering TB6600 + NEMA23
#   2) Mechanical brake TB6600 + NEMA23
#   3) GP8630N -> BLD-750 speed command
#   4) BLD-750 EN / BRK / F-R transistor control
#   5) RA-02 SX1278 wireless E-stop receiver
#   6) Wi-Fi RC web UI
#
# IMPORTANT SAFETY LIMITATION:
#   Steering and mechanical brake position are OPEN LOOP.
#   Software position is only valid when the kart starts with:
#       STEERING PHYSICALLY CENTERED
#       MECHANICAL BRAKE PHYSICALLY RELEASED
#
#   This is a PROTOTYPE RC controller, not an independent
#   safety-rated emergency-stop system. A real vehicle E-stop
#   must independently remove traction energy / motor enable.
#
# ============================================================


# ============================================================
# WIFI / RC WATCHDOG
# ============================================================

SSID = "AutoKart-RC"
PASSWORD = "12345678"

# Browser heartbeat is 250 ms. If it disappears while armed,
# the kart goes immediately to a latched E-stop.
COMMAND_TIMEOUT_MS = 1200


# ============================================================
# PIN MAP
# ============================================================
#
# Steering TB6600
#   GPIO4  -> PUL+
#   GPIO5  -> DIR+
#   GPIO6  -> ENA+
#   GND    -> PUL- / DIR- / ENA-
#
# Mechanical brake TB6600
#   GPIO10 -> PUL+
#   GPIO11 -> DIR+
#   GPIO12 -> ENA+
#   GND    -> PUL- / DIR- / ENA-
#
# GP8630N
#   GPIO8  -> SDA
#   GPIO9  -> SCL
#
# RA-02 SX1278 (kept on your tested receiver wiring)
#   GPIO14 -> SCK
#   GPIO13 -> MOSI
#   GPIO15 -> MISO
#   GPIO16 -> NSS/CS
#   GPIO17 -> RESET
#   GPIO18 -> DIO0
#
# BLD-750 transistor control
#   MOVED because GPIO13/14/15 are occupied by RA-02.
#   GPIO1 -> EN transistor base driver
#   GPIO2 -> BRK transistor base driver
#   GPIO7 -> F/R transistor base driver
#
# Verify GPIO1/2/7 are physically exposed on your ESP32-S3 board.
# ============================================================

STEER_STEP_PIN = 4
STEER_DIR_PIN = 5
STEER_EN_PIN = 6

BRAKE_STEP_PIN = 10
BRAKE_DIR_PIN = 11
BRAKE_EN_PIN = 12

SDA_PIN = 8
SCL_PIN = 9

LORA_SCK_PIN = 14
LORA_MOSI_PIN = 13
LORA_MISO_PIN = 15
LORA_CS_PIN = 16
LORA_RESET_PIN = 17
LORA_DIO0_PIN = 18

BLD_EN_PIN = 1
BLD_BRK_PIN = 2
BLD_FR_PIN = 7


# ============================================================
# VEHICLE LIMITS / CALIBRATION
# ============================================================

# User-requested steering software limit; mechanical clearance must be verified.
STEER_LIMIT_DEG = 90.0

# Mechanical brake travel command from released -> applied.
BRAKE_LIMIT_DEG = 15.0

# TB6600 switch setting used in your current setup:
#   S1 OFF / S2 OFF / S3 ON -> assumed 3200 pulses/rev.
PULSES_PER_REV = 3200

# Steering: 17T motor gear -> 51T steering gear = 3:1.
STEER_GEAR_RATIO = 51.0 / 17.0
BRAKE_GEAR_RATIO = 1.0

STEER_PULSES_PER_DEG = (
    PULSES_PER_REV / 360.0
) * STEER_GEAR_RATIO

BRAKE_PULSES_PER_DEG = (
    PULSES_PER_REV / 360.0
) * BRAKE_GEAR_RATIO

# Validated steering direction.
STEER_POSITIVE_DIR = 1   # positive = RIGHT

# Your latest brake build reversed APPLY direction.
BRAKE_POSITIVE_DIR = 0   # positive = APPLY

# TB6600 EN assumption used by your working code.
TB_ENABLE_LEVEL = 0
TB_DISABLE_LEVEL = 1


# ============================================================
# STEPPER TIMING
# ============================================================

STEP_PULSE_US = 10

# Loaded steering values from your validated 3:1 setup.
STEER_START_PERIOD_US = 1800
STEER_FAST_PERIOD_US = 800
STEER_ACCEL_US_PER_STEP = 5

BRAKE_START_PERIOD_US = 1500
BRAKE_FAST_PERIOD_US = 700
BRAKE_ACCEL_US_PER_STEP = 20

# Small stabilization delay after software reaches brake-released
# target before BLD-750 is allowed to run.
BRAKE_RELEASE_SETTLE_MS = 100


# ============================================================
# THROTTLE / BLD-750
# ============================================================

GP_ADDR = 0x58
REG_MODE = 0x01
REG_DAC = 0x02
MODE_0_10V = 0x1C

# USER-REQUESTED RC LIMIT:
# UI and software allow 0.00 to 5.00 V command.
# WARNING: previous bench history reported a BLD-750 fault near 0.75 V.
# 5.00 V is therefore NOT validated for loaded vehicle use.
# First test progressively with the rear wheels off the ground.
THROTTLE_MAX_V = 5.00
DEFAULT_DRIVE_V = 0.40

# Normal drive ramp.
THROTTLE_RISE_STEP = 0.02
THROTTLE_FALL_STEP = 0.10
THROTTLE_UPDATE_MS = 15

# Forward only in this RC build.
# With your NPN interface:
#   GPIO LOW -> transistor OFF -> BLD F/R high/open -> CW.
# If CW is not kart-forward, change ONLY this constant to 1
# after wheels-off-ground verification.
BLD_FORWARD_GPIO_LEVEL = 0


# ============================================================
# LORA E-STOP PROTOCOL
# ============================================================

LORA_FREQUENCY = 433_000_000
LORA_LINK_TIMEOUT_MS = 350
GOOD_RUN_PACKETS_REQUIRED = 5

MAGIC0 = 0xA5
MAGIC1 = 0x5A
PROTOCOL_VERSION = 1
EXPECTED_TRANSMITTER_ID = 0x31
CMD_RUN = 0x11
CMD_STOP = 0xEE


# ============================================================
# GPIO - SAFE FIRST
# ============================================================

# BLD digital outputs are initialized BEFORE Wi-Fi / I2C / radio.
# NPN logic from your tested BLD controller:
#   EN GPIO HIGH -> transistor ON -> EN pulled LOW -> RUN
#   EN GPIO LOW  -> transistor OFF -> EN high/open -> STOP
#
#   BRK GPIO HIGH -> transistor ON -> BRK pulled LOW -> RELEASE
#   BRK GPIO LOW  -> transistor OFF -> BRK high/open -> QUICK BRAKE

bld_en_pin = Pin(BLD_EN_PIN, Pin.OUT, value=0)    # STOP
bld_brk_pin = Pin(BLD_BRK_PIN, Pin.OUT, value=0) # BRAKE APPLIED
bld_fr_pin = Pin(BLD_FR_PIN, Pin.OUT, value=BLD_FORWARD_GPIO_LEVEL)

bld_enabled = False
bld_brake_released = False


def bld_enable_run():
    global bld_enabled
    bld_en_pin.value(1)
    bld_enabled = True


def bld_enable_stop():
    global bld_enabled
    bld_en_pin.value(0)
    bld_enabled = False


def bld_brake_release():
    global bld_brake_released
    bld_brk_pin.value(1)
    bld_brake_released = True


def bld_brake_apply():
    global bld_brake_released
    bld_brk_pin.value(0)
    bld_brake_released = False


def bld_set_forward():
    bld_fr_pin.value(BLD_FORWARD_GPIO_LEVEL)


# Ensure safe boot state immediately.
bld_enable_stop()
bld_brake_apply()
bld_set_forward()


# ============================================================
# STEPPER AXIS
# ============================================================

class StepperAxis:

    def __init__(
        self,
        step_pin,
        dir_pin,
        pulses_per_deg,
        min_deg,
        max_deg,
        positive_dir,
        start_period_us,
        fast_period_us,
        accel_us_per_step
    ):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.pulses_per_deg = pulses_per_deg
        self.min_deg = min_deg
        self.max_deg = max_deg
        self.positive_dir = positive_dir
        self.start_period_us = start_period_us
        self.fast_period_us = fast_period_us
        self.accel_us_per_step = accel_us_per_step

        # OPEN-LOOP software estimate.
        self.current_steps = 0
        self.target_steps = 0
        self.current_period_us = start_period_us
        self.last_direction = 0
        self.last_step_us = time.ticks_us()

    def set_target(self, degrees):
        degrees = float(degrees)
        degrees = max(self.min_deg, min(self.max_deg, degrees))
        self.target_steps = round(degrees * self.pulses_per_deg)

    def hold(self):
        self.target_steps = self.current_steps
        self.current_period_us = self.start_period_us
        self.last_direction = 0

    def current_deg(self):
        return self.current_steps / self.pulses_per_deg

    def target_deg(self):
        return self.target_steps / self.pulses_per_deg

    def at_target(self):
        return self.current_steps == self.target_steps

    def nudge(self, delta_deg):
        self.set_target(self.current_deg() + float(delta_deg))

    def update(self):
        if self.current_steps == self.target_steps:
            self.current_period_us = self.start_period_us
            self.last_direction = 0
            return

        direction = 1 if self.target_steps > self.current_steps else -1

        if direction != self.last_direction:
            self.current_period_us = self.start_period_us
            self.last_direction = direction

        now = time.ticks_us()

        if time.ticks_diff(now, self.last_step_us) < self.current_period_us:
            return

        if direction > 0:
            self.dir_pin.value(self.positive_dir)
        else:
            self.dir_pin.value(1 - self.positive_dir)

        self.step_pin.value(1)
        time.sleep_us(STEP_PULSE_US)
        self.step_pin.value(0)

        self.current_steps += direction
        self.last_step_us = now

        if self.current_period_us > self.fast_period_us:
            self.current_period_us = max(
                self.fast_period_us,
                self.current_period_us - self.accel_us_per_step
            )


STEER_STEP = Pin(STEER_STEP_PIN, Pin.OUT, value=0)
STEER_DIR = Pin(STEER_DIR_PIN, Pin.OUT, value=0)
STEER_EN = Pin(STEER_EN_PIN, Pin.OUT, value=TB_DISABLE_LEVEL)

BRAKE_STEP = Pin(BRAKE_STEP_PIN, Pin.OUT, value=0)
BRAKE_DIR = Pin(BRAKE_DIR_PIN, Pin.OUT, value=0)
BRAKE_EN = Pin(BRAKE_EN_PIN, Pin.OUT, value=TB_DISABLE_LEVEL)

steering = StepperAxis(
    STEER_STEP,
    STEER_DIR,
    STEER_PULSES_PER_DEG,
    -STEER_LIMIT_DEG,
    STEER_LIMIT_DEG,
    STEER_POSITIVE_DIR,
    STEER_START_PERIOD_US,
    STEER_FAST_PERIOD_US,
    STEER_ACCEL_US_PER_STEP
)

mechanical_brake = StepperAxis(
    BRAKE_STEP,
    BRAKE_DIR,
    BRAKE_PULSES_PER_DEG,
    0.0,
    BRAKE_LIMIT_DEG,
    BRAKE_POSITIVE_DIR,
    BRAKE_START_PERIOD_US,
    BRAKE_FAST_PERIOD_US,
    BRAKE_ACCEL_US_PER_STEP
)

STEER_EN.value(TB_ENABLE_LEVEL)
BRAKE_EN.value(TB_ENABLE_LEVEL)


# ============================================================
# GP8630N
# ============================================================

i2c = I2C(
    0,
    sda=Pin(SDA_PIN),
    scl=Pin(SCL_PIN),
    freq=100000
)

print()
print("I2C scan:", [hex(x) for x in i2c.scan()])

if GP_ADDR not in i2c.scan():
    bld_enable_stop()
    bld_brake_apply()
    raise RuntimeError("GP8630N NOT FOUND AT 0x58")

i2c.writeto_mem(
    GP_ADDR,
    REG_MODE,
    bytes([MODE_0_10V])
)

time.sleep_ms(100)

throttle_current = 0.0
throttle_target = 0.0
last_throttle_update = time.ticks_ms()


def set_dac_voltage(voltage):
    global throttle_current

    voltage = max(0.0, min(THROTTLE_MAX_V, float(voltage)))

    dac = int((voltage / 10.0) * 65535)
    low = dac & 0xFF
    high = (dac >> 8) & 0xFF

    i2c.writeto_mem(
        GP_ADDR,
        REG_DAC,
        bytes([low, high])
    )

    throttle_current = voltage


def throttle_zero_immediate():
    global throttle_target
    throttle_target = 0.0
    set_dac_voltage(0.0)


def throttle_update():
    global throttle_current
    global last_throttle_update

    now = time.ticks_ms()

    if time.ticks_diff(now, last_throttle_update) < THROTTLE_UPDATE_MS:
        return

    last_throttle_update = now

    if throttle_current < throttle_target:
        set_dac_voltage(
            min(throttle_current + THROTTLE_RISE_STEP, throttle_target)
        )
    elif throttle_current > throttle_target:
        set_dac_voltage(
            max(throttle_current - THROTTLE_FALL_STEP, throttle_target)
        )


# Absolute safe traction initialization.
throttle_zero_immediate()


# ============================================================
# SX1278 LORA DRIVER
# ============================================================

class SX1278:

    REG_FIFO = 0x00
    REG_OP_MODE = 0x01
    REG_FRF_MSB = 0x06
    REG_FRF_MID = 0x07
    REG_FRF_LSB = 0x08
    REG_LNA = 0x0C
    REG_FIFO_ADDR_PTR = 0x0D
    REG_FIFO_TX_BASE_ADDR = 0x0E
    REG_FIFO_RX_BASE_ADDR = 0x0F
    REG_FIFO_RX_CURRENT_ADDR = 0x10
    REG_IRQ_FLAGS = 0x12
    REG_RX_NB_BYTES = 0x13
    REG_MODEM_CONFIG_1 = 0x1D
    REG_MODEM_CONFIG_2 = 0x1E
    REG_PREAMBLE_MSB = 0x20
    REG_PREAMBLE_LSB = 0x21
    REG_MODEM_CONFIG_3 = 0x26
    REG_SYNC_WORD = 0x39
    REG_DIO_MAPPING_1 = 0x40
    REG_VERSION = 0x42

    MODE_LONG_RANGE = 0x80
    MODE_SLEEP = 0x00
    MODE_STANDBY = 0x01
    MODE_RX_CONTINUOUS = 0x05

    IRQ_RX_DONE = 0x40
    IRQ_PAYLOAD_CRC_ERROR = 0x20

    def __init__(self, spi, cs, reset, dio0, frequency):
        self.spi = spi
        self.cs = cs
        self.reset = reset
        self.dio0 = dio0
        self.frequency = frequency

    def write_reg(self, address, value):
        self.cs.value(0)
        self.spi.write(bytes((address | 0x80, value & 0xFF)))
        self.cs.value(1)

    def read_reg(self, address):
        self.cs.value(0)
        self.spi.write(bytes((address & 0x7F,)))
        value = self.spi.read(1, 0x00)[0]
        self.cs.value(1)
        return value

    def read_fifo(self, length):
        self.cs.value(0)
        self.spi.write(bytes((self.REG_FIFO & 0x7F,)))
        data = self.spi.read(length, 0x00)
        self.cs.value(1)
        return data

    def reset_radio(self):
        self.reset.value(0)
        time.sleep_ms(10)
        self.reset.value(1)
        time.sleep_ms(20)

    def set_frequency(self, frequency):
        frf = int(frequency * 524288 / 32_000_000)
        self.write_reg(self.REG_FRF_MSB, (frf >> 16) & 0xFF)
        self.write_reg(self.REG_FRF_MID, (frf >> 8) & 0xFF)
        self.write_reg(self.REG_FRF_LSB, frf & 0xFF)

    def standby(self):
        self.write_reg(
            self.REG_OP_MODE,
            self.MODE_LONG_RANGE | self.MODE_STANDBY
        )

    def begin(self):
        self.cs.value(1)
        self.reset_radio()

        version = self.read_reg(self.REG_VERSION)
        print("SX1278 VERSION:", hex(version))

        if version != 0x12:
            raise OSError(
                "RA-02 not detected; expected 0x12 got 0x%02X" % version
            )

        self.write_reg(
            self.REG_OP_MODE,
            self.MODE_LONG_RANGE | self.MODE_SLEEP
        )
        time.sleep_ms(10)

        self.set_frequency(self.frequency)
        self.write_reg(self.REG_FIFO_TX_BASE_ADDR, 0x00)
        self.write_reg(self.REG_FIFO_RX_BASE_ADDR, 0x00)
        self.write_reg(self.REG_LNA, self.read_reg(self.REG_LNA) | 0x03)

        # Must match your Pico transmitter:
        # BW125 / CR4-5 / explicit header / SF7 / HW CRC / AGC.
        self.write_reg(self.REG_MODEM_CONFIG_1, 0x72)
        self.write_reg(self.REG_MODEM_CONFIG_2, 0x74)
        self.write_reg(self.REG_MODEM_CONFIG_3, 0x04)
        self.write_reg(self.REG_PREAMBLE_MSB, 0x00)
        self.write_reg(self.REG_PREAMBLE_LSB, 0x08)
        self.write_reg(self.REG_SYNC_WORD, 0x12)
        self.write_reg(self.REG_IRQ_FLAGS, 0xFF)
        self.standby()

    def start_receive(self):
        self.write_reg(self.REG_DIO_MAPPING_1, 0x00) # DIO0 = RxDone
        self.write_reg(self.REG_IRQ_FLAGS, 0xFF)
        self.write_reg(
            self.REG_OP_MODE,
            self.MODE_LONG_RANGE | self.MODE_RX_CONTINUOUS
        )

    def receive(self):
        if self.dio0.value() == 0:
            return None

        flags = self.read_reg(self.REG_IRQ_FLAGS)
        self.write_reg(self.REG_IRQ_FLAGS, flags)

        if not (flags & self.IRQ_RX_DONE):
            return None

        if flags & self.IRQ_PAYLOAD_CRC_ERROR:
            return None

        length = self.read_reg(self.REG_RX_NB_BYTES)
        fifo_address = self.read_reg(self.REG_FIFO_RX_CURRENT_ADDR)
        self.write_reg(self.REG_FIFO_ADDR_PTR, fifo_address)

        return self.read_fifo(length)


# ============================================================
# LORA PROTOCOL HELPERS
# ============================================================

def crc16(data):
    crc = 0xFFFF

    for byte in data:
        crc ^= byte << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


def parse_lora_packet(packet):
    if packet is None or len(packet) != 11:
        return None

    if packet[0] != MAGIC0 or packet[1] != MAGIC1:
        return None

    if packet[2] != PROTOCOL_VERSION:
        return None

    if packet[3] != EXPECTED_TRANSMITTER_ID:
        return None

    command = packet[4]

    if command not in (CMD_RUN, CMD_STOP):
        return None

    session = (packet[5] << 8) | packet[6]
    sequence = (packet[7] << 8) | packet[8]
    received_crc = (packet[9] << 8) | packet[10]

    if received_crc != crc16(packet[:9]):
        return None

    return command, session, sequence


def sequence_is_fresh(new_seq, old_seq):
    if old_seq is None:
        return True

    difference = (new_seq - old_seq) & 0xFFFF
    return 0 < difference < 0x8000


# ============================================================
# RADIO HARDWARE
# ============================================================

lora_spi = SoftSPI(
    baudrate=2_000_000,
    polarity=0,
    phase=0,
    sck=Pin(LORA_SCK_PIN),
    mosi=Pin(LORA_MOSI_PIN),
    miso=Pin(LORA_MISO_PIN)
)

lora_cs = Pin(LORA_CS_PIN, Pin.OUT, value=1)
lora_reset = Pin(LORA_RESET_PIN, Pin.OUT, value=1)
lora_dio0 = Pin(LORA_DIO0_PIN, Pin.IN)

radio = SX1278(
    lora_spi,
    lora_cs,
    lora_reset,
    lora_dio0,
    LORA_FREQUENCY
)

radio_fault = False
radio_fault_text = ""

try:
    radio.begin()
    radio.start_receive()
    print("RA-02 RX READY")
except Exception as e:
    radio_fault = True
    radio_fault_text = str(e)
    print("RA-02 FAULT:", e)


# ============================================================
# VEHICLE / SAFETY STATE
# ============================================================

armed = False
estop_latched = False
estop_reason = ""

# LoRa authorization state.
radio_session = None
radio_last_sequence = None
radio_last_valid_packet_ms = None
radio_good_run_count = 0
radio_run_permit = False
radio_timeout_latched_once = False

# RC motion state.
drive_requested = False
drive_start_pending = False
drive_release_settle_started = None
selected_drive_voltage = DEFAULT_DRIVE_V
controlled_stop_active = False
brake_commanded = True
last_contact = time.ticks_ms()

# Reject stale/out-of-order steering commands.
steer_cmd_seq = -1

# Start physically centered and brake released, then command brake apply.
steering.set_target(0.0)
mechanical_brake.set_target(BRAKE_LIMIT_DEG)


# ============================================================
# SAFETY FUNCTIONS
# ============================================================

def radio_link_alive():
    if radio_fault:
        return False

    if radio_last_valid_packet_ms is None:
        return False

    age = time.ticks_diff(
        time.ticks_ms(),
        radio_last_valid_packet_ms
    )

    return age <= LORA_LINK_TIMEOUT_MS


def radio_ready_for_arm():
    return (
        (not radio_fault) and
        radio_run_permit and
        radio_link_alive()
    )


def apply_mechanical_brake():
    global brake_commanded
    brake_commanded = True
    mechanical_brake.set_target(BRAKE_LIMIT_DEG)


def release_mechanical_brake():
    global brake_commanded
    brake_commanded = False
    mechanical_brake.set_target(0.0)


def safe_hardware_stop(immediate_dac=True):
    global throttle_target
    global drive_requested
    global drive_start_pending
    global drive_release_settle_started
    global controlled_stop_active

    drive_requested = False
    drive_start_pending = False
    drive_release_settle_started = None
    controlled_stop_active = False
    throttle_target = 0.0

    if immediate_dac:
        try:
            set_dac_voltage(0.0)
        except:
            pass

    bld_brake_apply()
    bld_enable_stop()
    apply_mechanical_brake()
    steering.hold()


def latch_estop(reason):
    global armed
    global estop_latched
    global estop_reason

    if not estop_latched:
        print()
        print("!!! E-STOP LATCHED !!!")
        print("Reason:", reason)
        print()

    estop_latched = True
    estop_reason = reason
    armed = False
    safe_hardware_stop(immediate_dac=True)


def disarm_now(reason="DISARM"):
    global armed
    armed = False
    safe_hardware_stop(immediate_dac=True)
    print(reason)


def request_throttle_zero_only(reason="FORWARD RELEASED"):
    global drive_requested
    global drive_start_pending
    global drive_release_settle_started
    global controlled_stop_active
    global throttle_target

    # Requested RC behavior:
    # releasing FORWARD changes ONLY the traction command to 0 V.
    # It does NOT apply the BLD brake, does NOT disable BLD EN,
    # and does NOT apply the mechanical stepper brake.
    drive_requested = False
    drive_start_pending = False
    drive_release_settle_started = None
    controlled_stop_active = False
    throttle_target = 0.0
    try:
        set_dac_voltage(0.0)
    except Exception as e:
        print("FORWARD RELEASE DAC ZERO ERROR:", e)
    print("FORWARD RELEASE -> IMMEDIATE THROTTLE 0 V ONLY:", reason)


def brake_now(reason="BRAKE BUTTON"):
    global controlled_stop_active
    global drive_requested
    global drive_start_pending
    global throttle_target

    drive_requested = False
    drive_start_pending = False
    controlled_stop_active = False
    throttle_target = 0.0

    try:
        set_dac_voltage(0.0)
    except:
        pass

    bld_brake_apply()
    apply_mechanical_brake()
    print("BRAKE: DAC 0 + BLD BRAKE + MECHANICAL BRAKE:", reason)


def start_drive_request():
    global drive_requested
    global drive_start_pending
    global drive_release_settle_started
    global controlled_stop_active
    global throttle_target

    if not armed:
        return False, "NOT_ARMED"

    if estop_latched:
        return False, "ESTOP_LATCHED"

    # LoRa health does NOT gate driving in this requested prototype mode.
    # A valid received CMD_STOP still latches E-stop immediately.

    # Traction remains stopped while mechanical brake retracts.
    drive_requested = True
    drive_start_pending = True
    drive_release_settle_started = None
    controlled_stop_active = False
    throttle_target = 0.0

    bld_enable_stop()
    bld_brake_apply()
    bld_set_forward()
    release_mechanical_brake()

    return True, "RELEASING_BRAKE"


# ============================================================
# LORA PROCESSING
# ============================================================

def process_lora():
    global radio_session
    global radio_last_sequence
    global radio_last_valid_packet_ms
    global radio_good_run_count
    global radio_run_permit
    global radio_timeout_latched_once

    if radio_fault:
        return

    packet = radio.receive()

    if packet is not None:
        parsed = parse_lora_packet(packet)

        if parsed is not None:
            command, session, sequence = parsed

            # Transmitter reboot/session change invalidates RUN permission.
            if radio_session != session:
                had_previous_session = radio_session is not None
                radio_session = session
                radio_last_sequence = None
                radio_good_run_count = 0
                radio_run_permit = False
                radio_timeout_latched_once = False

                if had_previous_session:
                    print("LORA WARNING: REMOTE TX SESSION CHANGED")

            if sequence_is_fresh(sequence, radio_last_sequence):
                radio_last_sequence = sequence
                radio_last_valid_packet_ms = time.ticks_ms()
                radio_timeout_latched_once = False

                if command == CMD_STOP:
                    radio_good_run_count = 0
                    radio_run_permit = False
                    latch_estop("REMOTE E-STOP PRESSED")

                elif command == CMD_RUN:
                    radio_good_run_count += 1

                    if radio_good_run_count >= GOOD_RUN_PACKETS_REQUIRED:
                        radio_run_permit = True

    # Once a valid link has existed, packet loss is WARNING-ONLY in this requested prototype mode.
    if radio_last_valid_packet_ms is not None:
        age = time.ticks_diff(
            time.ticks_ms(),
            radio_last_valid_packet_ms
        )

        if age > LORA_LINK_TIMEOUT_MS:
            radio_good_run_count = 0
            radio_run_permit = False

            if not radio_timeout_latched_once:
                radio_timeout_latched_once = True
                print("LORA WARNING: HEARTBEAT TIMEOUT - VEHICLE CONTINUES")


# ============================================================
# VEHICLE STATE UPDATE
# ============================================================

def update_drive_state():
    global drive_start_pending
    global drive_release_settle_started
    global controlled_stop_active
    global throttle_target

    now = time.ticks_ms()

    # Start sequence:
    #   1) mechanical brake retracts
    #   2) small settle time
    #   3) BLD brake release
    #   4) BLD enable RUN
    #   5) throttle ramp begins
    if drive_start_pending:
        if not drive_requested or not armed or estop_latched:
            drive_start_pending = False
            return

        if mechanical_brake.at_target():
            if drive_release_settle_started is None:
                drive_release_settle_started = now

            elif time.ticks_diff(
                now,
                drive_release_settle_started
            ) >= BRAKE_RELEASE_SETTLE_MS:
                bld_set_forward()
                bld_brake_release()
                bld_enable_run()
                throttle_target = selected_drive_voltage
                drive_start_pending = False
                print("DRIVE ACTIVE")
        else:
            drive_release_settle_started = None

    # While Forward is held, update requested throttle.
    if (
        drive_requested and
        (not drive_start_pending) and
        armed and
        (not estop_latched)
    ):
        if bld_enabled and bld_brake_released:
            throttle_target = selected_drive_voltage

    # FORWARD release is throttle-zero only in this requested RC mode.
    if controlled_stop_active:
        throttle_target = 0.0
        controlled_stop_active = False


# ============================================================
# UI STATE
# ============================================================

def state_name():
    if estop_latched:
        return "ESTOP"

    if radio_fault:
        return "RADIO_FAULT"

    if not armed:
        return "SAFE"

    if drive_start_pending:
        return "BRAKE_RELEASE"

    if drive_requested and bld_enabled:
        return "DRIVE"

    if controlled_stop_active:
        return "STOPPING"

    if brake_commanded:
        return "BRAKED"

    return "ARMED"


def lora_state_name():
    if radio_fault:
        return "FAULT"
    if radio_ready_for_arm():
        return "LINK_OK"
    if radio_last_valid_packet_ms is None:
        return "WAITING"
    if not radio_link_alive():
        return "TIMEOUT"
    return "VALIDATING"


def lora_warning_text():
    if radio_fault:
        return "LORA RECEIVER FAULT - WIRELESS E-STOP UNAVAILABLE"
    if radio_last_valid_packet_ms is None:
        return "LORA WAITING - WIRELESS E-STOP NOT CONFIRMED"
    if not radio_link_alive():
        return "LORA LINK LOST - WARNING ONLY - KART CONTINUES"
    return "-"


def status_text():
    # Pipe-separated compact status for low-overhead polling.
    reason = estop_reason.replace("|", "/") if estop_reason else "-"

    return "|".join((
        state_name(),
        "1" if armed else "0",
        lora_state_name(),
        "1" if estop_latched else "0",
        "%.1f" % steering.current_deg(),
        "%.1f" % mechanical_brake.current_deg(),
        "%.2f" % throttle_current,
        "%.2f" % selected_drive_voltage,
        reason,
        lora_warning_text().replace("|", "/")
    ))


# ============================================================
# WIFI ACCESS POINT
# ============================================================

ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=SSID, password=PASSWORD)

while not ap.active():
    time.sleep_ms(100)

IP = ap.ifconfig()[0]

print()
print("============================================")
print("        AUTOKART RC - INTEGRATED")
print("============================================")
print("Wi-Fi       :", SSID)
print("Password    :", PASSWORD)
print("URL         : http://" + IP)
print("Steering    : +/-", STEER_LIMIT_DEG, "deg")
print("Brake       :", BRAKE_LIMIT_DEG, "deg")
print("Throttle max:", THROTTLE_MAX_V, "V")
print("LoRa        : 433 MHz / RX on GPIO13-18")
print("BLD EN/BRK/F-R: GPIO1 / GPIO2 / GPIO7")
print("============================================")
print()


# ============================================================
# WEB UI
# ============================================================

HTML = r'''<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="theme-color" content="#07090d">
<title>AutoKart RC</title>
<style>
:root{
  --bg:#07090d;--panel:#11151c;--panel2:#171c25;--line:#252d39;
  --text:#f6f8fb;--muted:#8f9aaa;--green:#19b66f;--red:#ef3948;
  --amber:#f4b23d;--blue:#4b8cff;
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;user-select:none}
html,body{margin:0;min-height:100%;background:var(--bg);color:var(--text);font-family:Inter,Arial,sans-serif;overscroll-behavior:none}
body{padding:12px}
.app{width:min(1050px,100%);margin:auto}
.top{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:10px}
.brand{font-size:22px;font-weight:900;letter-spacing:.3px}.sub{font-size:11px;color:var(--muted);margin-top:3px;letter-spacing:1.5px}
.pills{display:flex;gap:8px;flex-wrap:wrap;justify-content:flex-end}.pill{padding:8px 11px;border:1px solid var(--line);background:var(--panel);border-radius:999px;font-size:12px;font-weight:900}.ok{color:#61e9a8}.bad{color:#ff7380}.warn{color:#ffd27a}
.grid{display:grid;grid-template-columns:1.3fr 1fr .78fr;gap:10px}.card{background:linear-gradient(180deg,var(--panel2),var(--panel));border:1px solid var(--line);border-radius:20px;padding:12px;box-shadow:0 10px 30px rgba(0,0,0,.18)}
.label{font-size:11px;letter-spacing:1.8px;color:var(--muted);font-weight:900;margin-bottom:10px;text-align:center}
.steer-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}.steer{height:148px;border:0;border-radius:18px;background:#252c37;color:white;font-size:64px;font-weight:900;touch-action:none}.steer.active{background:#485467;transform:scale(.985)}
.center{width:100%;height:48px;margin-top:10px;border:0;border-radius:14px;background:#303846;color:white;font-weight:900}
.motion{display:grid;grid-template-columns:1fr 1fr;gap:10px}.drive,.brake{height:148px;border:0;border-radius:18px;color:white;font-size:22px;font-weight:1000;touch-action:none}.drive{background:linear-gradient(180deg,#1abe76,#0e854e)}.drive.active{filter:brightness(1.18);transform:scale(.985)}.brake{background:linear-gradient(180deg,#f44753,#ae1e29)}.brake.active{filter:brightness(1.18);transform:scale(.985)}
.speedbox{margin-top:11px;padding:10px;border-radius:15px;background:#0c1016;border:1px solid var(--line)}.speedrow{display:flex;justify-content:space-between;align-items:center;font-size:12px;font-weight:900}.speedval{font-size:20px}.slider{width:100%;margin-top:8px;accent-color:var(--green)}
.system{display:grid;grid-template-columns:1fr 1fr;gap:8px}.system button{height:55px;border:0;border-radius:14px;color:#fff;font-weight:900;font-size:14px}.arm{background:#148c55}.disarm{background:#82313a}
.telemetry{margin-top:10px;display:grid;grid-template-columns:1fr 1fr;gap:7px}.metric{padding:10px;background:#0b0f15;border:1px solid var(--line);border-radius:13px}.mname{font-size:9px;color:var(--muted);font-weight:900;letter-spacing:1px}.mval{font-size:17px;font-weight:900;margin-top:3px}
.estop{width:100%;height:70px;margin-top:10px;border:3px solid #ff7581;border-radius:18px;background:#df1424;color:#fff;font-size:24px;font-weight:1000;letter-spacing:1px}.estop:active{transform:scale(.99);background:#ff2234}
.hint{text-align:center;color:var(--muted);font-size:11px;margin:9px 0 0}
.overlay{position:fixed;inset:0;background:rgba(0,0,0,.91);display:none;align-items:center;justify-content:center;padding:18px;z-index:100}.overlay.show{display:flex}.modal{width:min(430px,100%);background:#1a0d11;border:3px solid var(--red);border-radius:22px;padding:24px;text-align:center}.mtitle{font-size:30px;color:#ff5260;font-weight:1000}.mreason{margin:14px 0 18px;font-size:15px;line-height:1.5}.reset{width:100%;height:56px;border:0;border-radius:14px;background:#424b59;color:white;font-size:16px;font-weight:900}.reset:disabled{opacity:.35}
.lost{position:fixed;top:10px;left:50%;transform:translateX(-50%);padding:10px 14px;border-radius:999px;background:#8c202b;color:#fff;font-size:12px;font-weight:900;display:none;z-index:80}.lost.show{display:block}.lora-warn{position:fixed;top:52px;left:50%;transform:translateX(-50%);padding:10px 14px;border-radius:999px;background:#6f4a08;color:#ffe7a6;border:1px solid #b47b18;font-size:12px;font-weight:900;display:none;z-index:79;max-width:calc(100vw - 20px);text-align:center}.lora-warn.show{display:block}
@media(max-width:820px){body{padding:7px}.grid{grid-template-columns:1.2fr 1fr}.card.system-card{grid-column:1/-1}.telemetry{grid-template-columns:repeat(4,1fr)}}
@media(max-width:560px){.brand{font-size:18px}.sub{font-size:9px}.pills{gap:4px}.pill{font-size:10px;padding:6px 8px}.grid{grid-template-columns:1fr 1fr;gap:7px}.card{padding:8px;border-radius:16px}.system-card{grid-column:1/-1!important}.steer{height:112px;font-size:49px}.drive,.brake{height:112px;font-size:18px}.center{height:42px}.motion{gap:7px}.telemetry{grid-template-columns:repeat(4,1fr)}.metric{padding:8px 5px}.mval{font-size:14px}.mname{font-size:8px}.estop{height:60px;font-size:20px}}
</style>
</head>
<body>
<div id="lost" class="lost">RC LINK LOST — E-STOP LATCHED</div>
<div id="loraWarn" class="lora-warn">LORA LINK WARNING — WIRELESS E-STOP UNAVAILABLE</div>
<div class="app">
  <div class="top">
    <div><div class="brand">AutoKart RC</div><div class="sub">ESP32-S3 · BLD-750 · DUAL STEPPER · LORA E-STOP</div></div>
    <div class="pills"><div id="modePill" class="pill warn">BOOT</div><div id="loraPill" class="pill warn">LORA WAIT</div></div>
  </div>

  <div class="grid">
    <div class="card">
      <div class="label">STEERING</div>
      <div class="steer-grid">
        <button id="left" class="steer">◀</button>
        <button id="right" class="steer">▶</button>
      </div>
      <button id="center" class="center">CENTER STEERING</button>
    </div>

    <div class="card">
      <div class="label">MOTION</div>
      <div class="motion">
        <button id="drive" class="drive">HOLD<br>FORWARD</button>
        <button id="brake" class="brake">BRAKE<br>NOW</button>
      </div>
      <div class="speedbox">
        <div class="speedrow"><span>DRIVE COMMAND</span><span id="speedVal" class="speedval">__DEFAULT_V__ V</span></div>
        <input id="speed" class="slider" type="range" min="0" max="__MAX_CV__" step="1" value="__DEFAULT_CV__">
      </div>
    </div>

    <div class="card system-card">
      <div class="label">SYSTEM</div>
      <div class="system">
        <button id="arm" class="arm">ARM</button>
        <button id="disarm" class="disarm">DISARM</button>
      </div>
      <div class="telemetry">
        <div class="metric"><div class="mname">STEER EST</div><div id="steerVal" class="mval">0.0°</div></div>
        <div class="metric"><div class="mname">BRAKE EST</div><div id="brakeVal" class="mval">--</div></div>
        <div class="metric"><div class="mname">SV ACTUAL</div><div id="svVal" class="mval">0.00 V</div></div>
        <div class="metric"><div class="mname">ARM</div><div id="armVal" class="mval">NO</div></div>
      </div>
    </div>
  </div>

  <button id="estop" class="estop">EMERGENCY STOP</button>
  <div class="hint">FORWARD release = throttle 0 V only. BRAKE NOW applies BLD + mechanical brake. LoRa loss = warning only; received LoRa STOP still latches E-stop.</div>
</div>

<div id="overlay" class="overlay">
  <div class="modal">
    <div class="mtitle">E-STOP LATCHED</div>
    <div id="reason" class="mreason">Traction zero · BLD stop · BLD brake · mechanical brake commanded</div>
    <button id="reset" class="reset">RESET E-STOP</button>
  </div>
</div>

<script>
let armed=false, estop=false, lora="WAITING", lostCount=0;
let steerSeq=0, steerTimer=null, steerContinuous=false, steerDir=0;
const HOLD_MS=120, TAP_DEG=4;
const $=id=>document.getElementById(id);

function req(url){
  return fetch(url,{cache:"no-store"}).then(r=>r.text());
}
function steerReq(path){steerSeq++; const s=path.includes("?")?"&":"?"; return req(path+s+"seq="+steerSeq);}

function pressSteer(e,dir){
  if(!armed||estop)return; e.preventDefault();
  clearTimeout(steerTimer); steerDir=dir; steerContinuous=false;
  const b=dir<0?$("left"):$("right"), o=dir<0?$("right"):$("left");
  b.classList.add("active");o.classList.remove("active");
  try{b.setPointerCapture(e.pointerId)}catch(_){ }
  steerReq("/steer_nudge?v="+(dir*TAP_DEG));
  steerTimer=setTimeout(()=>{if(steerDir!==dir||!armed||estop)return; steerContinuous=true; steerReq(dir<0?"/steer_left":"/steer_right");},HOLD_MS);
}
function releaseSteer(dir){
  if(steerDir!==dir)return;
  clearTimeout(steerTimer);$("left").classList.remove("active");$("right").classList.remove("active");
  if(steerContinuous)steerReq("/steer_hold"); steerDir=0;steerContinuous=false;
}
$("left").addEventListener("pointerdown",e=>pressSteer(e,-1));
$("right").addEventListener("pointerdown",e=>pressSteer(e,1));
["pointerup","pointercancel","lostpointercapture"].forEach(n=>{$("left").addEventListener(n,()=>releaseSteer(-1));$("right").addEventListener(n,()=>releaseSteer(1));});
$("center").onclick=()=>{if(armed&&!estop){steerDir=0;steerContinuous=false;steerReq("/steer_center");}};

let driveHeld=false;
function driveDown(e){
  if(!armed||estop)return; e.preventDefault(); driveHeld=true;$("drive").classList.add("active");
  try{$("drive").setPointerCapture(e.pointerId)}catch(_){ }
  req("/drive_on");
}
function driveUp(){
  if(!driveHeld)return;driveHeld=false;$("drive").classList.remove("active");req("/drive_off");
}
$("drive").addEventListener("pointerdown",driveDown);
["pointerup","pointercancel","lostpointercapture"].forEach(n=>$("drive").addEventListener(n,driveUp));

$("brake").addEventListener("pointerdown",e=>{if(!armed||estop)return;e.preventDefault();$("brake").classList.add("active");driveHeld=false;$("drive").classList.remove("active");req("/brake_now");});
["pointerup","pointercancel","lostpointercapture"].forEach(n=>$("brake").addEventListener(n,()=>$("brake").classList.remove("active")));

let speedTimer=null;
$("speed").addEventListener("input",()=>{
  const v=Number($("speed").value)/100;$("speedVal").textContent=v.toFixed(2)+" V";
  clearTimeout(speedTimer);speedTimer=setTimeout(()=>req("/speed_set?v="+v),70);
});

$("arm").onclick=()=>req("/arm");
$("disarm").onclick=()=>{driveHeld=false;$("drive").classList.remove("active");req("/disarm");};
$("estop").onclick=()=>{driveHeld=false;$("drive").classList.remove("active");estop=true;$("overlay").classList.add("show");req("/estop");};
$("reset").onclick=()=>req("/estop_reset");

function styleState(mode){
  const p=$("modePill");p.textContent=mode;p.className="pill "+((mode==="DRIVE"||mode==="ARMED")?"ok":(mode==="ESTOP"||mode==="RADIO_FAULT")?"bad":"warn");
  const l=$("loraPill");l.textContent="LORA "+lora;l.className="pill "+(lora==="LINK_OK"?"ok":(lora==="FAULT"||lora==="TIMEOUT")?"bad":"warn");
}
function processStatus(t){
  const p=t.split("|");if(p.length<10)return;
  const mode=p[0];armed=p[1]==="1";lora=p[2];estop=p[3]==="1";
  $("steerVal").textContent=Number(p[4]).toFixed(1)+"°";
  $("brakeVal").textContent=Number(p[5]).toFixed(1)+"°";
  $("svVal").textContent=Number(p[6]).toFixed(2)+" V";
  $("armVal").textContent=armed?"YES":"NO";
  if(document.activeElement!==$("speed")){const cv=Math.round(Number(p[7])*100);$("speed").value=cv;$("speedVal").textContent=Number(p[7]).toFixed(2)+" V";}
  styleState(mode);
  const lw=p[9]||"-";
  if(lw!=="-"){$("loraWarn").textContent=lw;$("loraWarn").classList.add("show");}
  else{$("loraWarn").classList.remove("show");}
  if(estop){$("reason").textContent=p[8]+" — E-STOP active. Reset does not re-arm; press ARM separately after the cause is cleared.";$("overlay").classList.add("show");}
  else{$("overlay").classList.remove("show");}
}

setInterval(()=>{
  req("/heartbeat").then(t=>{lostCount=0;$("lost").classList.remove("show");processStatus(t);}).catch(()=>{lostCount++;if(lostCount>=3)$("lost").classList.add("show");});
},250);

document.addEventListener("visibilitychange",()=>{if(document.hidden&&driveHeld){driveHeld=false;req("/drive_off");}});
window.addEventListener("pagehide",()=>{try{fetch("/disarm",{keepalive:true})}catch(_){ }});
</script>
</body>
</html>'''

HTML = HTML.replace("__MAX_CV__", str(int(THROTTLE_MAX_V * 100)))
HTML = HTML.replace("__DEFAULT_CV__", str(int(DEFAULT_DRIVE_V * 100)))
HTML = HTML.replace("__DEFAULT_V__", "%.2f" % DEFAULT_DRIVE_V)


# ============================================================
# HTTP HELPERS
# ============================================================

def send_all(sock, data):
    if isinstance(data, str):
        data = data.encode()

    while data:
        sent = sock.send(data)
        if sent <= 0:
            break
        data = data[sent:]


def response(client, body, content_type="text/plain", status="200 OK"):
    body_bytes = body.encode() if isinstance(body, str) else body

    header = (
        "HTTP/1.1 " + status + "\r\n"
        "Content-Type: " + content_type + "\r\n"
        "Cache-Control: no-store\r\n"
        "Content-Length: " + str(len(body_bytes)) + "\r\n"
        "Connection: close\r\n\r\n"
    )

    send_all(client, header)
    send_all(client, body_bytes)


def query_value(path, default=0.0):
    try:
        value = path.split("v=", 1)[1]
        value = value.split("&", 1)[0]
        return float(value)
    except:
        return default


def query_int(path, key, default=-1):
    try:
        value = path.split(key + "=", 1)[1]
        value = value.split("&", 1)[0]
        return int(value)
    except:
        return default


# ============================================================
# HTTP SERVER
# ============================================================

addr = socket.getaddrinfo("0.0.0.0", 80)[0][-1]
server = socket.socket()

try:
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
except:
    pass

server.bind(addr)
server.listen(2)
server.settimeout(0)

print("WEB SERVER READY:", "http://" + IP)
print("STEERING LIMIT: +/-90 deg (USER REQUESTED - VERIFY MECHANICAL CLEARANCE)")
print("THROTTLE MAX: 5.00 V (USER REQUESTED - NOT VALIDATED AGAINST PRIOR BLD FAULT)")
print("LORA TIMEOUT: WARNING ONLY; VALID CMD_STOP STILL LATCHES E-STOP")


# ============================================================
# MAIN LOOP
# ============================================================

try:
    while True:

        # Fast deterministic-ish cooperative tasks first.
        steering.update()
        mechanical_brake.update()

        try:
            throttle_update()
        except Exception as e:
            latch_estop("GP8630N I2C FAULT")
            print("DAC ERROR:", e)

        try:
            process_lora()
        except Exception as e:
            radio_fault = True
            radio_fault_text = str(e)
            radio_run_permit = False
            print("LORA WARNING - RECEIVER FAULT, VEHICLE CONTINUES:", e)

        update_drive_state()

        now = time.ticks_ms()

        # Browser / operator command watchdog.
        if armed:
            if time.ticks_diff(now, last_contact) > COMMAND_TIMEOUT_MS:
                latch_estop("RC HEARTBEAT TIMEOUT")

        # ----------------------------------------------------
        # HTTP request
        # ----------------------------------------------------
        try:
            client, remote = server.accept()

            try:
                request = client.recv(1024)

                if not request:
                    client.close()
                    continue

                request = request.decode("utf-8", "ignore")
                first_line = request.split("\r\n", 1)[0]
                parts = first_line.split(" ")

                if len(parts) < 2:
                    client.close()
                    continue

                path = parts[1]

                # --------------------------------------------
                # Page
                # --------------------------------------------
                if path == "/":
                    response(client, HTML, "text/html")

                # --------------------------------------------
                # Status / heartbeat
                # --------------------------------------------
                elif path.startswith("/heartbeat"):
                    if armed:
                        last_contact = time.ticks_ms()
                    response(client, status_text())

                # --------------------------------------------
                # Browser E-stop
                # --------------------------------------------
                elif path.startswith("/estop_reset"):
                    # Reset clears the latch but NEVER re-arms.
                    # If the remote mushroom is still sending STOP, it will
                    # immediately latch again on the next valid STOP packet.
                    estop_latched = False
                    estop_reason = ""
                    armed = False
                    safe_hardware_stop(immediate_dac=True)
                    response(client, "RESET_OK")

                elif path.startswith("/estop"):
                    latch_estop("WEB E-STOP PRESSED")
                    response(client, "ESTOP_LATCHED")

                # Everything below is blocked while E-stop is latched.
                elif estop_latched:
                    response(
                        client,
                        "ESTOP_LATCHED",
                        "text/plain",
                        "409 Conflict"
                    )

                # --------------------------------------------
                # Arm
                # --------------------------------------------
                elif path.startswith("/arm"):
                    safe_hardware_stop(immediate_dac=True)
                    armed = True
                    steer_cmd_seq = -1
                    last_contact = time.ticks_ms()
                    response(client, "ARMED")

                # --------------------------------------------
                # Disarm
                # --------------------------------------------
                elif path.startswith("/disarm"):
                    disarm_now("WEB DISARM")
                    response(client, "DISARMED")

                # --------------------------------------------
                # Drive voltage selection
                # --------------------------------------------
                elif path.startswith("/speed_set"):
                    value = query_value(path, selected_drive_voltage)
                    selected_drive_voltage = max(
                        0.0,
                        min(THROTTLE_MAX_V, value)
                    )

                    if drive_requested and bld_enabled and bld_brake_released:
                        throttle_target = selected_drive_voltage

                    if armed:
                        last_contact = time.ticks_ms()

                    response(client, "OK")

                # --------------------------------------------
                # Hold-forward dead-man drive
                # --------------------------------------------
                elif path.startswith("/drive_on"):
                    ok, msg = start_drive_request()

                    if ok:
                        last_contact = time.ticks_ms()
                        response(client, msg)
                    else:
                        response(client, msg, "text/plain", "409 Conflict")

                elif path.startswith("/drive_off"):
                    if armed:
                        request_throttle_zero_only("FORWARD RELEASE")
                        last_contact = time.ticks_ms()
                    else:
                        throttle_zero_immediate()

                    response(client, "THROTTLE_ZERO")

                # --------------------------------------------
                # Immediate brake
                # --------------------------------------------
                elif path.startswith("/brake_now"):
                    if armed:
                        brake_now("WEB BRAKE")
                        last_contact = time.ticks_ms()
                    response(client, "BRAKED")

                # --------------------------------------------
                # Steering
                # --------------------------------------------
                elif path.startswith("/steer_left"):
                    seq = query_int(path, "seq", -1)

                    if armed and (seq < 0 or seq > steer_cmd_seq):
                        if seq >= 0:
                            steer_cmd_seq = seq
                        steering.set_target(-STEER_LIMIT_DEG)
                        last_contact = time.ticks_ms()

                    response(client, "OK")

                elif path.startswith("/steer_right"):
                    seq = query_int(path, "seq", -1)

                    if armed and (seq < 0 or seq > steer_cmd_seq):
                        if seq >= 0:
                            steer_cmd_seq = seq
                        steering.set_target(STEER_LIMIT_DEG)
                        last_contact = time.ticks_ms()

                    response(client, "OK")

                elif path.startswith("/steer_nudge"):
                    seq = query_int(path, "seq", -1)
                    value = max(-10.0, min(10.0, query_value(path, 0.0)))

                    if armed and (seq < 0 or seq > steer_cmd_seq):
                        if seq >= 0:
                            steer_cmd_seq = seq
                        steering.nudge(value)
                        last_contact = time.ticks_ms()

                    response(client, "OK")

                elif path.startswith("/steer_hold"):
                    seq = query_int(path, "seq", -1)

                    if armed and (seq < 0 or seq > steer_cmd_seq):
                        if seq >= 0:
                            steer_cmd_seq = seq
                        steering.hold()
                        last_contact = time.ticks_ms()

                    response(client, "OK")

                elif path.startswith("/steer_center"):
                    seq = query_int(path, "seq", -1)

                    if armed and (seq < 0 or seq > steer_cmd_seq):
                        if seq >= 0:
                            steer_cmd_seq = seq
                        steering.set_target(0.0)
                        last_contact = time.ticks_ms()

                    response(client, "OK")

                else:
                    response(client, "NOT_FOUND", "text/plain", "404 Not Found")

            finally:
                try:
                    client.close()
                except:
                    pass

        except OSError:
            # Expected for non-blocking server.accept().
            pass

        time.sleep_us(120)


except KeyboardInterrupt:
    print("CTRL+C")

except Exception as e:
    print("FATAL ERROR:", e)

finally:
    # Software shutdown path. BLD digital control is forced safe first.
    try:
        throttle_zero_immediate()
    except:
        pass

    try:
        bld_brake_apply()
        bld_enable_stop()
    except:
        pass

    try:
        steering.hold()
        apply_mechanical_brake()
    except:
        pass

    # Briefly allow brake stepper to move toward apply before driver disable.
    shutdown_start = time.ticks_ms()

    try:
        while time.ticks_diff(time.ticks_ms(), shutdown_start) < 350:
            mechanical_brake.update()
            time.sleep_us(150)
    except:
        pass

    try:
        STEER_EN.value(TB_DISABLE_LEVEL)
        BRAKE_EN.value(TB_DISABLE_LEVEL)
    except:
        pass

    try:
        server.close()
    except:
        pass

    print("AUTOKART CONTROLLER STOPPED")
    print("BLD EN = STOP / BRK = APPLIED / SV = 0 V")
