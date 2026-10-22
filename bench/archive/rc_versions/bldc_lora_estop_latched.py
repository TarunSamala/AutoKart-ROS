import network
import socket
import time
import ujson
from machine import Pin, I2C, SoftSPI


# ============================================================
# AUTOKART - ESP32-S3
# BLD-750 TRACTION + GP8630N + LoRa WIRELESS E-STOP
#
# BENCH / WHEELS-OFF-GROUND TESTING ONLY
#
# E-STOP ACTION:
#   1. SV -> 0.00 V immediately
#   2. BRK -> APPLIED
#   3. EN  -> STOP
#   4. E-STOP latches in software
#   5. Web RUN / brake-release / throttle commands are blocked
#   6. Browser shows a red E-STOP warning
#
# IMPORTANT:
#   This is still a software-controlled safety channel.
#   Do not treat it as the final independent vehicle E-stop.
# ============================================================


# ============================================================
# CONFIGURATION
# ============================================================

SSID = "BLD750-Control"
PASSWORD = "12345678"


# ---------------- GP8630N ----------------

SDA_PIN = 8
SCL_PIN = 9

GP_ADDR = 0x58
REG_MODE = 0x01
REG_DAC = 0x02
MODE_0_10V = 0x1C


# ---------------- BLD-750 DIGITAL CONTROL ----------------
# Actual GPIOs used by this program.

EN_PIN = 1
BRK_PIN = 2
FR_PIN = 7


# ---------------- THROTTLE ----------------

MAX_VOLTAGE = 3.00

RAMP_STEP = 0.05
RAMP_DELAY_MS = 40


# ---------------- WEB WATCHDOG ----------------

COMMAND_TIMEOUT_MS = 1500


# ---------------- LoRa RA-02 / SX1278 ----------------

LORA_SCK_PIN = 14
LORA_MOSI_PIN = 13
LORA_MISO_PIN = 15

LORA_CS_PIN = 16
LORA_RESET_PIN = 17
LORA_DIO0_PIN = 18

LORA_FREQUENCY = 433_000_000

# TX heartbeat = 100 ms.
# More than 350 ms without a valid packet -> latched safe stop.
LORA_LINK_TIMEOUT_MS = 350

# Do not declare the E-stop link healthy from one RUN frame.
GOOD_RUN_PACKETS_REQUIRED = 5


# ---------------- LoRa APPLICATION PROTOCOL ----------------

MAGIC0 = 0xA5
MAGIC1 = 0x5A

PROTOCOL_VERSION = 1
EXPECTED_TRANSMITTER_ID = 0x31

CMD_RUN = 0x11
CMD_STOP = 0xEE


# ============================================================
# GLOBAL SAFETY STATE
# ============================================================

estop_latched = False
estop_reason = ""

lora_link_healthy = False
lora_current_session = None
lora_last_sequence = None
lora_last_valid_packet_ms = None
lora_good_run_count = 0


# ============================================================
# BLD CONTROL OUTPUTS
# ============================================================

en_pin = Pin(EN_PIN, Pin.OUT)
brk_pin = Pin(BRK_PIN, Pin.OUT)
fr_pin = Pin(FR_PIN, Pin.OUT)

en_running = False
brake_released = False
direction = "CW"


def enable_run():
    global en_running

    # NPN ON -> BLD EN pulled to COM -> RUN.
    en_pin.value(1)
    en_running = True
    print("EN -> RUN")


def enable_stop():
    global en_running

    # NPN OFF -> BLD EN high/open -> STOP.
    en_pin.value(0)
    en_running = False
    print("EN -> STOP")


def brake_release():
    global brake_released

    # NPN ON -> BLD BRK pulled to COM -> brake released.
    brk_pin.value(1)
    brake_released = True
    print("BRK -> RELEASED")


def brake_apply():
    global brake_released

    # NPN OFF -> BLD BRK high/open -> quick brake.
    brk_pin.value(0)
    brake_released = False
    print("BRK -> APPLIED")


def direction_cw():
    global direction

    fr_pin.value(0)
    direction = "CW"
    print("DIRECTION -> CW")


def direction_ccw():
    global direction

    fr_pin.value(1)
    direction = "CCW"
    print("DIRECTION -> CCW")


# ============================================================
# CRITICAL SAFE BOOT
# ============================================================

# Force safe digital outputs before I2C, Wi-Fi or LoRa setup.
enable_stop()
brake_apply()
direction_cw()


# ============================================================
# GP8630N
# ============================================================

i2c = I2C(
    0,
    sda=Pin(SDA_PIN),
    scl=Pin(SCL_PIN),
    freq=100000
)

devices = i2c.scan()
print("I2C:", [hex(x) for x in devices])

if GP_ADDR not in devices:
    enable_stop()
    brake_apply()
    raise RuntimeError("GP8630N NOT FOUND AT 0x58")

print("GP8630N detected:", hex(GP_ADDR))

i2c.writeto_mem(
    GP_ADDR,
    REG_MODE,
    bytes([MODE_0_10V])
)

time.sleep_ms(100)


current_voltage = 0.0
target_voltage = 0.0
last_command_ms = time.ticks_ms()


def write_voltage(voltage):
    global current_voltage

    if voltage < 0.0:
        voltage = 0.0

    if voltage > MAX_VOLTAGE:
        voltage = MAX_VOLTAGE

    dac = int((voltage / 10.0) * 65535)

    low = dac & 0xFF
    high = (dac >> 8) & 0xFF

    i2c.writeto_mem(
        GP_ADDR,
        REG_DAC,
        bytes([low, high])
    )

    current_voltage = voltage


def safe_stop(reason="SAFE STOP"):
    global target_voltage
    global current_voltage

    print()
    print("!!!", reason, "!!!")

    # Remove throttle command first.
    target_voltage = 0.0

    # Try to force the analog speed command to zero immediately.
    try:
        write_voltage(0.0)
    except Exception as e:
        print("DAC ZERO FAILED:", e)

    # Independent BLD control inputs then command brake + disable.
    brake_apply()
    enable_stop()

    print("SV -> 0V")
    print("BRK -> APPLIED")
    print("EN -> STOP")
    print()


def latch_estop(reason):
    global estop_latched
    global estop_reason

    # Latch first so no later browser command in this loop can re-enable drive.
    if not estop_latched:
        estop_latched = True
        estop_reason = reason

        print()
        print("########################################")
        print("###      E-STOP LATCHED             ###")
        print("###", reason)
        print("########################################")
        print()

    safe_stop("E-STOP: " + reason)


# DAC safe start.
write_voltage(0.0)
print("SV -> 0.00 V")


# ============================================================
# SX1278 LoRa DRIVER
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

        self.spi.write(
            bytes((
                address | 0x80,
                value & 0xFF
            ))
        )

        self.cs.value(1)

    def read_reg(self, address):
        self.cs.value(0)

        self.spi.write(
            bytes((address & 0x7F,))
        )

        value = self.spi.read(1, 0x00)[0]

        self.cs.value(1)

        return value

    def read_fifo(self, length):
        self.cs.value(0)

        self.spi.write(
            bytes((self.REG_FIFO & 0x7F,))
        )

        data = self.spi.read(length, 0x00)

        self.cs.value(1)

        return data

    def reset_radio(self):
        self.reset.value(0)
        time.sleep_ms(10)

        self.reset.value(1)
        time.sleep_ms(20)

    def set_frequency(self, frequency):
        frf = int(
            frequency * 524288 /
            32_000_000
        )

        self.write_reg(
            self.REG_FRF_MSB,
            (frf >> 16) & 0xFF
        )

        self.write_reg(
            self.REG_FRF_MID,
            (frf >> 8) & 0xFF
        )

        self.write_reg(
            self.REG_FRF_LSB,
            frf & 0xFF
        )

    def standby(self):
        self.write_reg(
            self.REG_OP_MODE,
            self.MODE_LONG_RANGE |
            self.MODE_STANDBY
        )

    def begin(self):
        self.cs.value(1)
        self.reset_radio()

        version = self.read_reg(
            self.REG_VERSION
        )

        print("SX1278 VERSION:", hex(version))

        if version != 0x12:
            raise OSError(
                "RA-02 not detected. Expected 0x12, got 0x%02X"
                % version
            )

        self.write_reg(
            self.REG_OP_MODE,
            self.MODE_LONG_RANGE |
            self.MODE_SLEEP
        )

        time.sleep_ms(10)

        self.set_frequency(
            self.frequency
        )

        self.write_reg(
            self.REG_FIFO_TX_BASE_ADDR,
            0x00
        )

        self.write_reg(
            self.REG_FIFO_RX_BASE_ADDR,
            0x00
        )

        self.write_reg(
            self.REG_LNA,
            self.read_reg(self.REG_LNA) | 0x03
        )

        # BW125 / CR4/5 / explicit header.
        self.write_reg(
            self.REG_MODEM_CONFIG_1,
            0x72
        )

        # SF7 / hardware CRC.
        self.write_reg(
            self.REG_MODEM_CONFIG_2,
            0x74
        )

        self.write_reg(
            self.REG_MODEM_CONFIG_3,
            0x04
        )

        self.write_reg(
            self.REG_PREAMBLE_MSB,
            0x00
        )

        self.write_reg(
            self.REG_PREAMBLE_LSB,
            0x08
        )

        self.write_reg(
            self.REG_SYNC_WORD,
            0x12
        )

        self.write_reg(
            self.REG_IRQ_FLAGS,
            0xFF
        )

        self.standby()

        print("RA-02 INITIALIZED")

    def start_receive(self):
        # DIO0 = RxDone.
        self.write_reg(
            self.REG_DIO_MAPPING_1,
            0x00
        )

        self.write_reg(
            self.REG_IRQ_FLAGS,
            0xFF
        )

        self.write_reg(
            self.REG_OP_MODE,
            self.MODE_LONG_RANGE |
            self.MODE_RX_CONTINUOUS
        )

    def receive(self):
        if self.dio0.value() == 0:
            return None

        flags = self.read_reg(
            self.REG_IRQ_FLAGS
        )

        self.write_reg(
            self.REG_IRQ_FLAGS,
            flags
        )

        if not (
            flags &
            self.IRQ_RX_DONE
        ):
            return None

        if (
            flags &
            self.IRQ_PAYLOAD_CRC_ERROR
        ):
            print("RADIO CRC ERROR")
            return None

        length = self.read_reg(
            self.REG_RX_NB_BYTES
        )

        fifo_address = self.read_reg(
            self.REG_FIFO_RX_CURRENT_ADDR
        )

        self.write_reg(
            self.REG_FIFO_ADDR_PTR,
            fifo_address
        )

        return self.read_fifo(length)


def crc16(data):
    crc = 0xFFFF

    for byte in data:
        crc ^= byte << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = (
                    (crc << 1) ^
                    0x1021
                ) & 0xFFFF
            else:
                crc = (
                    crc << 1
                ) & 0xFFFF

    return crc


def parse_lora_packet(packet):
    if packet is None:
        return None

    if len(packet) != 11:
        print("INVALID LoRa LENGTH:", len(packet))
        return None

    if (
        packet[0] != MAGIC0 or
        packet[1] != MAGIC1
    ):
        return None

    if packet[2] != PROTOCOL_VERSION:
        print("WRONG LoRa PROTOCOL")
        return None

    if packet[3] != EXPECTED_TRANSMITTER_ID:
        print("WRONG LoRa TX ID")
        return None

    command = packet[4]

    if command not in (
        CMD_RUN,
        CMD_STOP
    ):
        return None

    session = (
        (packet[5] << 8) |
        packet[6]
    )

    sequence = (
        (packet[7] << 8) |
        packet[8]
    )

    received_crc = (
        (packet[9] << 8) |
        packet[10]
    )

    calculated_crc = crc16(
        packet[:9]
    )

    if received_crc != calculated_crc:
        print("LoRa APPLICATION CRC ERROR")
        return None

    return (
        command,
        session,
        sequence
    )


def sequence_is_fresh(new_seq, old_seq):
    if old_seq is None:
        return True

    difference = (
        new_seq -
        old_seq
    ) & 0xFFFF

    return (
        0 <
        difference <
        0x8000
    )


# ============================================================
# LoRa HARDWARE STARTUP
# ============================================================

lora_spi = SoftSPI(
    baudrate=2_000_000,
    polarity=0,
    phase=0,
    sck=Pin(LORA_SCK_PIN),
    mosi=Pin(LORA_MOSI_PIN),
    miso=Pin(LORA_MISO_PIN)
)

lora_cs = Pin(
    LORA_CS_PIN,
    Pin.OUT,
    value=1
)

lora_reset = Pin(
    LORA_RESET_PIN,
    Pin.OUT,
    value=1
)

lora_dio0 = Pin(
    LORA_DIO0_PIN,
    Pin.IN
)

radio = SX1278(
    lora_spi,
    lora_cs,
    lora_reset,
    lora_dio0,
    LORA_FREQUENCY
)

try:
    radio.begin()
    radio.start_receive()

except Exception as e:
    # If the E-stop receiver itself fails to initialize, traction remains locked.
    latch_estop("LORA INITIALIZATION FAILURE: %s" % e)
    raise


# ============================================================
# LoRa SAFETY PROCESSOR
# ============================================================

def process_lora():
    global lora_link_healthy
    global lora_current_session
    global lora_last_sequence
    global lora_last_valid_packet_ms
    global lora_good_run_count

    packet = radio.receive()

    if packet is not None:
        parsed = parse_lora_packet(packet)

        if parsed is not None:
            command, session, sequence = parsed

            # New transmitter boot/session.
            if lora_current_session != session:
                print()
                print("NEW LoRa TX SESSION:", session)

                lora_current_session = session
                lora_last_sequence = None
                lora_good_run_count = 0
                lora_link_healthy = False

                # Do not latch only because the transmitter rebooted,
                # but force the traction controller safe while link
                # health is re-established.
                safe_stop("NEW LORA TRANSMITTER SESSION")

            if sequence_is_fresh(
                sequence,
                lora_last_sequence
            ):
                lora_last_sequence = sequence
                lora_last_valid_packet_ms = time.ticks_ms()

                if command == CMD_STOP:
                    print("RX E-STOP", sequence)

                    lora_good_run_count = 0
                    lora_link_healthy = False

                    latch_estop(
                        "REMOTE MUSHROOM PRESSED"
                    )

                elif command == CMD_RUN:
                    print("RX RUN", sequence)

                    if not estop_latched:
                        lora_good_run_count += 1

                        if (
                            lora_good_run_count >=
                            GOOD_RUN_PACKETS_REQUIRED
                        ):
                            if not lora_link_healthy:
                                print(
                                    "LoRa E-STOP LINK HEALTHY"
                                )

                            lora_link_healthy = True

            else:
                print(
                    "STALE/DUPLICATE LoRa:",
                    sequence
                )

    # Once at least one valid frame has been seen, loss of the
    # safety heartbeat is itself a latched stop.
    if lora_last_valid_packet_ms is not None:
        packet_age = time.ticks_diff(
            time.ticks_ms(),
            lora_last_valid_packet_ms
        )

        if packet_age > LORA_LINK_TIMEOUT_MS:
            if not estop_latched:
                lora_link_healthy = False
                lora_good_run_count = 0

                latch_estop(
                    "LORA HEARTBEAT TIMEOUT"
                )


def drive_permission_ok():
    return (
        (not estop_latched) and
        lora_link_healthy
    )


# ============================================================
# WI-FI ACCESS POINT
# ============================================================

ap = network.WLAN(network.AP_IF)

ap.active(True)

ap.config(
    essid=SSID,
    password=PASSWORD
)

while not ap.active():
    time.sleep_ms(100)


print()
print("==========================================")
print(" AUTOKART BLD-750 + LoRa E-STOP")
print("==========================================")
print("SSID:", SSID)
print("Password:", PASSWORD)
print("IP:", ap.ifconfig()[0])
print("MAX SV:", MAX_VOLTAGE, "V")
print("EN GPIO:", EN_PIN)
print("BRK GPIO:", BRK_PIN)
print("F/R GPIO:", FR_PIN)
print("LoRa:", LORA_FREQUENCY, "Hz")
print("E-stop timeout:", LORA_LINK_TIMEOUT_MS, "ms")
print("==========================================")
print()


# ============================================================
# HTML
# ============================================================

HTML = """<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>AutoKart BLD750</title>

<style>
body {
    background: #111;
    color: white;
    font-family: Arial;
    text-align: center;
    margin: 0;
    padding: 20px;
}

.container {
    max-width: 600px;
    margin: auto;
}

h1 {
    margin-bottom: 5px;
}

.subtitle {
    color: #888;
    margin-bottom: 20px;
}

.voltage {
    font-size: 52px;
    font-weight: bold;
    margin: 20px;
}

.slider {
    width: 95%;
    margin: 25px 0 35px 0;
}

.row {
    margin: 15px 0;
}

button {
    width: 42%;
    padding: 17px;
    margin: 6px;
    border: none;
    border-radius: 10px;
    font-size: 18px;
}

.green {
    background: #24a35a;
    color: white;
}

.red {
    background: #d43c3c;
    color: white;
}

.blue {
    background: #2878cf;
    color: white;
}

.orange {
    background: #d68125;
    color: white;
}

.safe {
    width: 90%;
    background: #ff2020;
    color: white;
    font-weight: bold;
    font-size: 22px;
}

.label {
    font-size: 18px;
    color: #aaa;
    margin-top: 20px;
}

#status {
    font-size: 24px;
    margin: 15px;
}

#lora {
    font-size: 18px;
    margin: 10px;
    color: #ffb347;
}

.warning {
    margin-top: 30px;
    color: #ff7777;
}

#estopOverlay {
    display: none;
    position: fixed;
    z-index: 9999;
    left: 0;
    right: 0;
    top: 0;
    bottom: 0;
    background: rgba(180, 0, 0, 0.96);
    color: white;
    align-items: center;
    justify-content: center;
    flex-direction: column;
    padding: 25px;
}

#estopOverlay .big {
    font-size: 46px;
    font-weight: bold;
    margin-bottom: 20px;
}

#estopReason {
    font-size: 24px;
    max-width: 600px;
}

#estopAction {
    margin-top: 25px;
    font-size: 19px;
    line-height: 1.5;
}
</style>
</head>

<body>

<div id="estopOverlay">
    <div class="big">E-STOP ACTIVE</div>
    <div id="estopReason">REMOTE STOP</div>
    <div id="estopAction">
        BLDC command forced to 0 V<br>
        BLD brake applied<br>
        BLD enable stopped<br><br>
        E-STOP IS LATCHED
    </div>
</div>

<div class="container">

<h1>AutoKart Traction Bench</h1>

<div class="subtitle">
ESP32-S3 · GP8630N · BLD-750 · LoRa E-STOP
</div>

<div id="status">
SAFE
</div>

<div id="lora">
LoRa E-STOP LINK: WAITING
</div>

<div class="voltage">
<span id="voltage">0.00</span> V
</div>

<input
type="range"
min="0"
max="300"
value="0"
step="5"
class="slider"
id="slider"
oninput="speedChanged()"
>

<div class="label">ENABLE</div>

<div class="row">
<button class="green" onclick="enableRun()">EN RUN</button>
<button class="red" onclick="enableStop()">EN STOP</button>
</div>

<div class="label">BRAKE</div>

<div class="row">
<button class="green" onclick="brakeRelease()">BRK RELEASE</button>
<button class="red" onclick="brakeApply()">BRK APPLY</button>
</div>

<div class="label">DIRECTION</div>

<div class="row">
<button class="blue" onclick="setCW()">CW</button>
<button class="orange" onclick="setCCW()">CCW</button>
</div>

<br>

<button class="safe" onclick="safeStop()">
SAFE STOP
</button>

<div class="warning">
BENCH TEST ONLY<br>
WHEELS OFF GROUND
</div>

</div>

<script>
let enRunning = false;
let brakeReleased = false;
let estopShown = false;

function setStatus(text) {
    document.getElementById("status").innerHTML = text;
}

function zeroSlider() {
    document.getElementById("slider").value = 0;
    document.getElementById("voltage").innerHTML = "0.00";
}

function showEstop(reason) {
    enRunning = false;
    brakeReleased = false;
    zeroSlider();

    document.getElementById("estopReason").innerText = reason;
    document.getElementById("estopOverlay").style.display = "flex";

    if (!estopShown) {
        estopShown = true;
        console.warn("AUTOKART E-STOP:", reason);
    }
}

function speedChanged() {
    let slider = document.getElementById("slider");
    let voltage = slider.value / 100;

    document.getElementById("voltage").innerHTML =
        voltage.toFixed(2);

    if (enRunning && brakeReleased) {
        fetch("/speed?v=" + voltage)
        .then(function(response) {
            if (!response.ok) {
                zeroSlider();
                setStatus("COMMAND BLOCKED");
            }
        })
        .catch(function() {
            zeroSlider();
        });
    }
}

function enableRun() {
    fetch("/en?state=run")
    .then(function(response) {
        if (response.ok) {
            enRunning = true;
            setStatus("EN ENABLED");
        } else {
            enRunning = false;
            setStatus("RUN BLOCKED");
        }
    });
}

function enableStop() {
    enRunning = false;
    zeroSlider();

    fetch("/en?state=stop");

    setStatus("EN STOPPED");
}

function brakeRelease() {
    fetch("/brake?state=release")
    .then(function(response) {
        if (response.ok) {
            brakeReleased = true;
            setStatus("BRAKE RELEASED");
        } else {
            brakeReleased = false;
            setStatus("BRAKE RELEASE BLOCKED");
        }
    });
}

function brakeApply() {
    brakeReleased = false;
    zeroSlider();

    fetch("/brake?state=apply");

    setStatus("BRAKE APPLIED");
}

function setCW() {
    fetch("/direction?d=cw")
    .then(function(response) {
        setStatus(
            response.ok ?
            "DIRECTION CW" :
            "STOP FIRST"
        );
    });
}

function setCCW() {
    fetch("/direction?d=ccw")
    .then(function(response) {
        setStatus(
            response.ok ?
            "DIRECTION CCW" :
            "STOP FIRST"
        );
    });
}

function safeStop() {
    enRunning = false;
    brakeReleased = false;

    zeroSlider();

    fetch("/safe_stop");

    setStatus("SAFE STOP");
}

function pollStatus() {
    fetch("/status", {cache: "no-store"})
    .then(function(response) {
        return response.json();
    })
    .then(function(data) {
        if (data.estop_latched) {
            showEstop(data.estop_reason || "E-STOP");
            return;
        }

        document.getElementById("lora").innerText =
            data.lora_link_healthy ?
            "LoRa E-STOP LINK: HEALTHY" :
            "LoRa E-STOP LINK: NOT READY";

        document.getElementById("lora").style.color =
            data.lora_link_healthy ?
            "#4ee07d" :
            "#ffb347";
    })
    .catch(function() {
        document.getElementById("lora").innerText =
            "LoRa E-STOP LINK: STATUS LOST";

        document.getElementById("lora").style.color =
            "#ff5555";
    });
}

/*
Browser heartbeat.
This is separate from the LoRa E-stop heartbeat.
*/
setInterval(
    function() {
        fetch("/heartbeat").catch(function() {});
    },
    400
);

/*
Poll controller state so an E-stop warning appears
even if the user is not touching any controls.
*/
setInterval(
    pollStatus,
    200
);

pollStatus();

window.addEventListener(
    "beforeunload",
    function() {
        fetch("/safe_stop");
    }
);
</script>
</body>
</html>
"""


# ============================================================
# HTTP HELPERS
# ============================================================

def send_response(
    client,
    body,
    content_type="text/html",
    status="200 OK"
):
    body_bytes = body.encode()

    header = (
        "HTTP/1.1 " + status + "\r\n"
        "Content-Type: " + content_type + "\r\n"
        "Cache-Control: no-store\r\n"
        "Content-Length: " + str(len(body_bytes)) + "\r\n"
        "Connection: close\r\n"
        "\r\n"
    )

    client.send(header.encode())
    client.send(body_bytes)


def get_voltage_from_request(request):
    try:
        marker = "v="
        position = request.find(marker)

        if position == -1:
            return 0.0

        value = request[
            position + 2:
        ]

        value = value.split(" ")[0]
        value = value.split("&")[0]

        voltage = float(value)

        return max(
            0.0,
            min(
                MAX_VOLTAGE,
                voltage
            )
        )

    except:
        return 0.0


def status_json():
    return ujson.dumps({
        "estop_latched": estop_latched,
        "estop_reason": estop_reason,
        "lora_link_healthy": lora_link_healthy,
        "en_running": en_running,
        "brake_released": brake_released,
        "current_voltage": current_voltage,
        "target_voltage": target_voltage,
        "direction": direction
    })


# ============================================================
# WEB SERVER
# ============================================================

addr = socket.getaddrinfo(
    "0.0.0.0",
    80
)[0][-1]

server = socket.socket()

server.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_REUSEADDR,
    1
)

server.bind(addr)
server.listen(2)
server.settimeout(0.03)

print("Web server ready")
print("Open: http://" + ap.ifconfig()[0])


# ============================================================
# MAIN LOOP
# ============================================================

try:
    while True:

        # ====================================================
        # 1. PROCESS E-STOP FIRST
        # ====================================================

        process_lora()


        # ====================================================
        # 2. WEB REQUEST
        # ====================================================

        try:
            client, address = server.accept()

            try:
                client.settimeout(0.05)
            except:
                pass

            request = client.recv(1024)

            request = request.decode(
                "utf-8",
                "ignore"
            )


            # ---------------- STATUS ----------------

            if "GET /status" in request:
                send_response(
                    client,
                    status_json(),
                    "application/json"
                )


            # ---------------- SAFE STOP ----------------

            elif "GET /safe_stop" in request:
                safe_stop(
                    "WEB SAFE STOP"
                )

                last_command_ms = time.ticks_ms()

                send_response(
                    client,
                    "SAFE",
                    "text/plain"
                )


            # ---------------- ENABLE RUN ----------------

            elif "GET /en?state=run" in request:

                if not drive_permission_ok():
                    safe_stop(
                        "RUN BLOCKED BY E-STOP INTERLOCK"
                    )

                    send_response(
                        client,
                        "E-STOP / LORA LINK NOT READY",
                        "text/plain",
                        "423 Locked"
                    )

                else:
                    enable_run()

                    send_response(
                        client,
                        "EN RUN",
                        "text/plain"
                    )

                last_command_ms = time.ticks_ms()


            # ---------------- ENABLE STOP ----------------

            elif "GET /en?state=stop" in request:
                target_voltage = 0.0

                try:
                    write_voltage(0.0)
                except:
                    pass

                enable_stop()

                last_command_ms = time.ticks_ms()

                send_response(
                    client,
                    "EN STOP",
                    "text/plain"
                )


            # ---------------- BRAKE RELEASE ----------------

            elif "GET /brake?state=release" in request:

                if not drive_permission_ok():
                    brake_apply()

                    send_response(
                        client,
                        "E-STOP / LORA LINK NOT READY",
                        "text/plain",
                        "423 Locked"
                    )

                elif current_voltage <= 0.01:
                    brake_release()

                    send_response(
                        client,
                        "BRAKE RELEASED",
                        "text/plain"
                    )

                else:
                    send_response(
                        client,
                        "THROTTLE MUST BE ZERO",
                        "text/plain",
                        "409 Conflict"
                    )

                last_command_ms = time.ticks_ms()


            # ---------------- BRAKE APPLY ----------------

            elif "GET /brake?state=apply" in request:
                target_voltage = 0.0

                try:
                    write_voltage(0.0)
                except:
                    pass

                brake_apply()

                last_command_ms = time.ticks_ms()

                send_response(
                    client,
                    "BRAKE APPLIED",
                    "text/plain"
                )


            # ---------------- CW ----------------

            elif "GET /direction?d=cw" in request:

                if (
                    current_voltage <= 0.01 and
                    target_voltage <= 0.01 and
                    not en_running and
                    not brake_released
                ):
                    direction_cw()

                    send_response(
                        client,
                        "CW",
                        "text/plain"
                    )

                else:
                    send_response(
                        client,
                        "STOP + BRAKE REQUIRED",
                        "text/plain",
                        "409 Conflict"
                    )

                last_command_ms = time.ticks_ms()


            # ---------------- CCW ----------------

            elif "GET /direction?d=ccw" in request:

                if (
                    current_voltage <= 0.01 and
                    target_voltage <= 0.01 and
                    not en_running and
                    not brake_released
                ):
                    direction_ccw()

                    send_response(
                        client,
                        "CCW",
                        "text/plain"
                    )

                else:
                    send_response(
                        client,
                        "STOP + BRAKE REQUIRED",
                        "text/plain",
                        "409 Conflict"
                    )

                last_command_ms = time.ticks_ms()


            # ---------------- SPEED ----------------

            elif "GET /speed" in request:
                voltage = get_voltage_from_request(
                    request
                )

                if not drive_permission_ok():
                    target_voltage = 0.0
                    safe_stop(
                        "THROTTLE BLOCKED BY E-STOP INTERLOCK"
                    )

                    send_response(
                        client,
                        "E-STOP / LORA LINK NOT READY",
                        "text/plain",
                        "423 Locked"
                    )

                elif (
                    en_running and
                    brake_released
                ):
                    target_voltage = voltage

                    print(
                        "TARGET SV ->",
                        target_voltage,
                        "V"
                    )

                    send_response(
                        client,
                        "OK",
                        "text/plain"
                    )

                else:
                    target_voltage = 0.0

                    send_response(
                        client,
                        "EN + BRK INTERLOCK",
                        "text/plain",
                        "409 Conflict"
                    )

                last_command_ms = time.ticks_ms()


            # ---------------- HEARTBEAT ----------------

            elif "GET /heartbeat" in request:
                last_command_ms = time.ticks_ms()

                send_response(
                    client,
                    "OK",
                    "text/plain"
                )


            # ---------------- PAGE ----------------

            else:
                send_response(
                    client,
                    HTML
                )

            client.close()

        except OSError:
            pass


        # ====================================================
        # 3. E-STOP ALWAYS OVERRIDES EVERYTHING
        # ====================================================

        if estop_latched:
            # Do not repeatedly write every loop unless something
            # somehow changed after the latch.
            if (
                en_running or
                brake_released or
                target_voltage > 0.01 or
                current_voltage > 0.01
            ):
                safe_stop(
                    "E-STOP OVERRIDE"
                )


        # ====================================================
        # 4. BROWSER COMMUNICATION WATCHDOG
        # ====================================================

        dangerous_state = (
            en_running or
            brake_released or
            target_voltage > 0.01 or
            current_voltage > 0.01
        )

        if dangerous_state:
            elapsed = time.ticks_diff(
                time.ticks_ms(),
                last_command_ms
            )

            if elapsed > COMMAND_TIMEOUT_MS:
                safe_stop(
                    "WEB COMMUNICATION TIMEOUT"
                )

                last_command_ms = time.ticks_ms()


        # ====================================================
        # 5. THROTTLE RAMP
        # ====================================================

        try:
            # No throttle movement is allowed unless the
            # E-stop link is healthy and not latched.
            if not drive_permission_ok():
                if (
                    target_voltage > 0.0 or
                    current_voltage > 0.0
                ):
                    target_voltage = 0.0
                    write_voltage(0.0)

            else:
                if current_voltage < target_voltage:
                    new_voltage = min(
                        current_voltage + RAMP_STEP,
                        target_voltage
                    )

                    write_voltage(
                        new_voltage
                    )

                elif current_voltage > target_voltage:
                    new_voltage = max(
                        current_voltage - RAMP_STEP,
                        target_voltage
                    )

                    write_voltage(
                        new_voltage
                    )

        except Exception as e:
            print("DAC ERROR:", e)

            latch_estop(
                "GP8630N I2C FAULT"
            )

        time.sleep_ms(
            RAMP_DELAY_MS
        )


except KeyboardInterrupt:
    print("CTRL+C")


except Exception as e:
    print("FATAL ERROR:", e)


finally:
    safe_stop(
        "PROGRAM EXIT"
    )

    try:
        server.close()
    except:
        pass

    print("CONTROLLER STOPPED")
