from machine import Pin, SoftSPI, unique_id
import time


# ============================================================
# AUTOKART WIRELESS E-STOP TRANSMITTER
# Raspberry Pi Pico + RA-02 + LAY37
# ============================================================


# ===================== PINS ================================

SCK_PIN = 18
MOSI_PIN = 19
MISO_PIN = 16

CS_PIN = 17
RESET_PIN = 20
DIO0_PIN = 21

ESTOP_PIN = 22


# =================== RADIO SETTINGS ========================

FREQUENCY = 433_000_000

# Send status every 100 ms
HEARTBEAT_MS = 100

# After button becomes healthy, wait before RUN
RELEASE_STABLE_MS = 300

# Immediate STOP burst
STOP_BURST_COUNT = 4
STOP_BURST_GAP_MS = 30


# ==================== PROTOCOL =============================

MAGIC0 = 0xA5
MAGIC1 = 0x5A

VERSION = 0x01
TRANSMITTER_ID = 0x31

CMD_RUN = 0x11
CMD_STOP = 0xEE


# ============================================================
# SX1278 DRIVER
# ============================================================

class SX1278:

    REG_FIFO = 0x00
    REG_OP_MODE = 0x01

    REG_FRF_MSB = 0x06
    REG_FRF_MID = 0x07
    REG_FRF_LSB = 0x08

    REG_PA_CONFIG = 0x09
    REG_OCP = 0x0B
    REG_LNA = 0x0C

    REG_FIFO_ADDR_PTR = 0x0D
    REG_FIFO_TX_BASE_ADDR = 0x0E
    REG_FIFO_RX_BASE_ADDR = 0x0F

    REG_IRQ_FLAGS = 0x12

    REG_MODEM_CONFIG_1 = 0x1D
    REG_MODEM_CONFIG_2 = 0x1E

    REG_PREAMBLE_MSB = 0x20
    REG_PREAMBLE_LSB = 0x21

    REG_PAYLOAD_LENGTH = 0x22
    REG_MODEM_CONFIG_3 = 0x26

    REG_SYNC_WORD = 0x39
    REG_DIO_MAPPING_1 = 0x40

    REG_VERSION = 0x42


    MODE_LONG_RANGE = 0x80

    MODE_SLEEP = 0x00
    MODE_STANDBY = 0x01
    MODE_TX = 0x03

    IRQ_TX_DONE = 0x08


    def __init__(
        self,
        spi,
        cs,
        reset,
        dio0,
        frequency
    ):

        self.spi = spi
        self.cs = cs
        self.reset = reset
        self.dio0 = dio0
        self.frequency = frequency


    # --------------------------------------------------------

    def write_reg(self, address, value):

        self.cs.value(0)

        self.spi.write(
            bytes((
                address | 0x80,
                value & 0xFF
            ))
        )

        self.cs.value(1)


    # --------------------------------------------------------

    def read_reg(self, address):

        self.cs.value(0)

        self.spi.write(
            bytes((
                address & 0x7F,
            ))
        )

        value = self.spi.read(
            1,
            0x00
        )[0]

        self.cs.value(1)

        return value


    # --------------------------------------------------------

    def write_fifo(self, data):

        self.cs.value(0)

        self.spi.write(
            bytes((
                self.REG_FIFO | 0x80,
            ))
        )

        self.spi.write(data)

        self.cs.value(1)


    # --------------------------------------------------------

    def reset_radio(self):

        self.reset.value(0)

        time.sleep_ms(10)

        self.reset.value(1)

        time.sleep_ms(30)


    # --------------------------------------------------------

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


    # --------------------------------------------------------

    def standby(self):

        self.write_reg(
            self.REG_OP_MODE,
            self.MODE_LONG_RANGE |
            self.MODE_STANDBY
        )


    # --------------------------------------------------------

    def begin(self):

        self.cs.value(1)

        self.reset_radio()

        version = self.read_reg(
            self.REG_VERSION
        )

        print(
            "SX1278 VERSION:",
            hex(version)
        )

        if version != 0x12:

            raise OSError(
                "RA-02 NOT FOUND - got 0x%02X"
                % version
            )


        # LoRa sleep
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


        # LNA boost
        self.write_reg(
            self.REG_LNA,
            self.read_reg(
                self.REG_LNA
            ) | 0x03
        )


        # BW 125 kHz
        # CR 4/5
        # explicit header
        self.write_reg(
            self.REG_MODEM_CONFIG_1,
            0x72
        )


        # SF7
        # radio CRC ON
        self.write_reg(
            self.REG_MODEM_CONFIG_2,
            0x74
        )


        # AGC ON
        self.write_reg(
            self.REG_MODEM_CONFIG_3,
            0x04
        )


        # Preamble 8
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


        # Moderate TX power for bench testing
        self.write_reg(
            self.REG_PA_CONFIG,
            0x8B
        )


        self.write_reg(
            self.REG_OCP,
            0x2B
        )


        self.write_reg(
            self.REG_IRQ_FLAGS,
            0xFF
        )


        self.standby()

        print(
            "RA-02 INITIALIZED"
        )


    # --------------------------------------------------------
    # ROBUST SEND
    # Polls IRQ register instead of relying only on DIO0
    # --------------------------------------------------------

    def send(self, payload):

        self.standby()

        time.sleep_ms(2)


        # clear all IRQs
        self.write_reg(
            self.REG_IRQ_FLAGS,
            0xFF
        )


        # DIO0 -> TxDone
        self.write_reg(
            self.REG_DIO_MAPPING_1,
            0x40
        )


        self.write_reg(
            self.REG_FIFO_ADDR_PTR,
            0x00
        )


        self.write_fifo(
            payload
        )


        self.write_reg(
            self.REG_PAYLOAD_LENGTH,
            len(payload)
        )


        # TX mode
        self.write_reg(
            self.REG_OP_MODE,
            self.MODE_LONG_RANGE |
            self.MODE_TX
        )


        start = time.ticks_ms()


        while True:

            flags = self.read_reg(
                self.REG_IRQ_FLAGS
            )


            if flags & self.IRQ_TX_DONE:

                self.write_reg(
                    self.REG_IRQ_FLAGS,
                    self.IRQ_TX_DONE
                )

                self.standby()

                return True


            if time.ticks_diff(
                time.ticks_ms(),
                start
            ) > 1000:

                self.standby()

                self.write_reg(
                    self.REG_IRQ_FLAGS,
                    0xFF
                )

                raise OSError(
                    "LoRa TX timeout"
                )


            time.sleep_ms(1)


# ============================================================
# CRC16
# ============================================================

def crc16(data):

    crc = 0xFFFF

    for byte in data:

        crc ^= (
            byte << 8
        )

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


# ============================================================
# HARDWARE
# ============================================================

spi = SoftSPI(
    baudrate=2_000_000,
    polarity=0,
    phase=0,

    sck=Pin(SCK_PIN),
    mosi=Pin(MOSI_PIN),
    miso=Pin(MISO_PIN)
)


cs = Pin(
    CS_PIN,
    Pin.OUT,
    value=1
)


reset = Pin(
    RESET_PIN,
    Pin.OUT,
    value=1
)


dio0 = Pin(
    DIO0_PIN,
    Pin.IN
)


# VERIFIED:
#
# RELEASED = LOW
# PRESSED  = HIGH

# Verified electrical logic:
#   RELEASED = 0
#   PRESSED  = 1
#
# This assumes the switch connects GPIO22 to 3.3 V when pressed,
# so the input must default LOW.
estop = Pin(
    ESTOP_PIN,
    Pin.IN,
    Pin.PULL_DOWN
)


radio = SX1278(
    spi,
    cs,
    reset,
    dio0,
    FREQUENCY
)


# ============================================================
# SESSION
# ============================================================

uid = unique_id()

mix = 0

for b in uid:

    mix = (
        ((mix << 1) ^ b)
        & 0xFFFF
    )


session = (
    mix ^
    (time.ticks_us() & 0xFFFF)
) & 0xFFFF


sequence = 0


# ============================================================
# PACKET
#
# 0  A5
# 1  5A
# 2  version
# 3  transmitter ID
# 4  command
# 5  session MSB
# 6  session LSB
# 7  sequence MSB
# 8  sequence LSB
# 9  CRC MSB
# 10 CRC LSB
# ============================================================

def build_packet(command):

    global sequence


    body = bytes((

        MAGIC0,
        MAGIC1,

        VERSION,

        TRANSMITTER_ID,

        command,

        (session >> 8) & 0xFF,
        session & 0xFF,

        (sequence >> 8) & 0xFF,
        sequence & 0xFF

    ))


    checksum = crc16(
        body
    )


    packet = (
        body +
        bytes((
            (checksum >> 8) & 0xFF,
            checksum & 0xFF
        ))
    )


    current_sequence = sequence


    sequence = (
        sequence + 1
    ) & 0xFFFF


    return packet, current_sequence


# ============================================================
# SEND COMMAND
# ============================================================

def send_command(command):

    packet, seq = build_packet(
        command
    )

    radio.send(
        packet
    )

    return seq


# ============================================================
# RADIO RECOVERY
# ============================================================

def safe_send(command):

    try:

        seq = send_command(
            command
        )

        return True, seq


    except Exception as e:

        print(
            "TX FAILURE:",
            e
        )

        print(
            "RESETTING RA-02"
        )


        try:

            radio.begin()

            seq = send_command(
                command
            )

            print(
                "RA-02 RECOVERED"
            )

            return True, seq


        except Exception as e2:

            print(
                "RA-02 RECOVERY FAILED:",
                e2
            )

            return False, None


# ============================================================
# START
# ============================================================

print()
print("================================")
print(" AUTOKART WIRELESS E-STOP TX   ")
print(" PICO + RA-02                  ")
print("================================")
print()


radio.begin()


print(
    "SESSION:",
    session
)


# ============================================================
# MAIN LOOP
# ============================================================

last_tx = time.ticks_ms()

previous_pressed = None

released_since = None


while True:

    now = time.ticks_ms()


    # ========================================================
    # VERIFIED SWITCH LOGIC:
    #
    # RELEASED = 0
    # PRESSED  = 1
    # ========================================================

    pressed = (
        estop.value() == 1
    )


    # ========================================================
    # BUTTON PRESSED
    # ========================================================

    if pressed:

        released_since = None


        # JUST PRESSED
        if previous_pressed is not True:

            print()
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("       E-STOP PRESSED")
            print("       STOP BURST TX")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print()


            for _ in range(
                STOP_BURST_COUNT
            ):

                ok, seq = safe_send(
                    CMD_STOP
                )


                if ok:

                    print(
                        "TX STOP seq",
                        seq
                    )


                time.sleep_ms(
                    STOP_BURST_GAP_MS
                )


            last_tx = (
                time.ticks_ms()
            )


        # Continue STOP heartbeat
        if time.ticks_diff(
            now,
            last_tx
        ) >= HEARTBEAT_MS:


            ok, seq = safe_send(
                CMD_STOP
            )


            if ok:

                print(
                    "TX STOP seq",
                    seq
                )


            last_tx = now


    # ========================================================
    # BUTTON RELEASED
    # ========================================================

    else:

        if released_since is None:

            released_since = now


        stable_time = (
            time.ticks_diff(
                now,
                released_since
            )
        )


        # Wait for stable released condition
        if stable_time >= RELEASE_STABLE_MS:


            if time.ticks_diff(
                now,
                last_tx
            ) >= HEARTBEAT_MS:


                ok, seq = safe_send(
                    CMD_RUN
                )


                if ok:

                    print(
                        "TX RUN  seq",
                        seq
                    )


                last_tx = now


    previous_pressed = (
        pressed
    )


    time.sleep_ms(2)