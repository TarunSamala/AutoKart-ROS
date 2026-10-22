from machine import Pin, SoftSPI
import time


# ============================================================
# AUTOKART WIRELESS E-STOP RECEIVER
# ESP32-S3 + RA-02 SX1278
#
# BENCH E-STOP TEST
# ============================================================


# ---------------------- PINS -------------------------------

SCK_PIN = 14
MOSI_PIN = 13
MISO_PIN = 15

CS_PIN = 16
RESET_PIN = 17
DIO0_PIN = 18


# -------------------- RADIO -------------------------------

FREQUENCY = 433_000_000


# Pico sends every 100 ms.
#
# 350 ms means ~3.5 missed heartbeats
# triggers failure.
LINK_TIMEOUT_MS = 350


# Receiver will not become healthy from one RUN packet.
GOOD_RUN_PACKETS_REQUIRED = 5


# ------------------- PROTOCOL ------------------------------

MAGIC0 = 0xA5
MAGIC1 = 0x5A

PROTOCOL_VERSION = 1

EXPECTED_TRANSMITTER_ID = 0x31

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

        value = self.spi.read(
            1,
            0x00
        )[0]

        self.cs.value(1)

        return value


    def read_fifo(self, length):

        self.cs.value(0)

        self.spi.write(
            bytes((self.REG_FIFO & 0x7F,))
        )

        data = self.spi.read(
            length,
            0x00
        )

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


        print(
            "SX1278 VERSION:",
            hex(version)
        )


        if version != 0x12:

            raise OSError(
                "RA-02 not detected. "
                "Expected 0x12, got 0x%02X"
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
            self.read_reg(
                self.REG_LNA
            ) | 0x03
        )


        # BW125 / CR4/5 / explicit
        self.write_reg(
            self.REG_MODEM_CONFIG_1,
            0x72
        )


        # SF7 / hardware CRC
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

        # DIO0 = RxDone
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

            print(
                "RADIO CRC ERROR"
            )

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


        return self.read_fifo(
            length
        )


# ============================================================
# CRC16
# ============================================================

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


# ============================================================
# SPI
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


radio = SX1278(
    spi,
    cs,
    reset,
    dio0,
    FREQUENCY
)


# ============================================================
# PARSE PACKET
# ============================================================

def parse_packet(packet):

    if packet is None:

        return None


    if len(packet) != 11:

        print(
            "INVALID LENGTH:",
            len(packet)
        )

        return None


    if (
        packet[0] != MAGIC0 or
        packet[1] != MAGIC1
    ):

        return None


    if (
        packet[2] !=
        PROTOCOL_VERSION
    ):

        print(
            "WRONG PROTOCOL"
        )

        return None


    if (
        packet[3] !=
        EXPECTED_TRANSMITTER_ID
    ):

        print(
            "WRONG TX ID"
        )

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


    if (
        received_crc !=
        calculated_crc
    ):

        print(
            "APPLICATION CRC ERROR"
        )

        return None


    return (
        command,
        session,
        sequence
    )


# ============================================================
# SEQUENCE CHECK
# ============================================================

def sequence_is_fresh(
    new_seq,
    old_seq
):

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
# SAFETY STATE
# ============================================================

current_session = None
last_sequence = None

last_valid_packet_ms = None

good_run_count = 0


# Receiver ALWAYS starts stopped.
stop_requested = True

# Actual STOP or timeout latches.
stop_latched = False


current_state = None


def print_state(reason):

    global current_state


    if stop_latched:

        state = "E_STOP_LATCHED"

    elif stop_requested:

        state = "STOPPED_WAITING_FOR_LINK"

    else:

        state = "RUN_LINK_HEALTHY"


    if state != current_state:

        print()
        print("================================")
        print(state)
        print("Reason:", reason)
        print("================================")
        print()

        current_state = state


def temporary_stop(reason):

    global stop_requested


    stop_requested = True


    print_state(
        reason
    )


def latch_stop(reason):

    global stop_requested
    global stop_latched


    stop_requested = True
    stop_latched = True


    print_state(
        reason
    )


def accept_run():

    global stop_requested


    if stop_latched:

        return


    stop_requested = False


    print_state(
        "5 VALID RUN HEARTBEATS"
    )


# ============================================================
# START
# ============================================================

print()
print("================================")
print(" AUTOKART WIRELESS E-STOP RX   ")
print(" ESP32-S3 + RA-02              ")
print("================================")
print()


print_state(
    "RECEIVER STARTUP"
)


radio.begin()

radio.start_receive()


print(
    "WAITING FOR PICO..."
)


# ============================================================
# LOOP
# ============================================================

while True:

    packet = radio.receive()


    if packet is not None:

        parsed = parse_packet(
            packet
        )


        if parsed is not None:

            command = parsed[0]
            session = parsed[1]
            sequence = parsed[2]


            # ================================================
            # NEW TRANSMITTER SESSION
            # ================================================

            if (
                current_session !=
                session
            ):

                print()
                print(
                    "NEW TX SESSION:",
                    session
                )


                current_session = (
                    session
                )


                last_sequence = None

                good_run_count = 0


                # New transmitter boot:
                # return to safe state until validated.
                temporary_stop(
                    "NEW TRANSMITTER SESSION"
                )


            # ================================================
            # FRESH PACKET ONLY
            # ================================================

            if sequence_is_fresh(
                sequence,
                last_sequence
            ):

                last_sequence = (
                    sequence
                )


                last_valid_packet_ms = (
                    time.ticks_ms()
                )


                # ============================================
                # STOP
                # ============================================

                if (
                    command ==
                    CMD_STOP
                ):

                    good_run_count = 0


                    print(
                        "RX STOP",
                        sequence
                    )


                    latch_stop(
                        "REMOTE MUSHROOM PRESSED"
                    )


                # ============================================
                # RUN HEARTBEAT
                # ============================================

                elif (
                    command ==
                    CMD_RUN
                ):

                    print(
                        "RX RUN ",
                        sequence
                    )


                    if not stop_latched:

                        good_run_count += 1


                        if (
                            good_run_count >=
                            GOOD_RUN_PACKETS_REQUIRED
                        ):

                            accept_run()


            else:

                print(
                    "STALE/DUPLICATE:",
                    sequence
                )


    # ========================================================
    # RADIO FAILURE / TRANSMITTER POWER LOSS
    # ========================================================

    if (
        last_valid_packet_ms is
        not None
    ):

        packet_age = (
            time.ticks_diff(
                time.ticks_ms(),
                last_valid_packet_ms
            )
        )


        if (
            packet_age >
            LINK_TIMEOUT_MS
        ):

            good_run_count = 0


            if not stop_latched:

                latch_stop(
                    "LORA HEARTBEAT TIMEOUT"
                )


    time.sleep_ms(2)