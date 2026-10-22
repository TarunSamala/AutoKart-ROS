from machine import Pin, SoftSPI
import time


# ============================================================
# ESP32-S3 + RA-02
# SIMPLE LORA STRING RECEIVER
#
# Receives ANY LoRa payload and prints it.
#
# No heartbeat
# No AutoKart protocol
# No application CRC
# No sequence counter
# No STOP/RUN logic
# ============================================================


# ============================================================
# PINS
# ============================================================

SCK_PIN = 14
MOSI_PIN = 13
MISO_PIN = 15

CS_PIN = 16
RESET_PIN = 17
DIO0_PIN = 18


# ============================================================
# LORA SETTINGS
# ============================================================

FREQUENCY = 433_000_000


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


    # --------------------------------------------------------

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

        data = self.spi.read(
            1,
            0x00
        )

        self.cs.value(1)

        return data[0]


    # --------------------------------------------------------

    def read_fifo(self, length):

        self.cs.value(0)

        self.spi.write(
            bytes((
                self.REG_FIFO & 0x7F,
            ))
        )

        data = self.spi.read(
            length,
            0x00
        )

        self.cs.value(1)

        return data


    # --------------------------------------------------------

    def reset_radio(self):

        self.reset.value(0)
        time.sleep_ms(10)

        self.reset.value(1)
        time.sleep_ms(20)


    # --------------------------------------------------------

    def set_frequency(self, frequency):

        frf = int(
            (frequency * 524288)
            / 32_000_000
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
                "RA-02 NOT FOUND. "
                "Expected 0x12, got 0x%02X"
                % version
            )


        # LoRa sleep
        self.write_reg(
            self.REG_OP_MODE,
            self.MODE_LONG_RANGE |
            self.MODE_SLEEP
        )

        time.sleep_ms(10)


        # Set frequency
        self.set_frequency(
            self.frequency
        )


        # FIFO base addresses
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


        # ====================================================
        # LoRa parameters
        #
        # BW = 125 kHz
        # CR = 4/5
        # Explicit header
        # ====================================================

        self.write_reg(
            self.REG_MODEM_CONFIG_1,
            0x72
        )


        # SF7
        # Hardware packet CRC ON
        self.write_reg(
            self.REG_MODEM_CONFIG_2,
            0x74
        )


        # AGC ON
        self.write_reg(
            self.REG_MODEM_CONFIG_3,
            0x04
        )


        # Preamble = 8
        self.write_reg(
            self.REG_PREAMBLE_MSB,
            0x00
        )

        self.write_reg(
            self.REG_PREAMBLE_LSB,
            0x08
        )


        # Sync word
        self.write_reg(
            self.REG_SYNC_WORD,
            0x12
        )


        # Clear interrupts
        self.write_reg(
            self.REG_IRQ_FLAGS,
            0xFF
        )


        self.standby()

        print(
            "RA-02 INITIALIZED"
        )


    # --------------------------------------------------------

    def start_receive(self):

        # DIO0 -> RxDone
        self.write_reg(
            self.REG_DIO_MAPPING_1,
            0x00
        )


        # Clear IRQ
        self.write_reg(
            self.REG_IRQ_FLAGS,
            0xFF
        )


        # Continuous receive
        self.write_reg(
            self.REG_OP_MODE,
            self.MODE_LONG_RANGE |
            self.MODE_RX_CONTINUOUS
        )


    # --------------------------------------------------------

    def receive(self):

        # Nothing received yet
        if self.dio0.value() == 0:
            return None


        flags = self.read_reg(
            self.REG_IRQ_FLAGS
        )


        # Clear IRQ flags
        self.write_reg(
            self.REG_IRQ_FLAGS,
            flags
        )


        # Check RX DONE
        if not (
            flags &
            self.IRQ_RX_DONE
        ):

            return None


        # Packet corrupted
        if (
            flags &
            self.IRQ_PAYLOAD_CRC_ERROR
        ):

            print(
                "CRC ERROR - PACKET IGNORED"
            )

            return None


        # Number of bytes
        length = self.read_reg(
            self.REG_RX_NB_BYTES
        )


        # FIFO location
        fifo_address = self.read_reg(
            self.REG_FIFO_RX_CURRENT_ADDR
        )


        self.write_reg(
            self.REG_FIFO_ADDR_PTR,
            fifo_address
        )


        # Read payload
        payload = self.read_fifo(
            length
        )


        return payload


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


# ============================================================
# CONTROL PINS
# ============================================================

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


# ============================================================
# RADIO
# ============================================================

radio = SX1278(

    spi,
    cs,
    reset,
    dio0,

    FREQUENCY
)


# ============================================================
# START
# ============================================================

print()
print("================================")
print(" ESP32-S3 SIMPLE LORA RECEIVER ")
print(" RA-02 SX1278 - 433 MHz        ")
print("================================")
print()


try:

    radio.begin()

except Exception as e:

    print()
    print("RA-02 INITIALIZATION FAILED")
    print(e)

    print()
    print("CHECK:")
    print("GPIO14 -> SCK")
    print("GPIO13 -> MOSI")
    print("GPIO15 -> MISO")
    print("GPIO16 -> NSS")
    print("GPIO17 -> RESET")
    print("GPIO18 -> DIO0")
    print("3.3V   -> RA02 3.3V")
    print("GND    -> RA02 GND")

    raise


radio.start_receive()


print()
print("WAITING FOR LORA MESSAGES...")
print()


# ============================================================
# MAIN LOOP
# ============================================================

packet_count = 0


while True:

    payload = radio.receive()


    if payload is not None:

        packet_count += 1


        print()
        print(
            "-----------------------------"
        )

        print(
            "PACKET:",
            packet_count
        )


        print(
            "BYTES:",
            len(payload)
        )


        # ----------------------------------------------
        # Try converting payload to normal text
        # ----------------------------------------------

        try:

            message = payload.decode(
                "utf-8"
            )

            print(
                "MESSAGE:",
                message
            )


        except:

            # If packet is not UTF-8 text,
            # show raw bytes instead.

            print(
                "RAW:",
                payload
            )


        print(
            "-----------------------------"
        )


    time.sleep_ms(2)