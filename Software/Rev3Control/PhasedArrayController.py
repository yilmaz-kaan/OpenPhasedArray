import os

os.environ['BLINKA_FT232H'] = '1'

import time
import board
import digitalio
import busio



# --- Pin Configuration (C bank = JP1, D bank = JP2) ---
PIN_A   = board.C0  # GPIOA   – Demux address bit A (LSB)
PIN_B   = board.C1  # GPIOB   – Demux address bit B
PIN_C   = board.C2  # GPIOC   – Demux address bit C (MSB)
PIN_TR  = board.C3  # TR_CTRL – TX (HIGH) / RX (LOW)
PIN_VEN = board.C4  # V_EN    – Enable +/- rail LDOs
PIN_VPF = board.C5  # VPD_EN  – Enable forward power amplifiers
PIN_DID = board.C6  # DEV_ID  – Identification LED
PIN_G1  = board.C7  # GPIO-G1 – Demux enable

# SPI Pins (D bank = JP2)
PIN_CLK  = board.SCK   # D0 – CLK
PIN_MOSI = board.MOSI  # D1 – SER_IN
PIN_MISO = board.MISO  # D2 – SER_OUT

# --- Demux lookup table (from datasheet, Section: Demultiplexer) ---
# SN74HCT138 index = A(LSB) + 2*B + 4*C(MSB)
# Channel numbers 0-3 correspond to board elements AT0-AT3 / PS0-PS3
ATTN_MUX = {0: 7, 1: 0, 2: 4, 3: 2}  # AT0→Y7, AT1→Y0, AT2→Y4, AT3→Y2
PS_MUX   = {0: 6, 1: 1, 2: 5, 3: 3}  # PS0→Y6, PS1→Y1, PS2→Y5, PS3→Y3


class SystemController:
    def __init__(self):
        # SPI bus (held low until V_EN is asserted)
        self.spi = busio.SPI(clock=PIN_CLK, MOSI=PIN_MOSI, MISO=PIN_MISO)
        
        # Per datasheet power-up sequence: assert ALL lines low before DC is connected
        self.a  = digitalio.DigitalInOut(PIN_A)
        self.b  = digitalio.DigitalInOut(PIN_B)
        self.c  = digitalio.DigitalInOut(PIN_C)
        self.g1 = digitalio.DigitalInOut(PIN_G1)

        for p in [self.a, self.b, self.c, self.g1]:
            p.direction = digitalio.Direction.OUTPUT
            p.value = False

        self.tr_ctrl = digitalio.DigitalInOut(PIN_TR)
        self.v_en    = digitalio.DigitalInOut(PIN_VEN)
        self.vpf     = digitalio.DigitalInOut(PIN_VPF)
        self.dev_id  = digitalio.DigitalInOut(PIN_DID)

        for p in [self.tr_ctrl, self.v_en, self.vpf, self.dev_id]:
            p.direction = digitalio.Direction.OUTPUT
            p.value = False


        self.dev_id.value = True
        print("Controller initialized. All lines low.")
        print("Connect DC barrel jack, then use VON to bring up IC rails.")

    # ------------------------------------------------------------------ #
    #  Power management  (follow datasheet power-up / power-down sequence)
    # ------------------------------------------------------------------ #

    def power_on(self):
        """
        Assert V_EN to enable +/- rail LDOs.
        DC barrel jack must already be connected before calling this.
        After this returns it is safe to use SPI, VPD_EN, and TR_CTRL.
        Default state after power-on is RX.
        """
        self.v_en.value = True
        print("V_EN asserted — +/- rails up. Safe to communicate with RFICs (default: RX).")

    def power_off(self):
        """
        Safe power-down per datasheet sequence:
        1. Stop SPI / pull demux to sleep
        2. Pull VPD_EN low
        3. Pull V_EN low
        """
        self._sleep_mux()
        self.vpf.value = False
        self.v_en.value = False
        print("VPD_EN then V_EN de-asserted — ICs off. Safe to disconnect DC.")

    def amps_on(self):
        """Assert VPD_EN to enable forward power amplifiers. V_EN must be on first."""
        self.vpf.value = True
        print("VPD_EN asserted — power amplifiers enabled.")

    def amps_off(self):
        """De-assert VPD_EN to disable forward power amplifiers."""
        self.vpf.value = False
        print("VPD_EN de-asserted — power amplifiers disabled.")

    # ------------------------------------------------------------------ #
    #  Demux control
    # ------------------------------------------------------------------ #

    def _select_mux(self, index: int):
        """
        Drive A/B/C address lines and assert G1 to enable one demux output (active LOW).
        The selected Y output goes LOW, which holds CS low on the target RFIC
        for the duration of the SPI transaction.
        """
        self.a.value  = bool(index & 0b001)
        self.b.value  = bool(index & 0b010)
        self.c.value  = bool(index & 0b100)
        self.g1.value = True

    def _sleep_mux(self):
        """
        De-assert G1 — all demux Y outputs return HIGH.
        The rising edge on the selected line latches data into the RFIC.
        Leave the demux in this state when idle.
        """
        self.g1.value = False

    # ------------------------------------------------------------------ #
    #  SPI helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _reverse_bits(byte_val: int) -> int:
        """Reverse bit order for LSB-first devices (RFSA3713 attenuator)."""
        return int('{:08b}'.format(byte_val)[::-1], 2)

    def _spi_write(self, data: bytearray):
        while not self.spi.try_lock():
            pass
        try:
            self.spi.write(data)
        finally:
            self.spi.unlock()

    # ------------------------------------------------------------------ #
    #  RFIC commands
    # ------------------------------------------------------------------ #

    def send_phase(self, channel: int, val: int):
        """
        Write to phase shifter on channel 0-3.
        MAPS-010164 is MSB-first — no bit reversal needed.
        val: 0-63  (6-bit, 64 states, 5.625 deg/LSB)
        The demux CS line is held LOW during the transaction; the rising
        edge from _sleep_mux() latches the data into the device.
        """
        mux_idx = PS_MUX[channel]
        data = bytearray([val & 0x3F])
        self._select_mux(mux_idx)
        self._spi_write(data)
        self._sleep_mux()
        print(f"  Phase ch{channel} (PS{channel}, mux Y{mux_idx}): {val}  "
              f"({val * 5.625:.3f} deg)")

    def send_attn(self, channel: int, val: int):
        """
        Write to attenuator on channel 0-3.
        RFSA3713 is LSB-first — bits are reversed before sending.
        Two-byte transfer: data byte (reversed) then address byte (0x00, reversed).
        val: 0-127  (7-bit, 128 states, 0.254 dB/LSB)
        The demux CS line is held LOW during the transaction; the rising
        edge from _sleep_mux() latches the data into the device.
        """
        mux_idx = ATTN_MUX[channel]
        data = bytearray([
            self._reverse_bits(val & 0x7F),
            self._reverse_bits(0x00),
        ])
        self._select_mux(mux_idx)
        self._spi_write(data)
        self._sleep_mux()
        print(f"  Attn  ch{channel} (AT{channel}, mux Y{mux_idx}): {val}  "
              f"({val * 0.254:.3f} dB)")


# ------------------------------------------------------------------ #
#  CLI helpers
# ------------------------------------------------------------------ #

def parse_four_values(tokens):
    """Parse exactly 4 integer values from a token list, with a clear error."""
    if len(tokens) != 4:
        raise ValueError(f"Expected 4 values (one per channel), got {len(tokens)}.")
    return [int(t) for t in tokens]


# ------------------------------------------------------------------ #
#  Main
# ------------------------------------------------------------------ #

if __name__ == '__main__':
    ctrl = SystemController()

    print("\n--- Command Interface ---")
    print("  P <v0> <v1> <v2> <v3>  Set phase shifters ch0-3  (values 0-63,  5.625 deg/LSB)")
    print("  A <v0> <v1> <v2> <v3>  Set attenuators    ch0-3  (values 0-127, 0.254 dB/LSB)")
    print("  TX / RX                Switch transmit (HIGH) / receive (LOW)")
    print("  VON  / VOFF            IC rail power via V_EN    (VON after connecting DC)")
    print("  PON  / POFF            Amp power via VPD_EN")
    print("  EXIT                   Safe shutdown")
    print()
    print("  e.g.  P 0 16 32 48   → PS0=0°, PS1=90°, PS2=180°, PS3=270°")
    print("  e.g.  A 0 10 20 30   → AT0=0dB, AT1=2.54dB, AT2=5.08dB, AT3=7.62dB")

    try:
        while True:
            raw = input("\nEnter Command: ").strip()
            if not raw:
                continue
            tokens = raw.upper().split()
            base = tokens[0]

            if base == 'EXIT':
                break

            elif base == 'TX':
                ctrl.tr_ctrl.value = True
                print("Mode: TX")

            elif base == 'RX':
                ctrl.tr_ctrl.value = False
                print("Mode: RX")

            elif base == 'VON':
                ctrl.power_on()

            elif base == 'VOFF':
                ctrl.power_off()

            elif base == 'PON':
                ctrl.amps_on()

            elif base == 'POFF':
                ctrl.amps_off()

            elif base == 'P':
                try:
                    vals = parse_four_values(tokens[1:])
                    print("Setting phase shifters:")
                    for ch, v in enumerate(vals):
                        ctrl.send_phase(ch, v)
                except (ValueError, KeyError) as e:
                    print(f"  Error: {e}")
                    print("  Usage: P <v0> <v1> <v2> <v3>  (0-63 each)")

            elif base == 'A':
                try:
                    vals = parse_four_values(tokens[1:])
                    print("Setting attenuators:")
                    for ch, v in enumerate(vals):
                        ctrl.send_attn(ch, v)
                except (ValueError, KeyError) as e:
                    print(f"  Error: {e}")
                    print("  Usage: A <v0> <v1> <v2> <v3>  (0-127 each)")

            else:
                print(f"  Unknown command '{base}'. Type EXIT to quit.")

    finally:
        # Safe power-down sequence per datasheet
        ctrl._sleep_mux()
        ctrl.vpf.value = False
        ctrl.v_en.value = False
        ctrl.dev_id.value = False
        print("Hardware safely de-initialized.")