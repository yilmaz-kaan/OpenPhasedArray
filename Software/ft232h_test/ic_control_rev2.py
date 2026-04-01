import os
import time
import board
import digitalio
import busio

os.environ['BLINKA_FT232H'] = '1'

# --- Pin Configuration ---
PIN_A = board.C0 # Decoder pin A
PIN_B = board.C1 # Decoder pin B
PIN_C = board.C2 # Decoder pin C
PIN_TR = board.C3   # TX/RX control
PIN_VEN = board.C4  # IC enable
PIN_VPF = board.C5  # Amp enable
PIN_DID = board.C6  # Device ID LED
PIN_G1 = board.C7   # Decoder enable

# SPI Pins (Standard FT232H D-Bus)
PIN_CLK = board.SCK   # D0
PIN_MOSI = board.MOSI # D1
PIN_MISO = board.MISO # D2

class SystemController:
    def __init__(self):
        # Initialize Decoder Pins
        self.a = digitalio.DigitalInOut(PIN_A)
        self.b = digitalio.DigitalInOut(PIN_B)
        self.c = digitalio.DigitalInOut(PIN_C)
        self.g1 = digitalio.DigitalInOut(PIN_G1)
        
        for p in [self.a, self.b, self.c, self.g1]:
            p.direction = digitalio.Direction.OUTPUT

        # Initialize System control Pins
        self.tr_ctrl = digitalio.DigitalInOut(PIN_TR)
        self.v_en = digitalio.DigitalInOut(PIN_VEN)
        self.vpf = digitalio.DigitalInOut(PIN_VPF)
        self.dev_id = digitalio.DigitalInOut(PIN_DID)

        for p in [self.tr_ctrl, self.v_en, self.vpf, self.dev_id]:
            p.direction = digitalio.Direction.OUTPUT
            p.value = False # Default safety: All OFF

        # Setup SPI
        self.spi = busio.SPI(clock=PIN_CLK, MOSI=PIN_MOSI, MISO=PIN_MISO)
        
        # Turn on Blue LED (Device ID)
        self.dev_id.value = True
        print("System Initialized. Device ID LED: ON")

    def set_decoder_output(self, index):
        """Sets A, B, C to select Y0-Y7. G1 is set HIGH to enable."""
        if not 0 <= index <= 7: return
        
        # HCT138 Logic: A=LSB, B=Mid, C=MSB [cite: 31, 247]
        self.a.value = (index & 0b001) > 0
        self.b.value = (index & 0b010) > 0
        self.c.value = (index & 0b100) > 0
        self.g1.value = True 

    def latch_and_disable(self):
        """Sets G1 LOW. Forces all Y outputs HIGH (creates rising edge)."""
        self.g1.value = False

    def reverse_bits(self, byte_val):
        """Reverses bits for LSB-first devices."""
        return int('{:08b}'.format(byte_val)[::-1], 2)

    def send_command(self, dev_type, dev_num, val):
        target_y = (8 - dev_num) if dev_type == 'P' else (4 - dev_num)
        data = bytearray()
        
        if dev_type == 'P':
            data.append(val)
        else: # Attenuator protocol (Address + Data)
            data.append(self.reverse_bits(val))
            data.append(self.reverse_bits(0x00))

        self.set_decoder_output(target_y)
        
        while not self.spi.try_lock(): pass
        try:
            self.spi.write(data)
        finally:
            self.spi.unlock()
            
        self.latch_and_disable()
        print(f"Sent {val} to {dev_type}{dev_num} (Mux Y{target_y})")

# --- Main CLI ---
if __name__ == '__main__':
    ctrl = SystemController()
    print("\n--- Command Interface Guide ---")
    print("A[1-4] [val] : Set Attenuator")
    print("P[1-4] [val] : Set Phase Shifter")
    print("TX / RX      : Switch Transmit/Receive")
    print("VON / VOFF   : Global IC Power (V_EN)")
    print("PON / POFF   : Amp Power (VPF)")
    print("EXIT         : Shutdown")

    try:
        while True:
            cmd = input("\nEnter Command: ").upper().split()
            if not cmd: continue
            
            base = cmd[0]
            
            if base == 'EXIT': break
            elif base == 'TX': ctrl.tr_ctrl.value = True; print("Mode: TX")
            elif base == 'RX': ctrl.tr_ctrl.value = False; print("Mode: RX")
            elif base == 'VON': ctrl.v_en.value = True; print("ICs: Powered")
            elif base == 'VOFF': ctrl.v_en.value = False; print("ICs: Off")
            elif base == 'PON': ctrl.vpf.value = True; print("Amps: Enabled")
            elif base == 'POFF': ctrl.vpf.value = False; print("Amps: Disabled")
            
            elif (base[0] in ['A', 'P']) and len(base) > 1:
                dtype = base[0]
                dnum = int(base[1])
                val = int(cmd[1])
                ctrl.send_command(dtype, dnum, val)
    finally:
        # Emergency Shutdown
        ctrl.v_en.value = False
        ctrl.vpf.value = False
        ctrl.dev_id.value = False
        print("Hardware Safely De-initialized.")