import os
import time

class HardwareControl:
    def __init__(self, serial_number=None):
        self.decoder_pins = []
        self.spi = None
        self.board = None
        self.digitalio = None
        self.busio = None
        
        try:
            os.environ['BLINKA_FT232H'] = '1'
            if serial_number:
                os.environ['BLINKA_FT232H_SERIAL'] = serial_number
            import board
            import digitalio
            import busio
            self.board = board
            self.digitalio = digitalio
            self.busio = busio
            self._setup_decoder()
            self.spi = self.busio.SPI(clock=self.board.SCK, MOSI=self.board.MOSI, MISO=self.board.MISO)
        except (ImportError, RuntimeError, ValueError) as e:
            print(f"Hardware initialization failed: {e}")
            self.spi = None
            self.decoder_pins = []


    def _setup_decoder(self):
        """Initializes the GPIO pins and enables the decoder."""
        pin_c = self.digitalio.DigitalInOut(self.board.C0)
        pin_b = self.digitalio.DigitalInOut(self.board.C1)
        pin_a = self.digitalio.DigitalInOut(self.board.C2)
        pin_g1 = self.digitalio.DigitalInOut(self.board.C3)
        
        pin_c.direction = self.digitalio.Direction.OUTPUT
        pin_b.direction = self.digitalio.Direction.OUTPUT
        pin_a.direction = self.digitalio.Direction.OUTPUT
        pin_g1.direction = self.digitalio.Direction.OUTPUT
        
        pin_g1.value = True
        self.decoder_pins = [pin_c, pin_b, pin_a, pin_g1]

    def _select_output(self, output_index):
        """Sets the C, B, A select pins to choose a specific output (Y0-Y7)."""
        if not (self.decoder_pins and 0 <= output_index <= 7):
            return
        pin_c, pin_b, pin_a, pin_g1 = self.decoder_pins
        pin_g1.value = True
        binary_select = format(output_index, '03b')
        pin_c.value = binary_select[0] == '1'
        pin_b.value = binary_select[1] == '1'
        pin_a.value = binary_select[2] == '1'

    def _disable_decoder(self):
        """Disables the SN74HCT138 by setting G1 to LOW."""
        if self.decoder_pins:
            _, _, _, pin_g1 = self.decoder_pins
            pin_g1.value = False

    def _reverse_bits(self, byte_val):
        """Reverses the bits of an 8-bit integer."""
        byte_val = ((byte_val & 0xF0) >> 4) | ((byte_val & 0x0F) << 4)
        byte_val = ((byte_val & 0xCC) >> 2) | ((byte_val & 0x33) << 2)
        byte_val = ((byte_val & 0xAA) >> 1) | ((byte_val & 0x55) << 1)
        return byte_val

    def send_spi_command(self, device_type, device_id, value):
        """Handles the specific protocols for Phase Shifters and Attenuators."""
        if not self.spi:
            return

        target_y = 0
        data_bytes = bytearray()
        if device_type == 'P':
            target_y = 8 - device_id
            data_bytes.append(value)
        elif device_type == 'A':
            target_y = 4 - device_id
            reversed_val = self._reverse_bits(value)
            reversed_addr = self._reverse_bits(0x00)
            data_bytes.append(reversed_val)
            data_bytes.append(reversed_addr)
        else:
            print("Unknown Device Type")
            return

        self._select_output(target_y)
        
        while not self.spi.try_lock():
            pass
        try:
            self.spi.write(data_bytes)
        finally:
            self.spi.unlock()
        
        self._disable_decoder()

    def set_phase(self, device_id, value):
        """Sets the phase for a specific phase shifter."""
        self.send_spi_command('P', device_id, value)

    def set_gain(self, device_id, value):
        """Sets the gain for a specific attenuator."""
        self.send_spi_command('A', device_id, value)

    def deinitialize_hardware(self):
        """De-initializes the hardware."""
        if self.decoder_pins:
            self._disable_decoder()
            for pin in self.decoder_pins:
                pin.deinit()
        if self.spi:
            self.spi.deinit()
        print("Hardware de-initialized.")