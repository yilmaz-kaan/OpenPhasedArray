# FT232H TX Board control script
# To setup, follow instructions at: https://learn.adafruit.com/circuitpython-on-any-computer-with-ft232h/setup
# format for instruction is "<Mode><Device Number> <Value>" (ex. A1 0 to set Attenuator 1 to 0)

import time
import board
import digitalio

# --- Configuration for SN74HCT138 Connections ---
# command threshold to ensure proper timing between commands
cmd_threshold = 45e-9  # 45 nanoseconds
# Select pins (A, B, C) and G1 mapped to FT232H C-pins
# C is MSB | B is middle | A is LSB
PIN_C = board.C0 
PIN_B = board.C1
PIN_A = board.C2
PIN_G1 = board.C3 # Active-High Enable (Pins G2A/G2B are hardwired to GND)
PIN_SPI_CLK = board.SCK # D0
PIN_SPI_MOSI = board.MOSI # D1
PIN_SPI_MISO = board.MISO # D2
# Store the pin objects as a global list for easy management
decoder_pin_objects = []

def setup_decoder():
    """
    Initializes the GPIO pins as digital outputs and enables the decoder.
    Returns a list of DigitalInOut objects: [C_pin, B_pin, A_pin, G1_pin].
    """
    
    #  Create DigitalInOut objects
    pin_c = digitalio.DigitalInOut(PIN_C)
    pin_b = digitalio.DigitalInOut(PIN_B)
    pin_a = digitalio.DigitalInOut(PIN_A)
    pin_g1 = digitalio.DigitalInOut(PIN_G1)
    
    #  Set all pins to output direction
    pin_c.direction = digitalio.Direction.OUTPUT
    pin_b.direction = digitalio.Direction.OUTPUT
    pin_a.direction = digitalio.Direction.OUTPUT
    pin_g1.direction = digitalio.Direction.OUTPUT
    
    #  Enable the HCT138 by setting G1 to HIGH (True)
    pin_g1.value = True 
    
    print("Decoder setup complete.")
    print("SN74HCT138 Enabled (G1=HIGH, G2A/G2B=GND).")
    
    # Store and return the pin objects: [C, B, A, G1]
    global decoder_pin_objects
    decoder_pin_objects = [pin_c, pin_b, pin_a, pin_g1]
    return decoder_pin_objects

def select_output(pin_objects, output_index):
    """
    Sets the C, B, A select pins to choose a specific output (Y0-Y7).
    
    :param pin_objects: List of pin objects [C_pin, B_pin, A_pin, G1_pin].
    :param output_index: The desired output index (0 to 7).
    """
    if not 0 <= output_index <= 7:
        raise ValueError("Output index must be between 0 and 7.")
    
    pin_c, pin_b, pin_a, pin_g1 = pin_objects
    
    # Ensure G1 is HIGH to keep the decoder enabled
    pin_g1.value = True
    

    binary_select = format(output_index, '03b')
    

    pin_c.value = binary_select[0] == '1'
    pin_b.value = binary_select[1] == '1'
    pin_a.value = binary_select[2] == '1'
    
    print(f"Selected output Y{output_index} (CBA={binary_select}). The pin is LOW.")

def disable_decoder(pin_objects):
    """
    Disables the SN74HCT138 by setting G1 to LOW, forcing all outputs Y0-Y7 to High.
    """
    _, _, _, pin_g1 = pin_objects
    
    # Set G1 LOW (False) to disable all outputs
    pin_g1.value = False
    print("Decoder disabled (G1=LOW). All outputs are High.")

import busio

# SPI Configuration
# Initialize the SPI bus on the FT232H
spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)

def reverse_bits(byte_val):
    """
    Reverses the bits of an 8-bit integer.
    Required because RFSA3713 expects LSB First, but busio.SPI sends MSB First.
    """
    byte_val = ((byte_val & 0xF0) >> 4) | ((byte_val & 0x0F) << 4)
    byte_val = ((byte_val & 0xCC) >> 2) | ((byte_val & 0x33) << 2)
    byte_val = ((byte_val & 0xAA) >> 1) | ((byte_val & 0x55) << 1)
    return byte_val

def send_spi_command(device_type, device_id, value, decoder_pins):
    """
    Handles the specific protocols for Phase Shifters and Attenuators.
    
    :param device_type: 'P' for Phase Shifter, 'A' for Attenuator
    :param device_id: Integer 1-4
    :param value: Integer value to write (0-255)
    :param decoder_pins: The list of decoder pin objects
    """
    elapsed_time = time.monotonic()
    # --- 1. Determine Decoder Output & Protocol ---
    target_y = 0
    data_bytes = bytearray()
    print(device_type)
    if device_type == 'P':
        target_y = 8 - device_id 
        # send 1 byte, the relevant 6 bits are shifted in last.
        data_bytes.append(value)

    elif device_type == 'A': 
        target_y = 4 - device_id
        
        
        # Since hardware SPI is MSB First, reverse the bits of each byte.
        reversed_val = reverse_bits(value)
        reversed_addr = reverse_bits(0x00)
        
        data_bytes.append(reversed_val)
        data_bytes.append(reversed_addr)
    
    else:
        print("Unknown Device Type")
        return

    print(f"Communicating with {device_type}{device_id} on Mux Y{target_y}...")
    
    # Select the Device (Assert CS)
    select_output(decoder_pins, target_y)
    
    # Write Data
    while not spi.try_lock():
        pass
    try:
        spi.write(data_bytes)
    finally:
        spi.unlock()
    
    # Setting G1=Low disables the decoder, forcing all Y outputs HIGH.
    # This creates the Rising Edge required to latch data.
    disable_decoder(decoder_pins)
    print("Command Sent & Latched.\n")
    elapsed_time = time.monotonic() - elapsed_time
    if elapsed_time < cmd_threshold:
        time.sleep(cmd_threshold - elapsed_time)
    print(f"Elapsed Time: {elapsed_time:.3f} seconds")
    



# Main CLI Loop
if __name__ == '__main__':
    try:
        decoder_pins = setup_decoder()
        
        while True:
            print('-' * 40)
            print("Mode Selection:")
            mode_input = input("Enter 'P' for Phase Shifter, 'A' for Attenuator, or 'exit': ").upper()
            
            if mode_input == 'EXIT':
                break
            mode_sel = mode_input[0]
            if mode_sel not in ['P', 'A']:
                print("Invalid mode. Please try again.")
                continue
                
            dev_sel = mode_input[1]
            if not dev_sel.isdigit() or not (1 <= int(dev_sel) <= 4):
                print("Invalid Device Number. Must be 1-4.")
                continue
                
            val_sel = mode_input[3:]
            if not val_sel.isdigit():
                print("Invalid Value.")
                continue
                
            send_spi_command(mode_sel, int(dev_sel), int(val_sel), decoder_pins)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if 'decoder_pin_objects' in globals() and decoder_pin_objects:
            disable_decoder(decoder_pin_objects)
            for pin in decoder_pin_objects:
                pin.deinit()
        if 'spi' in globals():
            spi.deinit()
        print("\nHardware de-initialized.")