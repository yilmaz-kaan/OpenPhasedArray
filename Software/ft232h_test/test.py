# FT232H TX Board control script
# To setup, follow instructions at: https://learn.adafruit.com/circuitpython-on-any-computer-with-ft232h/setup
# format for instruction is "<Mode><Device Number> <Value>" (ex. A1 0 to set Attenuator 1 to 0)


import os

os.environ['BLINKA_FT232H'] = '1'

import busio
import time
import board
import digitalio
import math
import numpy as np
import pandas as pd

# --- Configuration for SN74HCT138 Connections ---
# command threshold to ensure proper timing between commands
cmd_threshold = 45e-9  # 45 nanoseconds
ELEMENT_COUNT = 4
D_OVER_LAMBDA = 0.5  # Standard half-wavelength spacing
CMD_THRESHOLD = 45e-9
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
    


def calculate_phases(angle_deg):
    """
    Calculates the required phase shift for each element.
    :param angle_deg: Steering angle in degrees (-90 to 90)
    :return: List of 4 phase values in degrees (0-360)
    """
    angle_rad = math.radians(angle_deg)
    # Calculate phase difference between elements
    delta_phi_rad = 2 * math.pi * D_OVER_LAMBDA * math.sin(angle_rad)
    delta_phi_deg = math.degrees(delta_phi_rad)
    
    phases = []
    for i in range(ELEMENT_COUNT):
        # Calculate progressive phase and wrap to [0, 360)
        p = (i * delta_phi_deg) % 360
        if p < 0: p += 360
        phases.append(p)
    return phases

def deg_to_bits(degrees):
    """Maps 0-360 degrees to 0-255 bit value."""
    return int((degrees / 360.0) * 255)
def get_calibration_offsets(freq_hz):
    """Reads S-parameters and calculates offsets relative to Channel 1."""
    files = ['out1.csv', 'out2.csv', 'out3.csv', 'out4.csv']
    offsets = []
    raw_s21 = []
    
    print(f"Loading calibration for {freq_hz/1e9:.3f} GHz...")
    for f in files:
        df = pd.read_csv(f, comment='#')
        # Find row closest to target frequency
        idx = (df['freq[Hz]'] - freq_hz).abs().idxmin()
        row = df.loc[idx]
        # S21 is Trc3 in your files
        val = complex(row['re:Trc3_S21'], row['im:Trc3_S21'])
        raw_s21.append(val)
    
    # Reference everything to Channel 1
    ref_phase = np.degrees(np.angle(raw_s21[0]))
    ref_mag = np.abs(raw_s21[0])
    
    for i in range(4):
        p_off = (ref_phase - np.degrees(np.angle(raw_s21[i]))) % 360
        m_ratio = ref_mag / np.abs(raw_s21[i])
        att_off_db = 20 * np.log10(m_ratio)
        offsets.append({'p_off': p_off, 'att_off_db': att_off_db})
        print(f"  Ch{i+1} Offset: Phase={p_off:.1f}°, Amp={att_off_db:.2f}dB")
    
    return offsets

# --- UPDATED: Calibrated Beam Update ---
def update_beam_calibrated(angle_deg, offsets, decoder_pins):
    """Calculates phase + calibration and updates all 4 elements."""
    # Theoretical steering math
    d_lambda = 0.5 
    delta_phi_steer = math.degrees(2 * math.pi * d_lambda * math.sin(math.radians(angle_deg)))
    
    print(f"\nSteering to {angle_deg}°...")
    for i in range(4):
        dev_id = i + 1
        # 1. Calculate Target Phase (Theory + Calibration)
        theory_p = (i * delta_phi_steer) % 360
        final_p = (theory_p + offsets[i]['p_off']) % 360
        p_bits = int((final_p / 360.0) * 255)
        
        # 2. Calculate Target Attenuation (Equalization)
        # We start at a baseline (e.g. 10dB) so we have room to go up/down
        base_att = 10.0 
        final_att_db = base_att + offsets[i]['att_off_db']
        # RFSA3713 0.25dB step mapping
        a_bits = max(0, min(255, int(final_att_db / 0.25)))
        
        # 3. Send Commands
        send_spi_command('P', dev_id, p_bits, decoder_pins)
        send_spi_command('A', dev_id, a_bits, decoder_pins)
        print(f"  CH{dev_id} -> Phase: {final_p:5.1f}° ({p_bits:3d} bits), Att: {final_att_db:4.1f}dB")
# --- Main CLI Loop Adapted for Steering ---
if __name__ == '__main__':
    try:
        decoder_pins = setup_decoder()
        spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)

        print("Beamforming Controller Active.")
        print("Enter steering angle between -90 and 90 degrees.")
        f_ghz = float(input("Enter operating frequency (GHz): "))
        offsets = get_calibration_offsets(f_ghz * 1e9)
        while True:
            user_input = input("\nEnter Angle (or 'exit'): ").strip().lower()
            
            if user_input == 'exit':
                break
            
            try:
                angle = float(user_input)
                if not -90 <= angle <= 90:
                    print("Error: Angle must be between -90 and 90.")
                    continue
                
                update_beam_calibrated(angle, offsets, decoder_pins)
                
            except ValueError:
                print("Invalid input. Please enter a number.")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'decoder_pin_objects' in globals() and decoder_pin_objects:
            disable_decoder(decoder_pin_objects)
            for pin in decoder_pin_objects:
                pin.deinit()
        if 'spi' in globals():
            spi.deinit()
        print("\nHardware de-initialized.")