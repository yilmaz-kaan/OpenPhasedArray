import pyvisa
import time

# --- Configuration ---
RESOURCE_STRING = 'USB0::2907::6::6261932852::0::INSTR' # Update with your IP or GPIB address
TEST_FREQ_HZ = 2.345e9      # 1 GHz
TEST_POWER_DBM = -20.0  # -20 dBm
SPAN_HZ = 100e3          # 100 kHz span around the center frequency
HOLD_TIME_SEC = 2.0    # Time to let Max Hold accumulate

def main():
    rm = pyvisa.ResourceManager()
    
    try:
        # Open connection to the instrument
        print(f"Connecting to {RESOURCE_STRING}...")
        inst = rm.open_resource(RESOURCE_STRING)
        inst.timeout = 10000 # 10 seconds timeout
        
        # Verify connection
        idn = inst.query('*IDN?')
        print(f"Connected to: {idn.strip()}")
        
        # Optional: Reset the instrument to a known state
        # inst.write('*RST')
        # time.sleep(1)

        
        """print("\n--- Configuring Signal Generator ---")
        # Route to Signal Generator subsystem and set parameters
        inst.write('OUTP ON')
        inst.write(f'SOURce:FREQuency {TEST_FREQ_HZ}')
        inst.write(f'SOURce:POWer {TEST_POWER_DBM}')
        inst.write('OUTPut:STATe ON')
        print(f"SG Output turned ON at {TEST_FREQ_HZ/1e9} GHz, {TEST_POWER_DBM} dBm.") """

        print("\n--- Configuring Spectrum Analyzer ---")
        # Ensure we are in Spectrum Analyzer mode
        inst.write('INST:SYST SPECT,ACT')
        print(f"Spectrum analyzer mode: {inst.query('INST:SYST? SPECT').strip()}")

        # Set Frequency and Span
        inst.write(f'FREQuency:CENTer {TEST_FREQ_HZ}')
        print(f"Center Frequency: {inst.query('FREQuency:CENTer?').strip()} Hz")
        inst.write(f'FREQuency:SPAN {SPAN_HZ}')
        print(f"Span: {inst.query('FREQuency:SPAN?').strip()} Hz")
        
        # Query Trace 1 type
        # trace_type = inst.query(':TRAC1:STOR:MODE?').strip()
        # print(f" Trace Type: {trace_type}")

        # Set Trace 1 to Max Hold 
        inst.write('TRACe1:STORage:MODE MAXHold')

        # Ensure continuous sweeping is on so it actively updates the Max Hold
        inst.write('INITiate:CONTinuous ON')

        inst.write('CALC:MARK:STAT ON')
        inst.write('CALC:MARK:MODE NORM')

        print(f"\nWaiting {HOLD_TIME_SEC} seconds for Max Hold trace to accumulate...")
        time.sleep(HOLD_TIME_SEC)

        print("\n--- Polling Measurements ---")
        # Turn on Marker 1
        
        # Execute a Peak Search on the current trace
        inst.write('CALC:MARK:MAX')
        
        # Query the X (Frequency) and Y (Amplitude) values of Marker 1
        marker_freq = float(inst.query('CALC:MARK:X?'))
        marker_amp = float(inst.query('CALC:MARK:Y?'))

        print(f"Peak Found:")
        print(f"Frequency: {marker_freq / 1e9:.6f} GHz")
        print(f"Amplitude: {marker_amp:.2f} dBm")
        return

    except pyvisa.VisaIOError as e:
        print(f"VISA Communication Error: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
    
    finally:
        # Cleanup
        try:
            inst.close()
            print("Connection closed.")
        except:
            pass

if __name__ == '__main__':
    main()