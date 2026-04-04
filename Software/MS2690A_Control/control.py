import pyvisa
import time

USB_RESOURCE = 'USB0::2907::6::6261932852::0::INSTR'

def main():
    rm = pyvisa.ResourceManager('@py')
    
    try:
        print(f"Connecting to {USB_RESOURCE}...")
        inst = rm.open_resource(USB_RESOURCE)
        
        inst.timeout = 10000 
        inst.read_termination = '\n'
        inst.write_termination = '\n'

        # 1. Clear status and verify IDN
        inst.write('*CLS')
        idn = inst.query('*IDN?')
        print(f"Connected to: {idn.strip()}")

        # 1. Force Spectrum Analyzer Mode (Critical)
        print("Selecting Spectrum Analyzer Application...")
        inst.write(':INST:SEL SAN')
        inst.query('*OPC?') # Wait for hardware relays to click over

        # 2. Set Frequency using full SCPI paths
        print("Setting Center Frequency to 2.4 GHz...")
        inst.write(':SENSE:FREQuency:CENTer 2.4GHZ')

        print("Setting Span to 10 MHz...")
        inst.write(':SENSE:FREQuency:SPAN 10MHZ')

        #	 3. Final Sync to ensure the display updates
        opc = inst.query('*OPC?')
        print(f"Display should now reflect 2.4 GHz center. OPC value is {opc.strip()}")
        # 5. Data Retrieval
        print("Querying Results...")
        # Note: Some Anritsu SCPI versions prefer :Y? or :AMPlitude?
        peak_amp = inst.query(':CALCulate:MARKer1:Y?')
        peak_freq = inst.query(':CALCulate:MARKer1:X?')

        print(f"\n--- Result at {time.strftime('%H:%M:%S')} ---")
        print(f"Center: 2.4 GHz | Span: 10 MHz")
        print(f"Marker Peak: {peak_amp.strip()} dBm at {float(peak_freq)/1e9:.4f} GHz")

    except pyvisa.VisaIOError as e:
        print(f"\nVISA Error: {e}")
    finally:
        if 'inst' in locals():
            inst.close()
            print("Connection closed.")

if __name__ == '__main__':
    main()