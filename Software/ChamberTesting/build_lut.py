import os
import glob
import pandas as pd
import json

def build_beamformer_lut(sweep_dir="./sweep_results", offset=30):
    lut_data = {}
    
    # Check if we are already inside the sweep_results directory
    if not os.path.exists(sweep_dir):
        # Check if the directories exist right here in the current folder
        if glob.glob("ang_*"):
            print(f"Found 'ang_' folders in current directory. Adjusting sweep_dir to '.'")
            sweep_dir = "."
        else:
            print(f"ERROR: Cannot find '{sweep_dir}'. Current working directory is: {os.getcwd()}")
            return

    print(f"Searching for directories in: {os.path.abspath(sweep_dir)}")
    dir_paths = glob.glob(os.path.join(sweep_dir, "ang_*"))
    
    if not dir_paths:
        print("ERROR: Found 0 directories matching 'ang_*'. Check your folder names.")
        return
        
    for dir_path in dir_paths:
        dir_name = os.path.basename(dir_path)
        
        # Verbose parsing to catch errors
        raw_angle_str = dir_name.replace("ang_", "")
        try:
            raw_angle = int(raw_angle_str)
        except ValueError:
            print(f"SKIPPED: Could not convert '{raw_angle_str}' to an integer in folder '{dir_name}'")
            continue 
            
        actual_angle = raw_angle + offset
        
        csv_files = glob.glob(os.path.join(dir_path, "phase_sweep_*.csv"))
        if not csv_files:
            print(f"Warning: No CSV found in {dir_name}")
            continue
            
        df = pd.read_csv(csv_files[0])
        max_row = df.loc[df['power_dbm'].idxmax()]
        min_row = df.loc[df['power_dbm'].idxmin()]
        
        def extract_state(row):
            return {
                "n": int(row['n']), "m": int(row['m']),
                "ph1": int(row['ph1']), "ph2": int(row['ph2']),
                "ph3": int(row['ph3']), "ph4": int(row['ph4']),
                "power_dbm": float(row['power_dbm'])
            }
            
        lut_data[actual_angle] = {
            "max_power": extract_state(max_row),
            "min_power": extract_state(min_row)
        }
        
        print(f"Processed raw angle {raw_angle:4d}° -> actual {actual_angle:4d}°")

    if not lut_data:
        print("\nFailed to build LUT. No valid data extracted.")
        return

    sorted_lut = {k: lut_data[k] for k in sorted(lut_data.keys())}
    
    with open("beamformer_lut.json", "w") as f:
        json.dump(sorted_lut, f, indent=4)
        
    print(f"\nSuccessfully built LUT for {len(sorted_lut)} angles.")
    print("Saved to beamformer_lut.json")

if __name__ == "__main__":
    # You can hardcode your path here if it's consistently failing
    # build_beamformer_lut(sweep_dir="/absolute/path/to/your/sweep_results")
    build_beamformer_lut()