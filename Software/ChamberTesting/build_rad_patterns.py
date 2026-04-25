import os
import glob
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# --- Configuration ---
LUT_FILE = 'beamformer_lut.json'
SWEEP_BASE_DIR = './sweep_results'
OUTPUT_DIR = './extracted_patterns_polar'
MIN_DBM_FLOOR = -110  # Sets the center of the polar plot to -110 dBm

def main():
    # 1. Create the output directory
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"Created output directory: {OUTPUT_DIR}")

    # 2. Load the Beamformer Lookup Table
    with open(LUT_FILE, 'r') as f:
        lut_data = json.load(f)
    print(f"Loaded LUT with {len(lut_data)} steering angles.")

    # 3. Discover and load the most recent CSV for each True Measurement Angle
    angle_dfs = {}
    
    for folder_name in os.listdir(SWEEP_BASE_DIR):
        if not folder_name.startswith('ang_'):
            continue
            
        folder_path = os.path.join(SWEEP_BASE_DIR, folder_name)
        if not os.path.isdir(folder_path):
            continue
            
        try:
            dir_angle = int(folder_name.replace('ang_', ''))
        except ValueError:
            continue
            
        # Apply the 30-degree measurement fixture offset
        true_angle = dir_angle + 30
        
        csv_files = glob.glob(os.path.join(folder_path, '*.csv'))
        if not csv_files:
            continue
            
        # Sort and pick the most recent CSV
        latest_csv = sorted(csv_files)[-1]
        
        df = pd.read_csv(latest_csv)
        angle_dfs[true_angle] = df

    # 4. Extract and plot the polar radiation pattern for each steered angle in the LUT
    for steer_angle_str, lut_entry in lut_data.items():
        steer_angle = int(steer_angle_str)
        
        best_state = lut_entry['max_power']
        target_n = best_state['n']
        target_m = best_state['m']
        
        pattern_data = []
        
        for meas_angle, df in angle_dfs.items():
            row = df[(df['n'] == target_n) & (df['m'] == target_m)]
            
            if not row.empty:
                power = row['power_dbm'].values[0]
                pattern_data.append({
                    'Measurement_Angle': meas_angle,
                    'Power_dBm': power
                })
        
        if not pattern_data:
            print(f"Could not find data for steering angle {steer_angle}°")
            continue
            
        pattern_df = pd.DataFrame(pattern_data).sort_values('Measurement_Angle')
        
        # Save to CSV
        csv_filename = os.path.join(OUTPUT_DIR, f'polar_steer_{steer_angle:03d}deg.csv')
        pattern_df.to_csv(csv_filename, index=False)
        
        # --- Prepare Data for Polar Plot ---
        # Convert degrees to radians for Matplotlib polar
        theta = np.deg2rad(pattern_df['Measurement_Angle'])
        
        # Shift powers to avoid negative radii issues in Matplotlib
        # Clip at 0 so anything below the floor just plots at the origin
        r_shifted = pattern_df['Power_dBm'] - MIN_DBM_FLOOR
        r_shifted = np.clip(r_shifted, a_min=0, a_max=None)
        
        # Generate the polar plot
        fig, ax = plt.subplots(figsize=(8, 8), subplot_kw={'projection': 'polar'})
        
        # Plot the data
        ax.plot(theta, r_shifted, marker='o', linestyle='-', color='b', linewidth=2)
        ax.fill(theta, r_shifted, color='b', alpha=0.2)
        
        # Set Top (North) as 0 degrees
        ax.set_theta_zero_location("N")
        # Set angles to increase clockwise
        ax.set_theta_direction(-1)
        
        # Optional: Limit the visible polar angle to a forward sector if your 
        # sweeps only go from -60 to 60. E.g., setting it to show -90 to 90
        ax.set_thetamin(-90)
        ax.set_thetamax(90)
        
        # Add target steering direction line
        target_theta = np.deg2rad(steer_angle)
        ax.plot([target_theta, target_theta], [0, max(r_shifted)+10], color='r', linestyle='--', alpha=0.8, 
                label=f'Target Direction ({steer_angle}°)')
        
        # Fix the radial ticks to show actual dBm instead of the shifted positive values
        # e.g., generating ticks every 10 dB from the floor up to the max shifted value
        max_r = max(r_shifted) if max(r_shifted) > 0 else 10
        r_ticks_shifted = np.arange(0, max_r + 10, 10)
        ax.set_yticks(r_ticks_shifted)
        
        # Map the shifted ticks back to the true dBm strings
        true_dbm_labels = [f"{int(t + MIN_DBM_FLOOR)} dBm" for t in r_ticks_shifted]
        ax.set_yticklabels(true_dbm_labels)
        
        plt.title(f'Polar Radiation Pattern\nTarget Angle: {steer_angle}°', va='bottom')
        plt.legend(loc='upper right', bbox_to_anchor=(1.2, 1.1))
        plt.tight_layout()
        
        plot_filename = os.path.join(OUTPUT_DIR, f'polar_steer_{steer_angle:03d}deg.png')
        plt.savefig(plot_filename, dpi=150)
        plt.close()

    print("\nPolar pattern extraction complete! Check the 'extracted_patterns_polar' directory.")

if __name__ == "__main__":
    main()