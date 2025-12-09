import numpy as np
import matplotlib.pyplot as plt

# --- Configuration ---
PHASE_STEP = 5.625  # Degrees per step
AMP_STEP = 0.254  # dB per step

# USER INPUT: Set the calibration steps for each port here.
# Format: [Port 1, Port 2, Port 3, Port 4]
# Example: [0, -2, 1, 0] means Port 2 is shifted -2 steps, Port 3 is +1 step.
port_phase_offsets = [1, 0, 0, -2] # best known so far is [1, 0, 0, -2]
port_amp_offsets = [0, 0, 2, 1] # best known so far is [0, 0, 2, 1]

# Define which pairs to analyze (Port A vs Port B)
pairs_to_check = [(1,2), (1,3), (1,4), (2,3), (2,4), (3,4)]

# --- Data Loading ---
data = {}
portkey = 'port{}'
valid_cols = range(9) # Columns to read

print("Loading files...")
for i in range(1, 5):
    filename = f'out{i}.csv'
    try:
        # Load columns: Freq (0), Re(S21) (5), Im(S21) (6)
        raw = np.loadtxt(filename, delimiter=',', skiprows=3, usecols=valid_cols)
        
        freqs = raw[:, 0]
        s_complex = raw[:, 5] + 1j * raw[:, 6]
        
        data[portkey.format(i)] = {'freqs': freqs, 'complex': s_complex}
        print(f"  {filename}: Loaded {len(freqs)} points")
    except OSError:
        print(f"  {filename}: Not found. Skipping.")

# --- Analysis & Plotting ---
print(f"\n--- Calibration Analysis ---")
print(f"Step Size: {PHASE_STEP}°")
print(f"Port Offsets (steps): {port_phase_offsets}")

plt.figure(figsize=(12, 7))

for pA, pB in pairs_to_check:
    kA, kB = portkey.format(pA), portkey.format(pB)
    
    if kA in data and kB in data:
        # 1. Calculate Raw Phase Difference (Phase A - Phase B)
        # We divide ComplexA / ComplexB to handle wrapping elegantly
        ratio = data[kA]['complex'] / data[kB]['complex']
        
        # Get angle and unwrap it to remove 360 jumps
        phase_diff_rad = np.angle(ratio)
        phase_diff_deg = np.degrees(np.unwrap(phase_diff_rad))
        
        # 2. Apply Calibration Correction
        # Correction = (Offset_A - Offset_B)
        # If we add phase to A, the difference (A-B) increases.
        # If we add phase to B, the difference (A-B) decreases.
        off_A = port_phase_offsets[pA-1] * PHASE_STEP
        off_B = port_phase_offsets[pB-1] * PHASE_STEP
        
        corrected_phase = phase_diff_deg + (off_A - off_B)
        
        # 3. Statistics
        avg_raw = np.mean(np.abs(phase_diff_deg))
        avg_corr = np.mean(np.abs(corrected_phase))
        
        print(f"Port {pA}-{pB}: Raw={avg_raw:6.2f}° | Offset=({off_A:+.1f} - {off_B:+.1f}) | Final={avg_corr:6.2f}°")
        
        # 4. Plot
        freqs_ghz = data[kA]['freqs'] / 1e9
        label_txt = f'P{pA}-P{pB} (Avg {avg_corr:.1f}°)'
        plt.plot(freqs_ghz, corrected_phase, linewidth=1.5, label=label_txt)

plt.xlabel('Frequency (GHz)')
plt.ylabel('Phase Difference (Degrees)')
plt.title(f'Phase Imbalance with Per-Port Calibration\nOffsets: {port_phase_offsets} x 5.625 degree step')
plt.grid(True, alpha=0.3, which='both')
plt.legend(loc='best')
plt.tight_layout()
plt.show(block=False)

# --- Analysis & Plotting ---
print(f"\n--- Amplitude Calibration Analysis ---")
print(f"Step Size: {AMP_STEP} dB")
print(f"Port Offsets (steps): {port_amp_offsets}")

plt.figure(figsize=(12, 7))

for pA, pB in pairs_to_check:
    kA, kB = portkey.format(pA), portkey.format(pB)
    
    if kA in data and kB in data:
        # 1. Calculate Ratio (Complex A / Complex B)
        ratio = data[kA]['complex'] / data[kB]['complex']
        
        # 2. Amplitude Imbalance (dB)
        # Formula: 20 * log10(|A/B|)
        raw_amp_imb = 20 * np.log10(np.abs(ratio))
        
        # 3. Apply Calibration Correction
        # Correction = (Offset_A - Offset_B)
        # Note: If Step_A is +1, we add 0.254 dB to Line A.
        off_A = port_amp_offsets[pA-1] * AMP_STEP
        off_B = port_amp_offsets[pB-1] * AMP_STEP
        
        corrected_amp_imb = raw_amp_imb + (off_A - off_B)
        
        # 4. Statistics
        avg_raw = np.mean(np.abs(raw_amp_imb))
        avg_corr = np.mean(np.abs(corrected_amp_imb))
        
        print(f"Port {pA}-{pB}: Raw={avg_raw:6.2f} dB | Offset=({off_A:+.3f} - {off_B:+.3f}) | Final={avg_corr:6.2f} dB")
        
        # 5. Plot
        freqs_ghz = data[kA]['freqs'] / 1e9
        label_txt = f'P{pA}-P{pB} (Avg {avg_corr:.2f} dB)'
        plt.plot(freqs_ghz, corrected_amp_imb, linewidth=1.5, label=label_txt)

plt.xlabel('Frequency (GHz)')
plt.ylabel('Amplitude Difference (dB)')
plt.title(f'Amplitude Imbalance with Per-Port Calibration\nOffsets: {port_amp_offsets} x 0.254 dB steps')
plt.grid(True, alpha=0.3, which='both')
plt.legend(loc='best')
plt.tight_layout()
plt.show()