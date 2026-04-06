import numpy as np

def calculate_phase_shifts(frequency, element_spacing, num_elements, steering_angle_deg):
    c = 3e8  # Speed of light (m/s)
    wavelength = c / frequency

    theta = np.radians(steering_angle_deg)

    # Phase shift between adjacent elements
    delta_phi = (2 * np.pi * element_spacing * np.sin(theta)) / wavelength

    # Centered array (recommended)
    element_indices = np.arange(num_elements) - (num_elements - 1) / 2

    # Phase shifts (radians)
    phases_rad = element_indices * delta_phi

    # Normalize to [-pi, pi]
    phases_rad = np.mod(phases_rad + np.pi, 2 * np.pi) - np.pi

    return phases_rad


def phase_to_6bit(phases_rad):
    """
    Convert phase (radians) → 6-bit values (0–63)
    """
    # Convert to degrees
    phases_deg = np.degrees(phases_rad)

    # Wrap to [0, 360)
    phases_deg = np.mod(phases_deg, 360)

    # Quantization step
    step_size = 360 / 64  # 5.625 degrees

    # Convert to integer values
    phase_codes = np.round(phases_deg / step_size).astype(int) % 64

    return phases_deg, phase_codes


if __name__ == "__main__":
    # Example parameters
    # positive angle is clockwise, negative angle is counterclockwise
    frequency = 2.4e9           # 10 GHz
    element_spacing = 0.054   # 1.5 cm
    num_elements = 4
    steering_angle_deg = 30

    phases_rad = calculate_phase_shifts(
        frequency,
        element_spacing,
        num_elements,
        steering_angle_deg
    )

    phases_deg, phase_codes = phase_to_6bit(phases_rad)

    print("Element | Phase (deg) | Phase Shifter Value (0–63)")
    print("--------------------------------------------------")
    for i in range(num_elements):
        print(f"{i:7d} | {phases_deg[i]:10.2f} | {phase_codes[i]:>5d}")