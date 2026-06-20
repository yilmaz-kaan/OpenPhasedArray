# beamers
Rice University 2025-2026 ECE Senior Capstone Project Repository
Team BEAMFORM

The OpenPhasedArray is an open-source, modular 4-element transceiver phased-array
board adaptable across software-defined radio platforms and optimized for S-band
frequencies. It is ideal for hobby or research-grade beamforming experiments or sensing
and communications prototyping. It is a modular design that is easily stackable both
vertically and horizontally using the provided 3D-printable board mount. It is (optionally) interfaced with by the FT232H USB-to-serial
converter for easy, Python-based control. 

**Specification and Info**
- 2.4 – 3.7 GHz operating frequency
- Small form factor, 4.128” x 3.534”
- 21 dB peak gain per channel in TX
- 14 dB peak gain per channel in RX
- 5.625° nominal phase shifter resolution with 64 states
- 0.254 dB attenuator resolution with 128 states
- 500 ns typical TX/RX switching time
- 6V DC Power thru barrel jack-- 16V maximum
- <1mA quiescent current, 700mA maximum @ 6V input

**Repo Contents**
- Hardware
    - OpenPhasedArray schematic, design, and gerber files
    - Evaluation boards for most RF components used on the OpenPhasedArray
    - An 8-element microstrip patch antenna array that can be paired with the OpenPhasedArray
    - The original 'TXBoard' Prototype schematic, design, and gerber files
    - The secondary 'TXBoard2' Prototype schematic, design, and gerber files
    - 'STM32EVAL,' an experiment for future revisions that allow much larger scale integration
- Software
    - Control code for various versions of the board, except for Rev. 2 due to layout errors.
    - Leftover code and scripts from testing and demonstrating board performance in an Anechoic chamber
