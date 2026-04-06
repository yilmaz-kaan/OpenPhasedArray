"""
phase_sweep.py
──────────────────────────────────────────────────────────────────────────────
Sweeps all 4096 combinations of arithmetic phase progressions across four
phase-shifter channels and records the peak marker reading from the MS2690A
spectrum analyzer at each state.

Phase vector for combination (n, m):
    CH1=n,  CH2=(n+m)%64,  CH3=(n+2m)%64,  CH4=(n+3m)%64
    where n, m ∈ {0, 1, …, 63}

Instrument control
──────────────────
  • MS2690A spectrum analyzer  –  controlled via pyvisa (USB/LAN)
  • Phase shifters / Attenuators  –  FT232H SPI + SN74HCT138 mux

Sweep algorithm
───────────────
  1. Connect to SA; set center frequency, span, and continuous sweep.
  2. Zero all four attenuators (SPI).
  3. Optionally pause for a keypress so the user can verify SPI connectivity.
  4. Switch SA trace to Linear Average (LAV).
  5. Open a timestamped CSV file.
  6. For each (n, m) in {0..63} × {0..63}:
       a. Write phase vector to the four phase-shifter ICs.
       b. Wait PHASE_SETTLE_SEC for the analog state to settle.
       c. Reset the LAV average so the accumulation starts clean.
       d. Wait DWELL_SEC for the average to converge.
       e. Run a peak search; read marker frequency and power.
       f. Write the row to CSV and flush immediately.
  7. Clean up all hardware.

Usage
─────
    python phase_sweep.py

    # Pause after attenuator zero-check so you can verify SPI before starting:
    python phase_sweep.py --verify-spi

    # Override center frequency and span:
    python phase_sweep.py --center-freq 2.45e9 --span 20e6

    # Dry-run (no hardware required):
    python phase_sweep.py --dry-run
"""

from __future__ import annotations

import argparse
import csv
import logging
import os
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Tuple

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION  –  edit these before running
# ═══════════════════════════════════════════════════════════════════════════════

# ── Spectrum Analyzer ──────────────────────────────────────────────────────────
SA_RESOURCE_STRING = "USB0::2907::6::6261932852::0::INSTR"  # pyvisa resource
SA_CENTER_FREQ_HZ  = 2.345e9    # Hz  –  carrier / signal frequency
SA_SPAN_HZ         = 50e3      # Hz  –  span around center frequency

# ── Timing ─────────────────────────────────────────────────────────────────────
PHASE_SETTLE_SEC   = 0.05      # Seconds to let the analog phase state settle
                                # before resetting the average
DWELL_SEC          = 2.0       # Seconds of linear averaging per phase state

# ── Output ─────────────────────────────────────────────────────────────────────
LOG_DIR = Path("./phase_sweeps/")

# ═══════════════════════════════════════════════════════════════════════════════
# LOGGING
# ═══════════════════════════════════════════════════════════════════════════════

logging.basicConfig(
    level   = logging.INFO,
    format  = "%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt = "%H:%M:%S",
)
log = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════════════════
# DATA STRUCTURE
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class PhaseMeasurement:
    """One measured point in the phase-state sweep."""
    n:          int            # Base phase code   (0–63)
    m:          int            # Step  phase code   (0–63)
    ph1:        int            # Phase code sent to CH1  = n
    ph2:        int            # Phase code sent to CH2  = (n +  m) % 64
    ph3:        int            # Phase code sent to CH3  = (n + 2m) % 64
    ph4:        int            # Phase code sent to CH4  = (n + 3m) % 64
    freq_hz:    float          # Marker frequency (Hz) from SA
    power_dbm:  float          # Marker peak power (dBm) from SA
    timestamp:  str = field(default_factory=lambda: datetime.now().isoformat())

    def __str__(self) -> str:
        return (
            f"  n={self.n:2d}  m={self.m:2d}  "
            f"vec=[{self.ph1:2d},{self.ph2:2d},{self.ph3:2d},{self.ph4:2d}]  "
            f"{self.power_dbm:+.2f} dBm  @ {self.freq_hz/1e9:.6f} GHz"
        )


# ═══════════════════════════════════════════════════════════════════════════════
# SPECTRUM ANALYZER
# ═══════════════════════════════════════════════════════════════════════════════

class SpectrumAnalyzer:
    """
    MS2690A VISA wrapper.

    Responsibilities
    ────────────────
      • Set center frequency and span on connect.
      • Run a continuous normal sweep initially (so the display is live while
        the user is verifying SPI connectivity).
      • Switch to Linear Average (LAV) mode before the sweep begins.
      • Reset the LAV accumulation at the start of each phase state.
      • Run a peak marker and return (frequency_hz, power_dbm).
    """

    def __init__(
        self,
        resource_string: str,
        center_freq_hz:  float = SA_CENTER_FREQ_HZ,
        span_hz:         float = SA_SPAN_HZ,
    ) -> None:
        self._resource = resource_string
        self._center   = center_freq_hz
        self._span     = span_hz
        self._inst     = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def connect(self) -> None:
        import pyvisa
        rm = pyvisa.ResourceManager()
        log.info("Connecting to SA: %s", self._resource)
        self._inst = rm.open_resource(self._resource)
        self._inst.timeout = 15_000   # ms

        idn = self._inst.query("*IDN?").strip()
        log.info("SA identified: %s", idn)

        # Set frequency context
        self._inst.write(f"FREQ:CENT {self._center:.6e}")
        self._inst.write(f"FREQ:SPAN {self._span:.6e}")
        log.info("SA  center=%.6e Hz  span=%.6e Hz", self._center, self._span)

        # Start a continuous normal (non-averaged) sweep so the display is live
        # during the SPI verification pause.
        self._inst.write("TRACe1:STORage:MODE NORMal")
        self._inst.write("INITiate:CONTinuous ON")

        # Enable a normal-mode marker (used for peak queries)
        self._inst.write("CALC:MARK:STAT ON")
        self._inst.write("CALC:MARK:MODE NORM")

        log.info("SA continuous normal sweep active.")

    def close(self) -> None:
        if self._inst is not None:
            try:
                self._inst.close()
                log.info("SA connection closed.")
            except Exception:
                pass
            self._inst = None

    def __enter__(self) -> "SpectrumAnalyzer":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # ── Public API ─────────────────────────────────────────────────────────────

    def enable_linear_average(self) -> None:
        """
        Switch the trace to Linear Average (LAV) mode and set the average count.
        The MS2690A command TRACe1:STORage:MODE LAV selects linear (power)
        averaging;
        Continuous sweep must remain ON so averaging runs automatically.
        """
        self._inst.write("TRACe1:STORage:MODE LAV")
        self._inst.write("INITiate:CONTinuous ON")
        log.info("SA switched to Linear Average (LAV),")

    def reset_average(self) -> None:
        """
        Discard the current accumulated average and start a fresh accumulation.
        Re-issuing the LAV storage mode command causes the SA to clear the
        prior average trace and begin accumulating from the very next sweep —
        identical in spirit to the reset_max_hold() approach used by
        azimuth_pattern_sweep.py.
        """
        self._inst.write("TRACe1:STORage:MODE LAV")

    def measure_peak(self) -> Tuple[float, float]:
        """
        Run a peak search on the current (already-averaged) trace and return
        (frequency_hz, power_dbm).
        """
        self._inst.write("CALC:MARK:MAX")
        freq_hz   = float(self._inst.query("CALC:MARK:X?").strip())
        power_dbm = float(self._inst.query("CALC:MARK:Y?").strip())
        return freq_hz, power_dbm


# ═══════════════════════════════════════════════════════════════════════════════
# DRY-RUN SA STUB
# ═══════════════════════════════════════════════════════════════════════════════

class _DryRunSA:
    """Simulates the SA with a simple phase-dependent power model."""

    def __init__(self, center_freq_hz: float = SA_CENTER_FREQ_HZ, **_) -> None:
        self._center = center_freq_hz
        self._ph1 = self._ph2 = self._ph3 = self._ph4 = 0

    def connect(self)  -> None: log.info("[DRY RUN] SA connected (simulated).")
    def close(self)    -> None: log.info("[DRY RUN] SA closed.")
    def __enter__(self):  self.connect(); return self
    def __exit__(self, *_): self.close()

    def enable_linear_average(self) -> None:
        log.info("[DRY RUN] SA LAV enabled.")

    def reset_average(self) -> None:
        pass   # no-op in dry-run

    def set_phase_hint(self, ph1, ph2, ph3, ph4) -> None:
        """Called by the sweep engine so the stub can fake a realistic reading."""
        self._ph1, self._ph2, self._ph3, self._ph4 = ph1, ph2, ph3, ph4

    def measure_peak(self) -> Tuple[float, float]:
        import math, random
        # Phase alignment quality: maximum power when all phases equal (m=0)
        diff = ((self._ph2 - self._ph1) % 64
               + (self._ph3 - self._ph1) % 64
               + (self._ph4 - self._ph1) % 64)
        alignment = math.cos(math.pi * diff / 96)
        power = -40.0 + 20.0 * (alignment ** 2) + random.gauss(0, 0.5)
        return self._center, power


# ═══════════════════════════════════════════════════════════════════════════════
# FT232H / SPI HARDWARE  (wraps PhasedArrayControl logic)
# ═══════════════════════════════════════════════════════════════════════════════

class PhasedArrayHardware:
    """
    Manages the FT232H SPI bus and SN74HCT138 demultiplexer.

    This is a thin class-based wrapper around the functions defined in
    PhasedArrayControl.py.  Keeping hardware state in an object makes
    it easy to initialise once and clean up safely in a finally block.
    """

    def __init__(self) -> None:
        self._decoder_pins = None
        self._spi          = None

    def connect(self) -> None:
        os.environ["BLINKA_FT232H"] = "1"

        import board
        import busio
        import digitalio

        # Store module refs for use in close()
        self._digitalio = digitalio
        self._board     = board

        # ── Decoder (SN74HCT138) ──────────────────────────────────────────────
        PIN_C  = board.C0
        PIN_B  = board.C1
        PIN_A  = board.C2
        PIN_G1 = board.C3

        pin_c  = digitalio.DigitalInOut(PIN_C)
        pin_b  = digitalio.DigitalInOut(PIN_B)
        pin_a  = digitalio.DigitalInOut(PIN_A)
        pin_g1 = digitalio.DigitalInOut(PIN_G1)

        for pin in (pin_c, pin_b, pin_a, pin_g1):
            pin.direction = digitalio.Direction.OUTPUT

        pin_g1.value = True   # enable decoder
        self._decoder_pins = [pin_c, pin_b, pin_a, pin_g1]
        log.info("SN74HCT138 decoder initialised (G1=HIGH).")

        # ── SPI bus ───────────────────────────────────────────────────────────
        self._spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        log.info("FT232H SPI bus open.")

    def close(self) -> None:
        if self._decoder_pins:
            self._disable_decoder()
            for pin in self._decoder_pins:
                pin.deinit()
            self._decoder_pins = None
        if self._spi:
            self._spi.deinit()
            self._spi = None
        log.info("FT232H hardware de-initialised.")

    def __enter__(self) -> "PhasedArrayHardware":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # ── Private helpers (mirror PhasedArrayControl.py internals) ──────────────

    @staticmethod
    def _reverse_bits(byte_val: int) -> int:
        """Reverse 8 bits (LSB-first workaround for busio.SPI MSB-first)."""
        byte_val = ((byte_val & 0xF0) >> 4) | ((byte_val & 0x0F) << 4)
        byte_val = ((byte_val & 0xCC) >> 2) | ((byte_val & 0x33) << 2)
        byte_val = ((byte_val & 0xAA) >> 1) | ((byte_val & 0x55) << 1)
        return byte_val

    def _select_output(self, output_index: int) -> None:
        pin_c, pin_b, pin_a, pin_g1 = self._decoder_pins
        pin_g1.value = True
        bits = format(output_index, "03b")
        pin_c.value = bits[0] == "1"
        pin_b.value = bits[1] == "1"
        pin_a.value = bits[2] == "1"

    def _disable_decoder(self) -> None:
        _, _, _, pin_g1 = self._decoder_pins
        pin_g1.value = False

    def _spi_write(self, data: bytearray) -> None:
        while not self._spi.try_lock():
            pass
        try:
            self._spi.write(data)
        finally:
            self._spi.unlock()

    CMD_THRESHOLD = 45e-9  # 45 ns minimum between commands

    def send_command(self, device_type: str, device_id: int, value: int) -> None:
        """
        Send an SPI command to a phase shifter ('P') or attenuator ('A').

        device_type : 'P' or 'A'
        device_id   : 1–4
        value       : 0–63 for phase shifters; 0–127 for attenuators
        """
        t_start = time.monotonic()

        if device_type == "P":
            target_y   = 8 - device_id
            data_bytes = bytearray([value])

        elif device_type == "A":
            target_y      = 4 - device_id
            data_bytes    = bytearray([
                self._reverse_bits(value),
                self._reverse_bits(0x00),
            ])

        else:
            raise ValueError(f"Unknown device type: {device_type!r}")

        self._select_output(target_y)
        self._spi_write(data_bytes)
        self._disable_decoder()   # rising edge on CS latches the data

        elapsed = time.monotonic() - t_start
        if elapsed < self.CMD_THRESHOLD:
            time.sleep(self.CMD_THRESHOLD - elapsed)

    def zero_all_attenuators(self) -> None:
        """Set all four attenuator ICs to 0 dB attenuation."""
        log.info("Zeroing all attenuators …")
        for dev_id in range(1, 5):
            self.send_command("A", dev_id, 0)
            log.info("  Attenuator A%d → 0", dev_id)
        log.info("All attenuators zeroed.")

    def set_phase_vector(self, ph1: int, ph2: int, ph3: int, ph4: int) -> None:
        """Write all four phase-shifter codes in order."""
        for dev_id, code in enumerate((ph1, ph2, ph3, ph4), start=1):
            self.send_command("P", dev_id, code)


# ═══════════════════════════════════════════════════════════════════════════════
# DRY-RUN HARDWARE STUB
# ═══════════════════════════════════════════════════════════════════════════════

class _DryRunHardware:
    """Simulates FT232H SPI without real hardware."""

    def connect(self)  -> None: log.info("[DRY RUN] FT232H hardware connected (simulated).")
    def close(self)    -> None: log.info("[DRY RUN] FT232H hardware closed.")
    def __enter__(self):  self.connect(); return self
    def __exit__(self, *_): self.close()

    def send_command(self, device_type: str, device_id: int, value: int) -> None:
        log.debug("[DRY RUN] SPI  %s%d → %d", device_type, device_id, value)

    def zero_all_attenuators(self) -> None:
        log.info("[DRY RUN] All attenuators zeroed.")

    def set_phase_vector(self, ph1, ph2, ph3, ph4) -> None:
        log.debug("[DRY RUN] Phase vector [%d, %d, %d, %d]", ph1, ph2, ph3, ph4)


# ═══════════════════════════════════════════════════════════════════════════════
# CSV LOGGER
# ═══════════════════════════════════════════════════════════════════════════════

class MeasurementLogger:
    """Writes each PhaseMeasurement to a CSV immediately (flush-after-write)."""

    HEADER = [
        "timestamp",
        "n", "m",
        "ph1", "ph2", "ph3", "ph4",
        "power_dbm", "freq_hz",
    ]

    def __init__(self, filepath: Path) -> None:
        self._filepath = filepath
        self._file     = None
        self._writer   = None

    def open(self) -> None:
        self._file   = open(self._filepath, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self._file.flush()
        log.info("CSV log opened: %s", self._filepath)

    def close(self) -> None:
        if self._file:
            self._file.close()
            log.info("CSV log closed: %s", self._filepath)

    def __enter__(self) -> "MeasurementLogger":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def record(self, meas: PhaseMeasurement) -> None:
        self._writer.writerow([
            meas.timestamp,
            meas.n,   meas.m,
            meas.ph1, meas.ph2, meas.ph3, meas.ph4,
            f"{meas.power_dbm:.4f}",
            f"{meas.freq_hz:.2f}",
        ])
        self._file.flush()   # safe to Ctrl-C at any point after this line


# ═══════════════════════════════════════════════════════════════════════════════
# SWEEP ENGINE
# ═══════════════════════════════════════════════════════════════════════════════

def run_phase_sweep(
    sa:               "SpectrumAnalyzer | _DryRunSA",
    hw:               "PhasedArrayHardware | _DryRunHardware",
    log_dir:          Path  = LOG_DIR,
    phase_settle_sec: float = PHASE_SETTLE_SEC,
    dwell_sec:        float = DWELL_SEC,
    verify_spi:       bool  = False,
) -> List[PhaseMeasurement]:
    """
    Main sweep routine.  Expects *sa* and *hw* to already be connected.

    Parameters
    ──────────
    sa               Open SA instance.
    hw               Open hardware instance.
    log_dir          Directory for CSV output.
    phase_settle_sec Delay after writing phase codes before resetting average.
    dwell_sec        Duration of LAV averaging per state.
    verify_spi       If True, pause for a keypress after zeroing attenuators.

    Returns a list of all PhaseMeasurements collected (may be partial on
    early exit via KeyboardInterrupt).
    """
    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = log_dir / f"phase_sweep_{ts}.csv"

    # ── Step 1: Zero attenuators ──────────────────────────────────────────────
    hw.zero_all_attenuators()

    # ── Step 2: Optional SPI verification pause ───────────────────────────────
    if verify_spi:
        print()
        print("=" * 60)
        print("  SPI VERIFICATION PAUSE")
        print("  All attenuators have been set to 0.")
        print("  Verify the SPI connection is correct on your hardware,")
        print("  then press ENTER to continue with the phase sweep.")
        print("=" * 60)
        input("  [Press ENTER to continue] ")
        print()

    # ── Step 3: Switch SA to Linear Average ───────────────────────────────────
    sa.enable_linear_average()

    # ── Step 4: Run sweep ─────────────────────────────────────────────────────
    results: List[PhaseMeasurement] = []
    total   = 64 * 64   # 4096 combinations

    try:
        with MeasurementLogger(csv_path) as logger:

            combo_index = 0
            for n in range(64):
                for m in range(64):
                    combo_index += 1

                    ph1 = n
                    ph2 = (n +     m) % 64
                    ph3 = (n + 2 * m) % 64
                    ph4 = (n + 3 * m) % 64

                    log.info(
                        "[%4d/%d]  n=%2d  m=%2d  →  [%2d, %2d, %2d, %2d]",
                        combo_index, total, n, m, ph1, ph2, ph3, ph4,
                    )

                    # (a) Write phase codes to all four ICs
                    hw.set_phase_vector(ph1, ph2, ph3, ph4)

                    # Hint to dry-run SA so it can produce a phase-dependent
                    # reading; no-op for real hardware.
                    if hasattr(sa, "set_phase_hint"):
                        sa.set_phase_hint(ph1, ph2, ph3, ph4)

                    # (b) Short settle delay before disturbing the average
                    time.sleep(phase_settle_sec)

                    # (c) Reset LAV so the average starts from this phase state
                    sa.reset_average()

                    # (d) Dwell while the SA accumulates a fresh average
                    time.sleep(dwell_sec)

                    # (e) Peak search
                    freq_hz, power_dbm = sa.measure_peak()

                    meas = PhaseMeasurement(
                        n=n, m=m,
                        ph1=ph1, ph2=ph2, ph3=ph3, ph4=ph4,
                        freq_hz=freq_hz,
                        power_dbm=power_dbm,
                    )
                    log.info("  %s", meas)

                    # (f) Flush to CSV
                    logger.record(meas)
                    results.append(meas)

    except KeyboardInterrupt:
        log.warning("Sweep interrupted by user after %d/%d states.", len(results), total)

    log.info("Sweep complete.  %d measurements written to: %s", len(results), csv_path)
    return results


# ═══════════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sweep 4096 arithmetic phase-vector combinations and record "
            "SA peak power for each state."
        )
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Simulate all hardware (no FT232H or SA required).",
    )
    parser.add_argument(
        "--verify-spi", action="store_true",
        help=(
            "Pause for a keypress after zeroing attenuators so the user can "
            "verify the SPI connection before the sweep begins."
        ),
    )
    parser.add_argument(
        "--center-freq", type=float, default=SA_CENTER_FREQ_HZ,
        metavar="HZ",
        help=f"SA center frequency in Hz (default: {SA_CENTER_FREQ_HZ:.3e}).",
    )
    parser.add_argument(
        "--span", type=float, default=SA_SPAN_HZ,
        metavar="HZ",
        help=f"SA span in Hz (default: {SA_SPAN_HZ:.3e}).",
    )
    parser.add_argument(
        "--dwell", type=float, default=DWELL_SEC,
        metavar="SEC",
        help=f"Dwell time per phase state in seconds (default: {DWELL_SEC}).",
    )
    parser.add_argument(
        "--settle", type=float, default=PHASE_SETTLE_SEC,
        metavar="SEC",
        help=(
            f"Phase-settle delay before resetting average (default: {PHASE_SETTLE_SEC})."
        ),
    )
    parser.add_argument(
        "--log-dir", type=Path, default=LOG_DIR,
        help=f"Output directory for the CSV file (default: {LOG_DIR}).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    log.info("=" * 60)
    log.info("Phase Sweep  –  %s", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    log.info("  Center freq : %.6e Hz", args.center_freq)
    log.info("  Span        : %.6e Hz", args.span)
    log.info("  Dwell       : %.2f s per state", args.dwell)
    log.info("  Settle      : %.3f s", args.settle)
    log.info("  Total states: 4096  (64 × 64)")
    log.info("  Est. runtime: ~%.0f min", 4096 * (args.dwell + args.settle) / 60)
    log.info("=" * 60)

    if args.dry_run:
        log.warning("DRY-RUN mode: using simulated instruments.")
        sa = _DryRunSA(center_freq_hz=args.center_freq)
        hw = _DryRunHardware()
    else:
        sa = SpectrumAnalyzer(
            resource_string = SA_RESOURCE_STRING,
            center_freq_hz  = args.center_freq,
            span_hz         = args.span,
        )
        hw = PhasedArrayHardware()

    try:
        with sa, hw:
            run_phase_sweep(
                sa               = sa,
                hw               = hw,
                log_dir          = args.log_dir,
                phase_settle_sec = args.settle,
                dwell_sec        = args.dwell,
                verify_spi       = args.verify_spi,
            )
    except KeyboardInterrupt:
        log.warning("Aborted by user.")
        sys.exit(1)
    except Exception as exc:
        log.exception("Fatal error: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()