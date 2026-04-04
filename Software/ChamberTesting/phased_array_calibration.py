"""
phased_array_calibration.py
─────────────────────────────────────────────────────────────────────────────
Calibrates a 4-element phased array by finding the phase-offset vector that
maximizes received signal power, as measured by an MS2690A spectrum analyzer.

Algorithm: Two-Phase Coordinate Descent
────────────────────────────────────────
Exhaustive search over 64^4 = 16,777,216 states is infeasible at ~200 ms per
measurement (~39 days).  Coordinate descent exploits the fact that received
power is a smooth, roughly separable function of each phase shifter:

  Phase 1 – Coarse (step = 8)
    Sweep each of the 4 shifters through {0, 8, 16, …, 56} while holding the
    others fixed.  8 × 4 = 32 measurements per round.  Repeat until no
    improvement is observed (typically 3–5 rounds).

  Phase 2 – Fine (step = 1)
    Re-initialize from the Phase 1 result and sweep all 64 states per
    shifter.  64 × 4 = 256 measurements per round.  Repeat until convergence.

Total: ~1 000–1 500 measurements vs. 16 777 216 for exhaustive search.
Every measurement and the final calibration result are written to a CSV log.

Hardware dependencies
─────────────────────
  • Adafruit Blinka / FT232H  (phased array SPI + decoder)
  • pyvisa                    (MS2690A spectrum analyzer)

Usage
─────
  Adjust CONFIGURATION constants below, then:
      python phased_array_calibration.py

  Use --dry-run to simulate hardware with random noise (unit-testing without
  physical instruments):
      python phased_array_calibration.py --dry-run
"""

from __future__ import annotations

import argparse
import csv
import logging
import os
import random
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple


# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION  –  edit these values before running
# ═══════════════════════════════════════════════════════════════════════════════

# ── Spectrum Analyzer ──────────────────────────────────────────────────────────
SA_RESOURCE_STRING = 'USB0::2907::6::6261932852::0::INSTR'  # VISA resource
SA_CENTER_FREQ_HZ  = 2.345e9   # Center frequency (Hz)
SA_SPAN_HZ         = 100e3     # Span around center (Hz)
SA_SETTLE_SEC      = 1         # Wait time after re-configuring before querying
                               # the marker.  Increase if readings are noisy.

# ── Phased Array (FT232H / SPI) ───────────────────────────────────────────────
NUM_PHASE_SHIFTERS = 4         # Number of phase shifters (1-indexed, 1–4)
PHASE_STATES       = 64        # States per shifter (0–63, 6-bit RFSA device)
CMD_THRESHOLD_SEC  = 45e-9     # Minimum time between SPI commands (45 ns)

# ── Calibration Algorithm ─────────────────────────────────────────────────────
COARSE_STEP        = 4         # Step size for Phase 1 sweep  (must divide 64)
MAX_COARSE_ROUNDS  = 10        # Safety cap on Phase 1 iterations
MAX_FINE_ROUNDS    = 10        # Safety cap on Phase 2 iterations

# ── Output ────────────────────────────────────────────────────────────────────
LOG_DIR = Path("./cals/")            # Directory for CSV log files


# ═══════════════════════════════════════════════════════════════════════════════
# LOGGING SETUP
# ═══════════════════════════════════════════════════════════════════════════════

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════════════════
# DATA STRUCTURES
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class Measurement:
    """A single power measurement associated with a complete phase vector."""
    phases: List[int]           # Phase state for each shifter, e.g. [0, 32, 8, 16]
    power_dbm: float            # Measured peak power (dBm)
    freq_hz: float              # Marker frequency returned by the SA (Hz)
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())

    def __str__(self) -> str:
        phases_str = ", ".join(f"P{i+1}={v:2d}" for i, v in enumerate(self.phases))
        return f"[{phases_str}]  →  {self.power_dbm:+.2f} dBm @ {self.freq_hz/1e9:.6f} GHz"


# ═══════════════════════════════════════════════════════════════════════════════
# SPECTRUM ANALYZER CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════

class SpectrumAnalyzer:
    """
    Thin wrapper around an MS2690A (or compatible) spectrum analyzer via VISA.

    The SA is configured once on connect(), then measure_peak() can be called
    repeatedly to retrieve the marker frequency and amplitude after each
    phase-vector update.
    """

    def __init__(self, resource_string: str, center_freq_hz: float,
                 span_hz: float, settle_sec: float = 0.15) -> None:
        self._resource_string = resource_string
        self._center_freq_hz  = center_freq_hz
        self._span_hz         = span_hz
        self._settle_sec      = settle_sec
        self._inst            = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Open a VISA connection and configure the SA for peak-power measurement."""
        import pyvisa  # deferred so --dry-run works without pyvisa installed
        rm = pyvisa.ResourceManager()
        log.info("Connecting to spectrum analyzer: %s", self._resource_string)
        self._inst = rm.open_resource(self._resource_string)
        self._inst.timeout = 10_000  # ms

        idn = self._inst.query("*IDN?").strip()
        log.info("Connected: %s", idn)

        self._configure()

    def close(self) -> None:
        if self._inst is not None:
            try:
                self._inst.close()
                log.info("Spectrum analyzer connection closed.")
            except Exception:
                pass
            self._inst = None

    def __enter__(self) -> "SpectrumAnalyzer":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # ── Private helpers ────────────────────────────────────────────────────────

    def _configure(self) -> None:
        """One-time SA configuration: mode, frequency, span, trace, marker."""
        inst = self._inst
        inst.write("INST:SYST SPECT,ACT")
        inst.write(f"FREQuency:CENTer {self._center_freq_hz}")
        inst.write(f"FREQuency:SPAN {self._span_hz}")

        # Max Hold accumulates the highest power seen across sweeps.
        inst.write("TRACe1:STORage:MODE MAXHold")
        inst.write("INITiate:CONTinuous ON")

        inst.write("CALC:MARK:STAT ON")
        inst.write("CALC:MARK:MODE NORM")

        actual_freq = inst.query("FREQuency:CENTer?").strip()
        actual_span = inst.query("FREQuency:SPAN?").strip()
        log.info("SA configured  –  center=%.6f GHz  span=%.3f kHz",
                 float(actual_freq) / 1e9, float(actual_span) / 1e3)

    # ── Public API ─────────────────────────────────────────────────────────────

    def measure_peak(self) -> Tuple[float, float]:
        """
        Wait for the SA to settle, run a peak search, and return
        (peak_frequency_hz, peak_amplitude_dbm).
        """
        self._inst.write("TRACe1:STORage:MODE MAXHold") # re-initialize the max hold to avoid old peaks lingering across measurements
        time.sleep(self._settle_sec)
        self._inst.write("CALC:MARK:MAX")
        freq_hz  = float(self._inst.query("CALC:MARK:X?"))
        power_dbm = float(self._inst.query("CALC:MARK:Y?"))
        return freq_hz, power_dbm


# ═══════════════════════════════════════════════════════════════════════════════
# PHASED ARRAY CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════

class PhasedArrayController:
    """
    Controls 4 RFSA3713 6-bit phase shifters via SPI through an FT232H and a
    SN74HCT138 3-to-8 decoder that handles chip-select demultiplexing.

    Pin mapping (FT232H)
    ────────────────────
      C0 → decoder SEL_C (MSB)
      C1 → decoder SEL_B
      C2 → decoder SEL_A (LSB)
      C3 → decoder G1 (active-high enable / latch)
      D0 → SPI CLK
      D1 → SPI MOSI
      D2 → SPI MISO

    Decoder output assignment (Y0–Y7, active LOW)
    ──────────────────────────────────────────────
      Phase Shifter n → Y(8 - n)   (n = 1..4)
    """

    # Decoder select-pin indices within self._pins list
    _IDX_C  = 0
    _IDX_B  = 1
    _IDX_A  = 2
    _IDX_G1 = 3

    def __init__(self, cmd_threshold_sec: float = CMD_THRESHOLD_SEC) -> None:
        self._cmd_threshold = cmd_threshold_sec
        self._pins: list = []
        self._spi  = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Initialize FT232H GPIO and SPI bus."""
        os.environ.setdefault("BLINKA_FT232H", "1")

        import board          # deferred: only available with Blinka installed
        import busio
        import digitalio

        self._board      = board
        self._digitalio  = digitalio

        pin_defs = [board.C0, board.C1, board.C2, board.C3]
        self._pins = []
        for pin_def in pin_defs:
            p = digitalio.DigitalInOut(pin_def)
            p.direction = digitalio.Direction.OUTPUT
            self._pins.append(p)

        # Enable the decoder
        self._pins[self._IDX_G1].value = True

        self._spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        log.info("Phased array controller initialized (FT232H ready).")

    def close(self) -> None:
        if self._pins:
            self._disable_decoder()
            for p in self._pins:
                try:
                    p.deinit()
                except Exception:
                    pass
            self._pins = []
        if self._spi is not None:
            try:
                self._spi.deinit()
            except Exception:
                pass
            self._spi = None
        log.info("Phased array controller de-initialized.")

    def __enter__(self) -> "PhasedArrayController":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # ── Private helpers ────────────────────────────────────────────────────────

    def _select_decoder_output(self, y_index: int) -> None:
        """Drive CBA select pins to activate decoder output Y<y_index> (LOW)."""
        if not 0 <= y_index <= 7:
            raise ValueError(f"Decoder output index must be 0–7, got {y_index}")
        bits = format(y_index, "03b")
        self._pins[self._IDX_G1].value = True
        self._pins[self._IDX_C].value  = bits[0] == "1"
        self._pins[self._IDX_B].value  = bits[1] == "1"
        self._pins[self._IDX_A].value  = bits[2] == "1"

    def _disable_decoder(self) -> None:
        """Pull G1 LOW → all Y outputs go HIGH, creating the latch rising edge."""
        if self._pins:
            self._pins[self._IDX_G1].value = False

    @staticmethod
    def _reverse_bits(byte_val: int) -> int:
        """Bit-reverse an 8-bit integer (RFSA3713 attenuator expects LSB-first)."""
        byte_val = ((byte_val & 0xF0) >> 4) | ((byte_val & 0x0F) << 4)
        byte_val = ((byte_val & 0xCC) >> 2) | ((byte_val & 0x33) << 2)
        byte_val = ((byte_val & 0xAA) >> 1) | ((byte_val & 0x55) << 1)
        return byte_val

    # ── Public API ─────────────────────────────────────────────────────────────

    def set_phase(self, shifter_id: int, phase_state: int) -> None:
        """
        Set one phase shifter to the given 6-bit state.

        :param shifter_id:   1-indexed phase shifter number (1–4).
        :param phase_state:  6-bit state value (0–63).
        """
        if not 1 <= shifter_id <= NUM_PHASE_SHIFTERS:
            raise ValueError(f"shifter_id must be 1–{NUM_PHASE_SHIFTERS}, got {shifter_id}")
        if not 0 <= phase_state < PHASE_STATES:
            raise ValueError(f"phase_state must be 0–{PHASE_STATES - 1}, got {phase_state}")

        t_start   = time.monotonic()
        y_index   = 8 - shifter_id          # Decoder output per hardware mapping
        data_bytes = bytearray([phase_state])

        self._select_decoder_output(y_index)
        while not self._spi.try_lock():
            pass
        try:
            self._spi.write(data_bytes)
        finally:
            self._spi.unlock()

        # Rising edge on G1 latches the data into the phase shifter.
        self._disable_decoder()

        # Respect minimum command timing.
        elapsed = time.monotonic() - t_start
        if elapsed < self._cmd_threshold:
            time.sleep(self._cmd_threshold - elapsed)

    def set_phase_vector(self, phases: List[int]) -> None:
        """
        Set all phase shifters from a list of states.

        :param phases: List of NUM_PHASE_SHIFTERS state values (0-indexed by
                       position, so phases[0] → shifter 1).
        """
        if len(phases) != NUM_PHASE_SHIFTERS:
            raise ValueError(
                f"Expected {NUM_PHASE_SHIFTERS} phase values, got {len(phases)}"
            )
        for idx, state in enumerate(phases):
            self.set_phase(idx + 1, state)


# ═══════════════════════════════════════════════════════════════════════════════
# DRY-RUN STUBS  (used when --dry-run flag is set)
# ═══════════════════════════════════════════════════════════════════════════════

class _DryRunSA:
    """Simulates an SA that prefers phases close to a hidden target vector."""

    _TARGET = [24, 8, 48, 16]

    def connect(self)  -> None: log.info("[DRY RUN] SA connected (simulated).")
    def close(self)    -> None: log.info("[DRY RUN] SA closed.")
    def __enter__(self): self.connect(); return self
    def __exit__(self, *_): self.close()

    def measure_peak(self) -> Tuple[float, float]:
        return SA_CENTER_FREQ_HZ, self._current_power

    # Called by dry-run controller to update simulated power
    def _update(self, phases: List[int]) -> None:
        distance = sum(
            min(abs(p - t), PHASE_STATES - abs(p - t)) ** 2
            for p, t in zip(phases, self._TARGET)
        )
        self._current_power = -60.0 + 30.0 * (1 - distance / (4 * 32 ** 2))
        self._current_power += random.gauss(0, 0.2)


class _DryRunArray:
    """Simulates the phased array; delegates power model to the dry-run SA."""

    def __init__(self, dry_sa: _DryRunSA) -> None:
        self._sa = dry_sa
        self._phases = [0] * NUM_PHASE_SHIFTERS

    def connect(self)  -> None: log.info("[DRY RUN] Array controller connected (simulated).")
    def close(self)    -> None: log.info("[DRY RUN] Array controller closed.")
    def __enter__(self): self.connect(); return self
    def __exit__(self, *_): self.close()

    def set_phase(self, shifter_id: int, phase_state: int) -> None:
        self._phases[shifter_id - 1] = phase_state
        self._sa._update(self._phases)

    def set_phase_vector(self, phases: List[int]) -> None:
        self._phases = list(phases)
        self._sa._update(self._phases)


# ═══════════════════════════════════════════════════════════════════════════════
# CSV LOGGER
# ═══════════════════════════════════════════════════════════════════════════════

class MeasurementLogger:
    """Appends each Measurement to a CSV file as it is taken."""

    def __init__(self, filepath: Path) -> None:
        self._filepath = filepath
        self._file   = None
        self._writer = None

    def open(self) -> None:
        self._file   = open(self._filepath, "w", newline="")
        self._writer = csv.writer(self._file)
        header = (
            ["timestamp"]
            + [f"phase_{i+1}" for i in range(NUM_PHASE_SHIFTERS)]
            + ["power_dbm", "freq_hz"]
        )
        self._writer.writerow(header)
        self._file.flush()
        log.info("Logging measurements to: %s", self._filepath)

    def close(self) -> None:
        if self._file:
            self._file.close()

    def __enter__(self) -> "MeasurementLogger":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def record(self, m: Measurement) -> None:
        row = [m.timestamp] + m.phases + [f"{m.power_dbm:.4f}", f"{m.freq_hz:.2f}"]
        self._writer.writerow(row)
        self._file.flush()


# ═══════════════════════════════════════════════════════════════════════════════
# CALIBRATOR
# ═══════════════════════════════════════════════════════════════════════════════

class PhaseCalibrator:
    """
    Implements Two-Phase Coordinate Descent to maximize received power.

    Coordinate descent sweeps each phase shifter independently while the
    others are held fixed, updating each to the best-found state before
    moving to the next.  A full pass over all shifters is one "round".
    Rounds repeat until the global best power stops improving or the round
    limit is hit.

    Phase 1 uses a coarse step size to quickly reach a good neighbourhood.
    Phase 2 then sweeps all 64 states from that starting point.

    Convergence tracking
    ────────────────────
    A global best is maintained across every individual measurement taken
    during the entire run.  At the start of each round, current_phases is
    re-anchored to the global best rather than carried forward from the end
    of the previous round.  This prevents two failure modes seen in the
    naive implementation:

      1. Per-round power (the last shifter's result) is not the round's
         true best — an intermediate shifter sweep may have found a higher
         power that gets silently discarded when the next shifter is swept.

      2. Carrying the end-state of a round forward as the next round's
         starting point means subsequent rounds explore from a potentially
         worse position than the true best already found.

    Convergence is declared when no measurement in an entire round improves
    on the global best, rather than comparing end-of-round to start-of-round.
    """

    def __init__(
        self,
        array:         "PhasedArrayController | _DryRunArray",
        sa:            "SpectrumAnalyzer | _DryRunSA",
        logger:        MeasurementLogger,
        coarse_step:   int = COARSE_STEP,
        max_coarse:    int = MAX_COARSE_ROUNDS,
        max_fine:      int = MAX_FINE_ROUNDS,
    ) -> None:
        self._array       = array
        self._sa          = sa
        self._logger      = logger
        self._coarse_step = coarse_step
        self._max_coarse  = max_coarse
        self._max_fine    = max_fine
        self._total_meas  = 0
        self._global_best: Optional[Measurement] = None  # best across all measurements

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _measure(self, phases: List[int]) -> Measurement:
        """Apply a phase vector, record a Measurement, and update the global best."""
        self._array.set_phase_vector(phases)
        freq_hz, power_dbm = self._sa.measure_peak()
        m = Measurement(phases=list(phases), power_dbm=power_dbm, freq_hz=freq_hz)
        self._logger.record(m)
        self._total_meas += 1
        log.debug("  meas #%d  %s", self._total_meas, m)

        if self._global_best is None or m.power_dbm > self._global_best.power_dbm:
            self._global_best = m
            log.debug("  ★ new global best: %.2f dBm  %s", m.power_dbm, m.phases)

        return m

    def _sweep_one_shifter(
        self,
        current_phases: List[int],
        shifter_idx:    int,
        step:           int,
    ) -> List[int]:
        """
        Sweep a single shifter (0-indexed) through all states at the given
        step size, holding the others fixed at current_phases.

        Updates self._global_best via _measure() for every candidate.
        Returns the phase vector corresponding to the best state found for
        this shifter (which may not be the global best if a prior shifter in
        the same round already holds that title).
        """
        best_power_this_sweep = float("-inf")
        best_phases_this_sweep = list(current_phases)

        for state in range(0, PHASE_STATES, step):
            candidate = list(current_phases)
            candidate[shifter_idx] = state
            m = self._measure(candidate)
            if m.power_dbm > best_power_this_sweep:
                best_power_this_sweep  = m.power_dbm
                best_phases_this_sweep = list(candidate)

        log.info(
            "  Shifter %d best state: %2d  (%.2f dBm)  [global best: %.2f dBm]",
            shifter_idx + 1,
            best_phases_this_sweep[shifter_idx],
            best_power_this_sweep,
            self._global_best.power_dbm,
        )
        return best_phases_this_sweep

    def _coordinate_descent_phase(
        self,
        init_phases: List[int],
        step:        int,
        max_rounds:  int,
        phase_label: str,
    ) -> None:
        """
        Run coordinate descent at a given step size, updating self._global_best
        throughout.  Convergence is declared when a complete round produces no
        improvement to the global best.
        """
        log.info("═" * 60)
        log.info("%s  (step=%d, max_rounds=%d)", phase_label, step, max_rounds)
        log.info("═" * 60)

        # Measure the explicit starting point so it participates in global best.
        self._measure(init_phases)

        for round_num in range(1, max_rounds + 1):
            log.info("── Round %d / %d ──────────────────────────────", round_num, max_rounds)

            # Re-anchor to the global best at the start of every round.
            # This ensures we always explore from the best-known position,
            # not from wherever the previous round happened to finish.
            current_phases = list(self._global_best.phases)
            global_best_at_round_start = self._global_best.power_dbm

            for idx in range(NUM_PHASE_SHIFTERS):
                current_phases = self._sweep_one_shifter(current_phases, idx, step)

            improvement = self._global_best.power_dbm - global_best_at_round_start
            log.info(
                "Round %d complete.  Global best: %.2f dBm  (Δ = %+.3f dBm)",
                round_num, self._global_best.power_dbm, improvement,
            )
            log.info("Global best phases: %s", self._global_best.phases)

            if improvement <= 0.0:
                log.info("%s converged after %d round(s).", phase_label, round_num)
                break

    # ── Public API ─────────────────────────────────────────────────────────────

    def run(self) -> Tuple[Measurement, Measurement]:
        """
        Execute the full two-phase calibration.

        Returns (confirmed_best, global_best) where:
          confirmed_best – a fresh measurement taken at the converged phase
                           vector after the algorithm finishes.
          global_best    – the single highest-power measurement seen at any
                           point during the entire run (may differ from
                           confirmed_best due to SA noise).
        """
        t_start = time.monotonic()

        # ── Phase 1: Coarse coordinate descent ────────────────────────────────
        self._coordinate_descent_phase(
            init_phases = [0] * NUM_PHASE_SHIFTERS,
            step        = self._coarse_step,
            max_rounds  = self._max_coarse,
            phase_label = "Phase 1 – Coarse Sweep",
        )

        log.info("\nCoarse optimum: %s  →  %.2f dBm",
                 self._global_best.phases, self._global_best.power_dbm)

        # ── Phase 2: Fine coordinate descent ──────────────────────────────────
        # Initialize from the coarse global best (already set in self._global_best).
        self._coordinate_descent_phase(
            init_phases = list(self._global_best.phases),
            step        = 1,
            max_rounds  = self._max_fine,
            phase_label = "Phase 2 – Fine Sweep",
        )

        elapsed = time.monotonic() - t_start
        global_best = self._global_best  # snapshot before confirmation measurement

        # ── Confirmation measurement ───────────────────────────────────────────
        # Re-apply the converged phase vector and take one fresh reading.
        # SA noise means this single sample may not equal global_best.power_dbm,
        # so both values are reported.
        log.info("\nApplying converged phase vector for confirmation measurement …")
        confirmed = self._measure(global_best.phases)

        log.info("═" * 60)
        log.info("CALIBRATION COMPLETE")
        log.info("  Converged phase vector : %s", confirmed.phases)
        log.info("  Confirmed power        : %+.2f dBm", confirmed.power_dbm)
        log.info("  Global best power      : %+.2f dBm  (seen at meas. taken during run)",
                 global_best.power_dbm)
        log.info("  Global best phases     : %s", global_best.phases)
        log.info("  Marker frequency       : %.6f GHz", confirmed.freq_hz / 1e9)
        log.info("  Total measurements     : %d", self._total_meas)
        log.info("  Elapsed time           : %.1f s", elapsed)
        log.info("═" * 60)

        return confirmed, global_best


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Phased-array calibration via coordinate-descent power maximization."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Simulate hardware with a synthetic power model (no instruments needed).",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=LOG_DIR,
        help="Directory for the CSV measurement log (default: current directory).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # ── CSV log file ───────────────────────────────────────────────────────────
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    args.log_dir.mkdir(parents=True, exist_ok=True)
    log_path = args.log_dir / f"calibration_{timestamp_str}.csv"

    # ── Instantiate hardware (or dry-run stubs) ────────────────────────────────
    if args.dry_run:
        log.warning("DRY-RUN mode: using simulated hardware.")
        sa    = _DryRunSA()
        array = _DryRunArray(sa)
    else:
        sa    = SpectrumAnalyzer(
            resource_string = SA_RESOURCE_STRING,
            center_freq_hz  = SA_CENTER_FREQ_HZ,
            span_hz         = SA_SPAN_HZ,
            settle_sec      = SA_SETTLE_SEC,
        )
        array = PhasedArrayController(cmd_threshold_sec=CMD_THRESHOLD_SEC)

    # ── Run calibration with automatic cleanup ─────────────────────────────────
    try:
        with sa, array, MeasurementLogger(log_path) as logger:
            calibrator = PhaseCalibrator(
                array       = array,
                sa          = sa,
                logger      = logger,
                coarse_step = COARSE_STEP,
                max_coarse  = MAX_COARSE_ROUNDS,
                max_fine    = MAX_FINE_ROUNDS,
            )
            confirmed, global_best = calibrator.run()

        print("\n" + "=" * 60)
        print("CALIBRATION RESULT – CONFIRMED (fresh measurement)")
        print("=" * 60)
        print(confirmed)
        print()
        print("=" * 60)
        print("CALIBRATION RESULT – GLOBAL BEST (highest seen during run)")
        print("=" * 60)
        print(global_best)
        print(f"\nLog saved to: {log_path}")

    except KeyboardInterrupt:
        log.warning("Calibration interrupted by user.")
        sys.exit(1)
    except Exception as e:
        log.exception("Fatal error during calibration: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    time.sleep(10) # time to run away from measurement before it starts
    main()