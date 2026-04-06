"""
phased_array_calibration.py
─────────────────────────────────────────────────────────────────────────────
Calibrates a 4-element phased array by finding the phase-offset vector that
maximises (beam steering) or minimises (null steering) received signal power,
as measured by an MS2690A spectrum analyzer.

Algorithm: Multi-Start Two-Phase Coordinate Descent
────────────────────────────────────────────────────
Exhaustive search over 64^4 = 16,777,216 states is infeasible at ~200 ms per
measurement (~39 days).  Coordinate descent is used instead, but a single
start from all-zeros can converge to a local optimum rather than the global
one — mutual coupling between array elements means the phase-power landscape
is not perfectly separable and can contain correlated ridges.

Multi-start descent addresses this by running the coarse phase from several
diverse starting vectors and only entering the fine phase from the globally
best result found across all of them:

  Starting vectors
  ────────────────
  1. All-zeros  [0, 0, 0, 0]     – always included; reproducible baseline
  2. NUM_RANDOM_STARTS additional – random coarse-grid vectors drawn uniformly
     random coarse-grid vectors     to probe other basins of the landscape

  Phase 1 – Coarse (step = COARSE_STEP, default 8)
    For each starting vector: sweep each of the 4 shifters through
    {0, 8, 16, …, 56} while holding the others fixed, re-anchoring to the
    global best at the start of each round.  Converges when a full round
    produces no improvement.

  Phase 2 – Fine (step = 1)
    A single descent from the coarse global best, sweeping all 64 states per
    shifter.  Converges when a full round produces no improvement.

Total: ~1 500–3 000 measurements (default 3 random starts) vs. 16 777 216
for exhaustive search.  Every measurement is written to a CSV log.

Hardware dependencies
─────────────────────
  • Adafruit Blinka / FT232H  (phased array SPI + decoder)
  • pyvisa                    (MS2690A spectrum analyzer)

Usage
─────
  Beam steering (maximise power):
      python phased_array_calibration.py --mode max

  Null steering (minimise power):
      python phased_array_calibration.py --mode min

  Dry-run (simulated hardware, no instruments needed):
      python phased_array_calibration.py --mode max --dry-run
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
SA_SPAN_HZ         = 20e3     # Span around center (Hz)
SA_SETTLE_SEC      = 1.5      # Wait time after re-configuring before querying
                               # the marker.  Increase if readings are noisy.

# ── Phased Array (FT232H / SPI) ───────────────────────────────────────────────
NUM_PHASE_SHIFTERS = 4         # Number of phase shifters (1-indexed, 1–4)
PHASE_STATES       = 64        # States per shifter (0–63, 6-bit RFSA device)
CMD_THRESHOLD_SEC  = 45e-9     # Minimum time between SPI commands (45 ns)

# ── Calibration Algorithm ─────────────────────────────────────────────────────
COARSE_STEP        = 8         # Step size for Phase 1 sweep  (must divide 64)
MAX_COARSE_ROUNDS  = 10        # Safety cap on Phase 1 iterations
MAX_FINE_ROUNDS    = 10        # Safety cap on Phase 2 iterations

# Number of random coarse-grid starting vectors explored in addition to the
# fixed all-zeros start.  Higher values improve robustness against local
# optima at the cost of proportionally more measurements.
# 0 → single-start (original behaviour); 3 is a reasonable default.
NUM_RANDOM_STARTS  = 3

# ── Output ────────────────────────────────────────────────────────────────────
LOG_DIR = Path(".")            # Directory for CSV log files


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
        inst.write("TRACe1:STORage:MODE LAV")
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
        time.sleep(0.2) #just in case
        self._inst.write("TRACe1:STORage:MODE LAV")
        time.sleep(self._settle_sec)
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
    Finds the phase vector that maximises or minimises received power using a
    three-stage algorithm designed around the physics of uniform linear arrays.

    Why coordinate descent alone is insufficient for null steering
    ──────────────────────────────────────────────────────────────
    For a ULA the optimal null/beam phase vector is a linear progression across
    elements: [φ₀, φ₀+d, φ₀+2d, φ₀+3d] mod 64, where d is the inter-element
    phase step.  This means the optimum lies on a *diagonal ridge* in the 4-D
    phase space.  Coordinate descent can only move along axis-aligned directions
    (one shifter at a time), so it cannot climb or descend a diagonal ridge
    unless it happens to start very close to it.  Sweeping shifter 2 while
    holding the others at 0 does not reveal that d=16 is optimal — the best
    single-element state depends entirely on what the other elements are doing,
    and that coupling is invisible to per-axis search.

    Three-stage algorithm
    ─────────────────────
    Stage 1  –  Linear Phase Sweep  (64 measurements)
        Exploit the ULA structure directly.  Pin element 0 at state 0 and sweep
        the inter-element step d over all 64 values.  For step d, apply:
            phases[n] = (n * d) % PHASE_STATES
        This exhausts the entire family of linear phase progressions in exactly
        PHASE_STATES measurements — far fewer than any 4-D search — and will
        directly land on [0, 16, 32, 48] for a broadside null (d=16) or the
        beam-steering optimum for any other angle.

        The top-K results (default K=4) are retained as seeds for Stage 2.
        K > 1 hedges against the case where mutual coupling shifts the true
        optimum slightly away from pure linear phase.

    Stage 2  –  Multi-start Coarse Coordinate Descent
        Run coordinate descent (step = COARSE_STEP) from each of the K seeds.
        All starts share a single global_best.  Coordinate descent is now
        appropriate because Stage 1 already placed each start *near* the true
        optimum — the search only needs to correct for the coupling-induced
        deviation from ideal linear phase, which is small and axis-aligned
        corrections can handle.

    Stage 3  –  Fine Coordinate Descent  (step = 1)
        A single descent from the Stage 2 global best.  Converges when a full
        round produces no improvement to global_best.

    Measurement count
    ─────────────────
      Stage 1:  64
      Stage 2:  K × (rounds × N_shifters × 64/COARSE_STEP)  ≈  K × 2 × 4 × 8  = 256 (K=4)
      Stage 3:  rounds × N_shifters × 64                     ≈  2 × 4 × 64     = 512
      Total:    ~832 measurements, independent of NUM_RANDOM_STARTS.

    Convergence tracking
    ────────────────────
    A single global_best is shared across all stages and starts.  Every round
    re-anchors to global_best before sweeping, so later rounds always build
    from the best-known position.  Convergence is declared when a complete
    round fails to improve global_best.
    """

    def __init__(
        self,
        array:             "PhasedArrayController | _DryRunArray",
        sa:                "SpectrumAnalyzer | _DryRunSA",
        logger:            MeasurementLogger,
        maximize:          bool = True,
        coarse_step:       int  = COARSE_STEP,
        max_coarse:        int  = MAX_COARSE_ROUNDS,
        max_fine:          int  = MAX_FINE_ROUNDS,
        num_random_starts: int  = NUM_RANDOM_STARTS,
        linear_sweep_top_k: int = 4,
    ) -> None:
        self._array          = array
        self._sa             = sa
        self._logger         = logger
        self._maximize       = maximize
        self._coarse_step    = coarse_step
        self._max_coarse     = max_coarse
        self._max_fine       = max_fine
        self._n_random       = num_random_starts
        self._top_k          = linear_sweep_top_k
        self._total_meas     = 0
        self._global_best: Optional[Measurement] = None

    # ── Mode helpers ───────────────────────────────────────────────────────────

    def _is_better(self, candidate: float, reference: float) -> bool:
        """Return True if candidate is 'better' than reference given the mode."""
        return candidate > reference if self._maximize else candidate < reference

    def _worst_sentinel(self) -> float:
        """A sentinel value that any real measurement will beat."""
        return float("-inf") if self._maximize else float("+inf")

    # ── Measurement ───────────────────────────────────────────────────────────

    def _measure(self, phases: List[int]) -> Measurement:
        """Apply a phase vector, record a Measurement, and update the global best."""
        self._array.set_phase_vector(phases)
        freq_hz, power_dbm = self._sa.measure_peak()
        m = Measurement(phases=list(phases), power_dbm=power_dbm, freq_hz=freq_hz)
        self._logger.record(m)
        self._total_meas += 1
        log.debug("  meas #%d  %s", self._total_meas, m)

        if self._global_best is None or self._is_better(m.power_dbm, self._global_best.power_dbm):
            self._global_best = m
            log.debug("  ★ new global best: %.2f dBm  %s", m.power_dbm, m.phases)

        return m

    # ── Stage 1: Linear phase sweep ───────────────────────────────────────────

    @staticmethod
    def _linear_phase_vector(step: int) -> List[int]:
        """
        Build the ideal ULA phase vector for inter-element step d.

        Element n receives phase (n * step) % PHASE_STATES, with element 0
        pinned to state 0.  This is the exact closed-form solution for a
        uniform linear array steered to the angle whose sine satisfies
            sin(θ) = step × (λ/d_el) / PHASE_STATES
        where d_el is the element spacing.
        """
        return [(n * step) % PHASE_STATES for n in range(NUM_PHASE_SHIFTERS)]

    def _linear_phase_sweep(self) -> List[Measurement]:
        """
        Stage 1: sweep all PHASE_STATES inter-element step values and return
        all measurements sorted best-first.  This directly enumerates the full
        family of linear phase progressions in PHASE_STATES measurements.
        """
        log.info("═" * 60)
        log.info("Stage 1 – Linear Phase Sweep  (%d measurements)", PHASE_STATES)
        log.info("  Sweeping inter-element step d = 0 … %d", PHASE_STATES - 1)
        log.info("  Vector: [0, d, 2d, 3d] mod %d", PHASE_STATES)
        log.info("═" * 60)

        results: List[Measurement] = []
        for d in range(PHASE_STATES):
            phases = self._linear_phase_vector(d)
            m = self._measure(phases)
            results.append(m)
            log.debug("  d=%2d  %s  →  %.2f dBm", d, phases, m.power_dbm)

        # Sort best-first for logging and seed selection
        results.sort(key=lambda m: m.power_dbm, reverse=self._maximize)

        log.info("Stage 1 complete.  Top results:")
        for i, m in enumerate(results[: self._top_k]):
            d_eff = m.phases[1]   # element 1 state = d directly
            log.info("  #%d  d=%2d  %s  →  %.2f dBm", i + 1, d_eff, m.phases, m.power_dbm)

        return results

    # ── Stage 2 & 3: Coordinate descent ───────────────────────────────────────

    @staticmethod
    def _wrapped_states(center: int, step: int) -> List[int]:
        """
        Generate all PHASE_STATES // step states starting from *center* and
        stepping in increments of *step*, wrapping modulo PHASE_STATES.

        Starting from the current best state makes the phase space circular:
        state 63 and state 0 are always one step apart, so the descent can
        cross the wrap boundary freely.

        Example (PHASE_STATES=64, step=8, center=56):
            [56, 0, 8, 16, 24, 32, 40, 48]   ← wraps at 63→0
        """
        n = PHASE_STATES // step
        return [(center + i * step) % PHASE_STATES for i in range(n)]

    def _sweep_one_shifter(
        self,
        current_phases: List[int],
        shifter_idx:    int,
        step:           int,
    ) -> List[int]:
        """
        Sweep a single shifter through all states at the given step, starting
        from its current state so that the sweep is circular (wraps mod 64).
        Returns the phase vector with the best state found for this shifter.
        """
        best_power_this_sweep = self._worst_sentinel()
        best_phases_this_sweep = list(current_phases)

        for state in self._wrapped_states(current_phases[shifter_idx], step):
            candidate = list(current_phases)
            candidate[shifter_idx] = state
            m = self._measure(candidate)
            if self._is_better(m.power_dbm, best_power_this_sweep):
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

    def _coordinate_descent(
        self,
        init_phases: List[int],
        step:        int,
        max_rounds:  int,
        label:       str,
    ) -> None:
        """
        Run coordinate descent from init_phases at the given step size.

        Shares self._global_best with all other descents so prior stages
        immediately benefit later ones.  Re-anchors to global_best at the
        start of every round.  Converges when a full round produces no
        improvement.
        """
        log.info("  [%s] start=%s  step=%d", label, init_phases, step)
        self._measure(init_phases)

        for round_num in range(1, max_rounds + 1):
            current_phases      = list(self._global_best.phases)
            best_at_round_start = self._global_best.power_dbm

            for idx in range(NUM_PHASE_SHIFTERS):
                current_phases = self._sweep_one_shifter(current_phases, idx, step)

            improvement = (
                self._global_best.power_dbm - best_at_round_start
                if self._maximize
                else best_at_round_start - self._global_best.power_dbm
            )
            log.info(
                "  [%s] round %d  global best: %.2f dBm  (Δ = %+.3f dBm)",
                label, round_num, self._global_best.power_dbm, improvement,
            )
            if improvement <= 0.0:
                log.info("  [%s] converged after %d round(s).", label, round_num)
                break

    # ── Public API ─────────────────────────────────────────────────────────────

    def run(self) -> Tuple[Measurement, Measurement]:
        """
        Execute three-stage calibration and return (confirmed_best, global_best).

          confirmed_best – a fresh measurement at the converged phase vector.
          global_best    – the single best measurement seen at any point during
                           the run (may differ from confirmed_best due to SA
                           noise; use whichever is more appropriate).
        """
        mode_str = "MAX" if self._maximize else "MIN"
        t_start  = time.monotonic()

        # ── Stage 1: Linear phase sweep ───────────────────────────────────────
        linear_results = self._linear_phase_sweep()

        # Top-K seeds for Stage 2 (best-first already sorted)
        seeds = [m.phases for m in linear_results[: self._top_k]]

        # Optionally append random coarse-grid seeds for extra coverage.
        # These are less useful here than they were in the old algorithm because
        # Stage 1 already covers the most important region of the space, but
        # they still help when coupling is severe.
        if self._n_random > 0:
            coarse_states = list(range(0, PHASE_STATES, self._coarse_step))
            for _ in range(self._n_random):
                rv = [random.choice(coarse_states) for _ in range(NUM_PHASE_SHIFTERS)]
                if rv not in seeds:
                    seeds.append(rv)

        log.info("═" * 60)
        log.info("Stage 2 – Coarse Coordinate Descent  "
                 "[mode=%s  step=%d  seeds=%d  max_rounds=%d]",
                 mode_str, self._coarse_step, len(seeds), self._max_coarse)
        log.info("═" * 60)

        # ── Stage 2: Coarse descent from each seed ────────────────────────────
        for i, seed in enumerate(seeds):
            log.info("── Seed %d / %d ───────────────────────────────────────",
                     i + 1, len(seeds))
            self._coordinate_descent(
                seed, step=self._coarse_step, max_rounds=self._max_coarse,
                label=f"coarse-{i+1}",
            )

        log.info("\nCoarse optimum: %s  →  %.2f dBm",
                 self._global_best.phases, self._global_best.power_dbm)

        # ── Stage 3: Fine descent from the coarse global best ─────────────────
        log.info("═" * 60)
        log.info("Stage 3 – Fine Coordinate Descent  [mode=%s  step=1  max_rounds=%d]",
                 mode_str, self._max_fine)
        log.info("═" * 60)

        self._coordinate_descent(
            list(self._global_best.phases), step=1, max_rounds=self._max_fine,
            label="fine",
        )

        elapsed     = time.monotonic() - t_start
        global_best = self._global_best   # snapshot before confirmation

        # ── Confirmation measurement ───────────────────────────────────────────
        log.info("\nApplying converged phase vector for confirmation measurement …")
        confirmed = self._measure(global_best.phases)

        log.info("═" * 60)
        log.info("CALIBRATION COMPLETE  [mode=%s]", mode_str)
        log.info("  Converged phase vector : %s", confirmed.phases)
        log.info("  Confirmed power        : %+.2f dBm", confirmed.power_dbm)
        log.info("  Global best power      : %+.2f dBm  (best seen during run)",
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
        description=(
            "Phased-array calibration via multi-start coordinate-descent. "
            "Finds the phase vector that maximises (beam) or minimises (null) "
            "received power."
        )
    )
    parser.add_argument(
        "--mode", choices=["max", "min"], default="max",
        help=(
            "'max' to maximise received power (beam steering, default). "
            "'min' to minimise received power (null steering)."
        ),
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
    parser.add_argument(
        "--random-starts", type=int, default=NUM_RANDOM_STARTS,
        help=(
            f"Number of random coarse-grid seeds added after the top-K linear "
            f"sweep results (default: {NUM_RANDOM_STARTS}). Set to 0 to rely "
            "entirely on the linear sweep seeds."
        ),
    )
    parser.add_argument(
        "--top-k", type=int, default=4,
        help=(
            "Number of top linear-sweep results to use as Stage 2 seeds "
            "(default: 4). Higher values increase robustness when coupling "
            "shifts the optimum away from pure linear phase."
        ),
    )
    parser.add_argument(
        "--coarse-step", type=int, default=COARSE_STEP,
        help=f"Coarse sweep step size (default: {COARSE_STEP}, must divide {PHASE_STATES}).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.coarse_step <= 0 or PHASE_STATES % args.coarse_step != 0:
        log.error("--coarse-step must be a positive divisor of %d.", PHASE_STATES)
        sys.exit(1)

    maximize = (args.mode == "max")

    # ── CSV log file ───────────────────────────────────────────────────────────
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    args.log_dir.mkdir(parents=True, exist_ok=True)
    log_path = args.log_dir / f"calibration_{args.mode}_{timestamp_str}.csv"

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
                array                = array,
                sa                   = sa,
                logger               = logger,
                maximize             = maximize,
                coarse_step          = args.coarse_step,
                max_coarse           = MAX_COARSE_ROUNDS,
                max_fine             = MAX_FINE_ROUNDS,
                num_random_starts    = args.random_starts,
                linear_sweep_top_k   = args.top_k,
            )
            confirmed, global_best = calibrator.run()

        mode_label = "MAXIMUM (beam steering)" if maximize else "MINIMUM (null steering)"
        print("\n" + "=" * 60)
        print(f"CALIBRATION RESULT – {mode_label}")
        print("=" * 60)
        print(f"Confirmed : {confirmed}")
        print(f"Global best: {global_best}")
        print(f"\nLog saved to: {log_path}")

    except KeyboardInterrupt:
        log.warning("Calibration interrupted by user.")
        sys.exit(1)
    except Exception as e:
        log.exception("Fatal error during calibration: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    time.sleep(5)
    main()