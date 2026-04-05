"""
beam_steering_sweep.py
─────────────────────────────────────────────────────────────────────────────
Sweeps a turntable through a range of azimuth angles and, at each angle,
finds the 4-element phased-array phase vector that maximises (beam steering)
or minimises (null steering) the received signal power.

The result is a steering lookup table CSV: one row per angle containing the
optimal phase vector and the achieved power.  This table can be loaded
directly into a beam/null steering controller.

Why multi-start coordinate descent?
─────────────────────────────────────
The phase-power landscape in a real phased array is not smoothly separable.
Mutual coupling between elements means the optimal state of each shifter
depends on what the others are doing; the surface has correlated ridges and
can contain multiple local optima.  A single-start coordinate descent
reliably finds *a* local optimum but not necessarily the global one.

Multi-start descent addresses this by running the coarse phase from several
diverse starting vectors and promoting only the globally best result into
the fine phase:

  Starting vectors used per angle
  ────────────────────────────────
  1. All-zeros  [0, 0, 0, 0]      — always included; reproducible baseline
  2. Warm start from previous      — adjacent angles share similar optima;
     angle's confirmed best          re-using the last result often converges
                                     in 1 round and costs very few measurements
  3. NUM_RANDOM_STARTS uniform     — random coarse-grid vectors to probe
     random coarse-grid vectors      other basins of the landscape

  The warm-start chain means the sweep self-improves: each angle seeds the
  next.  For beam steering the optimal vector tends to evolve smoothly with
  angle, so the warm start is typically near-optimal and the random starts
  serve as a safety net.

Calibration measurement protocol
──────────────────────────────────
Each individual SA measurement resets the Max Hold trace (as confirmed
working in your modified script) and dwells for CALIB_HOLD_SEC before
querying the peak marker.  The pattern measurement at the end of each angle
uses the longer PATTERN_HOLD_SEC for a clean final reading.

Dependencies
────────────
  azimuth_pattern_sweep.py  –  Turntable, SpectrumAnalyzer,
                                _DryRunTurntable  (place in same directory)
  hardware_control.py       –  HardwareControl   (place in same directory)

Usage
─────
  Beam steering (maximise power):
      python beam_steering_sweep.py --mode max

  Null steering (minimise power):
      python beam_steering_sweep.py --mode min

  Dry-run (no hardware needed, simulates all instruments):
      python beam_steering_sweep.py --mode max --dry-run

  All options:
      python beam_steering_sweep.py --help
"""

from __future__ import annotations

import argparse
import csv
import logging
import math
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

# ── Turntable ─────────────────────────────────────────────────────────────────
TURNTABLE_IP          = "172.16.1.168"
TURNTABLE_PORT        = 200
TURNTABLE_CMD_DELAY   = 0.15
TURNTABLE_SPEED_DEG_S = 2.0
TURNTABLE_ACCEL_DEG_S2 = 4.0

# ── Azimuth sweep range ───────────────────────────────────────────────────────
SWEEP_START_DEG = -50.0    # Start angle (°)
SWEEP_STOP_DEG  =  50.0    # Stop angle  (°)
SWEEP_STEP_DEG  =  10.0    # Angular step between calibration points (°)

# ── Spectrum Analyzer ──────────────────────────────────────────────────────────
SA_RESOURCE_STRING = 'USB0::2907::6::6261932852::0::INSTR'

# ── Phased Array (FT232H) ─────────────────────────────────────────────────────
NUM_PHASE_SHIFTERS = 4
PHASE_STATES       = 64    # 0–63, 6-bit RFSA device

# ── Calibration algorithm ─────────────────────────────────────────────────────
# SA dwell time per individual measurement during phase optimisation.
# Shorter than pattern hold time to keep calibration fast; increase if noisy.
CALIB_HOLD_SEC = 0.3

# SA dwell time for the final confirmed pattern reading after calibration.
PATTERN_HOLD_SEC = 2.0

# Step size for the coarse phase of coordinate descent (must divide 64).
COARSE_STEP = 8

# Number of random starting vectors to add per angle (in addition to
# the all-zeros and warm-start vectors).  Higher = more robust but slower.
# A value of 3 gives 5 total starts, exploring the space reasonably well
# without multiplying run time dramatically.
NUM_RANDOM_STARTS = 3

# Round limits per start.  Coarse rounds usually converge in 2–4; fine in 2–3.
MAX_COARSE_ROUNDS = 8
MAX_FINE_ROUNDS   = 8

# ── Output ────────────────────────────────────────────────────────────────────
LOG_DIR = Path("./steering_tables/")


# ═══════════════════════════════════════════════════════════════════════════════
# LOGGING SETUP
# ═══════════════════════════════════════════════════════════════════════════════

logging.basicConfig(
    level   = logging.INFO,
    format  = "%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt = "%H:%M:%S",
)
log = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════════════════
# DATA STRUCTURES
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class SteeringEntry:
    """One row of the steering lookup table: angle → optimal phase vector → power."""
    angle_deg:   float
    phases:      List[int]   # length = NUM_PHASE_SHIFTERS
    power_dbm:   float
    freq_hz:     float
    mode:        str         # 'max' or 'min'
    num_starts:  int         # how many starting vectors were tried
    timestamp:   str = field(default_factory=lambda: datetime.now().isoformat())

    def __str__(self) -> str:
        pv = ", ".join(f"P{i+1}={v:2d}" for i, v in enumerate(self.phases))
        return (f"  {self.angle_deg:+7.2f}°   [{pv}]   "
                f"{self.power_dbm:+.2f} dBm  ({self.mode})")


@dataclass
class _CalibMeasurement:
    """A single power measurement during phase calibration."""
    phases:    List[int]
    power_dbm: float
    freq_hz:   float


# ═══════════════════════════════════════════════════════════════════════════════
# STEERING TABLE LOGGER
# ═══════════════════════════════════════════════════════════════════════════════

class SteeringTableLogger:
    """
    Writes each SteeringEntry to a CSV immediately after it is confirmed.

    The CSV is flush-safe: a partial run is still a valid lookup table for
    the angles that were completed.

    Columns
    ───────
    timestamp, angle_deg, phase_1 … phase_N, power_dbm, freq_hz, mode,
    num_starts
    """

    def __init__(self, filepath: Path) -> None:
        self._filepath = filepath
        self._file     = None
        self._writer   = None

    def open(self) -> None:
        self._file   = open(self._filepath, "w", newline="")
        self._writer = csv.writer(self._file)
        header = (
            ["timestamp", "angle_deg"]
            + [f"phase_{i+1}" for i in range(NUM_PHASE_SHIFTERS)]
            + ["power_dbm", "freq_hz", "mode", "num_starts"]
        )
        self._writer.writerow(header)
        self._file.flush()
        log.info("Steering table → %s", self._filepath)

    def close(self) -> None:
        if self._file:
            self._file.close()

    def __enter__(self) -> "SteeringTableLogger":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def record(self, entry: SteeringEntry) -> None:
        row = (
            [entry.timestamp, f"{entry.angle_deg:.4f}"]
            + entry.phases
            + [f"{entry.power_dbm:.4f}", f"{entry.freq_hz:.2f}",
               entry.mode, entry.num_starts]
        )
        self._writer.writerow(row)
        self._file.flush()


# ═══════════════════════════════════════════════════════════════════════════════
# PHASED ARRAY ADAPTER
# Wraps hardware_control.HardwareControl with a context-manager interface
# and the set_phase_vector() method expected by PhaseCalibrator.
# ═══════════════════════════════════════════════════════════════════════════════

class PhasedArrayAdapter:
    """
    Thin adapter around HardwareControl that adds:
      • Context-manager lifecycle (connect / close)
      • set_phase_vector(phases) convenience method
    """

    def __init__(self, serial_number: Optional[str] = None) -> None:
        self._serial = serial_number
        self._hw: Optional[object] = None

    def connect(self) -> None:
        from hardware_control import HardwareControl  # deferred: requires Blinka
        self._hw = HardwareControl(serial_number=self._serial)
        if self._hw.spi is None:
            raise RuntimeError(
                "HardwareControl failed to initialise FT232H. "
                "Check USB connection and Blinka install."
            )
        log.info("Phased array controller connected (FT232H).")

    def close(self) -> None:
        if self._hw is not None:
            self._hw.deinitialize_hardware()
            self._hw = None

    def __enter__(self) -> "PhasedArrayAdapter":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def set_phase(self, shifter_id: int, phase_state: int) -> None:
        """1-indexed shifter; state 0–63."""
        self._hw.set_phase(shifter_id, phase_state)

    def set_phase_vector(self, phases: List[int]) -> None:
        """Apply all phase states at once (0-indexed list → shifter 1..N)."""
        if len(phases) != NUM_PHASE_SHIFTERS:
            raise ValueError(
                f"Expected {NUM_PHASE_SHIFTERS} phase values, got {len(phases)}"
            )
        for idx, state in enumerate(phases):
            self._hw.set_phase(idx + 1, state)


# ═══════════════════════════════════════════════════════════════════════════════
# SPECTRUM ANALYZER CALIBRATION ADAPTER
# The azimuth_pattern_sweep SA exposes measure_peak_after_hold(hold_sec).
# PhaseCalibrator expects a simpler measure_peak() → (freq_hz, power_dbm).
# This adapter bridges the two.
# ═══════════════════════════════════════════════════════════════════════════════

class _CalibSAAdapter:
    """
    Wraps a SpectrumAnalyzer (from azimuth_pattern_sweep) and exposes a
    single measure_peak() method that resets Max Hold, dwells for
    calib_hold_sec, and returns (freq_hz, power_dbm).
    """

    def __init__(self, sa: object, calib_hold_sec: float) -> None:
        self._sa   = sa
        self._hold = calib_hold_sec

    def measure_peak(self) -> Tuple[float, float]:
        return self._sa.measure_peak_after_hold(self._hold)


# ═══════════════════════════════════════════════════════════════════════════════
# PHASE CALIBRATOR  (Multi-Start Coordinate Descent)
# ═══════════════════════════════════════════════════════════════════════════════

class PhaseCalibrator:
    """
    Finds the phase vector that maximises or minimises received power using
    multi-start coordinate descent.

    The calibrator is stateless between angles: call run() once per angle,
    passing an optional warm_start vector seeded from the previous angle's
    result.

    Parameters
    ──────────
    array          PhasedArrayAdapter (or dry-run stub) — must implement
                   set_phase_vector(phases).
    calib_sa       _CalibSAAdapter (or dry-run stub) — must implement
                   measure_peak() → (freq_hz, power_dbm).
    maximize       True  → maximise power (beam steering).
                   False → minimise power (null steering).
    coarse_step    Step size for the coarse descent phase.
    num_random_starts
                   Number of random coarse-grid starting vectors to explore
                   in addition to all-zeros and the warm start.
    """

    def __init__(
        self,
        array:             "PhasedArrayAdapter | _DryRunPhasedArray",
        calib_sa:          "_CalibSAAdapter | _DryRunCalibSA",
        maximize:          bool = True,
        coarse_step:       int  = COARSE_STEP,
        num_random_starts: int  = NUM_RANDOM_STARTS,
        max_coarse_rounds: int  = MAX_COARSE_ROUNDS,
        max_fine_rounds:   int  = MAX_FINE_ROUNDS,
    ) -> None:
        self._array        = array
        self._sa           = calib_sa
        self._maximize     = maximize
        self._coarse_step  = coarse_step
        self._n_random     = num_random_starts
        self._max_coarse   = max_coarse_rounds
        self._max_fine     = max_fine_rounds
        self._total_meas   = 0
        self._global_best: Optional[_CalibMeasurement] = None

    # ── Score helpers ──────────────────────────────────────────────────────────

    def _is_better(self, candidate_power: float, reference_power: float) -> bool:
        """Return True if candidate_power is 'better' given the current mode."""
        return (candidate_power > reference_power if self._maximize
                else candidate_power < reference_power)

    def _worst_power(self) -> float:
        return float("-inf") if self._maximize else float("+inf")

    # ── Measurement ───────────────────────────────────────────────────────────

    def _measure(self, phases: List[int]) -> _CalibMeasurement:
        """Apply a phase vector, take a measurement, and update global best."""
        self._array.set_phase_vector(phases)
        freq_hz, power_dbm = self._sa.measure_peak()
        m = _CalibMeasurement(phases=list(phases), power_dbm=power_dbm, freq_hz=freq_hz)
        self._total_meas += 1

        if (self._global_best is None
                or self._is_better(m.power_dbm, self._global_best.power_dbm)):
            self._global_best = m
            log.debug("  ★ new global best: %.2f dBm  %s", m.power_dbm, m.phases)

        return m

    # ── Single-start coordinate descent ───────────────────────────────────────

    def _sweep_one_shifter(
        self,
        current_phases: List[int],
        shifter_idx:    int,
        step:           int,
    ) -> List[int]:
        """
        Sweep one shifter through all states at *step* while holding the
        others fixed.  Updates self._global_best via _measure().
        Returns the best phase vector found for this shifter sweep.
        """
        best_power = self._worst_power()
        best_phases = list(current_phases)

        for state in range(0, PHASE_STATES, step):
            candidate = list(current_phases)
            candidate[shifter_idx] = state
            m = self._measure(candidate)
            if self._is_better(m.power_dbm, best_power):
                best_power  = m.power_dbm
                best_phases = list(candidate)

        log.debug(
            "    Shifter %d → state %2d  (%.2f dBm)",
            shifter_idx + 1, best_phases[shifter_idx], best_power,
        )
        return best_phases

    def _coordinate_descent(
        self,
        init_phases: List[int],
        step:        int,
        max_rounds:  int,
    ) -> _CalibMeasurement:
        """
        Run coordinate descent from *init_phases* at the given step.

        Re-anchors to self._global_best at the start of each round so
        subsequent rounds always build from the best-known foundation.
        Converges when a complete round fails to improve the global best.

        Returns the global best _CalibMeasurement seen during this descent
        (may have been set on a prior start — the caller should compare).
        """
        # Snapshot before this descent starts so we can detect improvement.
        best_before_start = (self._global_best.power_dbm
                             if self._global_best else self._worst_power())

        self._measure(init_phases)   # seed with the start itself

        for round_num in range(1, max_rounds + 1):
            current_phases = list(self._global_best.phases)
            best_at_round_start = self._global_best.power_dbm

            for idx in range(NUM_PHASE_SHIFTERS):
                current_phases = self._sweep_one_shifter(current_phases, idx, step)

            improvement = (self._global_best.power_dbm - best_at_round_start
                           if self._maximize
                           else best_at_round_start - self._global_best.power_dbm)

            log.debug(
                "    round %d  global best: %.2f dBm  (Δ = %+.3f dBm)",
                round_num, self._global_best.power_dbm, improvement,
            )

            if improvement <= 0.0:
                break   # converged

        return self._global_best

    # ── Public API ─────────────────────────────────────────────────────────────

    def run(
        self,
        warm_start: Optional[List[int]] = None,
    ) -> Tuple[_CalibMeasurement, int]:
        """
        Run multi-start calibration and return (best_measurement, num_starts).

        Parameters
        ──────────
        warm_start   Optional phase vector from the previous angle.  When
                     supplied it is included as one of the starting points.
                     Pass None for the first angle.

        Returns
        ───────
        (best_measurement, num_starts_used)
        """
        self._total_meas  = 0
        self._global_best = None

        # ── Build the list of starting vectors ────────────────────────────────
        starts: List[List[int]] = [[0] * NUM_PHASE_SHIFTERS]   # always first

        if warm_start is not None and warm_start != starts[0]:
            starts.append(list(warm_start))

        # Random starts: sample uniformly from the coarse grid
        coarse_states = list(range(0, PHASE_STATES, self._coarse_step))
        for _ in range(self._n_random):
            rv = [random.choice(coarse_states) for _ in range(NUM_PHASE_SHIFTERS)]
            if rv not in starts:
                starts.append(rv)

        n_starts = len(starts)
        mode_str = "MAX" if self._maximize else "MIN"
        log.info("  Calibrating with %d starting vectors  [mode=%s]", n_starts, mode_str)

        # ── Phase 1: Coarse descent from every starting vector ────────────────
        for i, sv in enumerate(starts):
            log.debug("  Start %d/%d: %s", i + 1, n_starts, sv)
            self._coordinate_descent(sv, step=self._coarse_step, max_rounds=self._max_coarse)

        log.info(
            "  Coarse complete  →  best so far: %.2f dBm  %s",
            self._global_best.power_dbm, self._global_best.phases,
        )

        # ── Phase 2: Fine descent from the coarse global best ─────────────────
        self._coordinate_descent(
            list(self._global_best.phases), step=1, max_rounds=self._max_fine
        )

        log.info(
            "  Fine complete   →  best: %.2f dBm  %s  (%d meas.)",
            self._global_best.power_dbm, self._global_best.phases, self._total_meas,
        )

        return self._global_best, n_starts


# ═══════════════════════════════════════════════════════════════════════════════
# DRY-RUN STUBS
# ═══════════════════════════════════════════════════════════════════════════════

class _DryRunPhasedArray:
    """Simulates a phased array; exposes set_phase_vector() for calibrator."""

    def __init__(self) -> None:
        self._phases = [0] * NUM_PHASE_SHIFTERS
        self._calib_sa: Optional["_DryRunCalibSA"] = None  # injected after construction

    def connect(self)  -> None: log.info("[DRY RUN] Phased array connected (simulated).")
    def close(self)    -> None: log.info("[DRY RUN] Phased array closed.")
    def __enter__(self): self.connect(); return self
    def __exit__(self, *_): self.close()

    def set_phase_vector(self, phases: List[int]) -> None:
        self._phases = list(phases)
        if self._calib_sa is not None:
            self._calib_sa._update_phases(phases)


class _DryRunCalibSA:
    """
    Simulated SA that models received power as a function of both azimuth
    angle and the current phase vector.

    Power model
    ───────────
    We simulate a 4-element ULA with λ/2 spacing.  For each element n the
    expected phase to steer to angle θ is:

        φ_n = n × π × sin(θ_rad)   (mod 2π, mapped to 0–63 states)

    Power is proportional to the array factor magnitude squared, with added
    Gaussian noise.
    """

    def __init__(self) -> None:
        self._angle_deg = 0.0
        self._phases    = [0] * NUM_PHASE_SHIFTERS

    def _update_angle(self, angle_deg: float) -> None:
        self._angle_deg = angle_deg

    def _update_phases(self, phases: List[int]) -> None:
        self._phases = list(phases)

    def measure_peak(self) -> Tuple[float, float]:
        theta_rad = math.radians(self._angle_deg)
        # Ideal phase for element n steering to θ: φ_n = n·π·sin(θ) (radians)
        # Quantised to 0–63 (one state ≈ 2π/64 radians)
        af_real = 0.0
        af_imag = 0.0
        for n, state in enumerate(self._phases):
            ideal_phase_rad = n * math.pi * math.sin(theta_rad)
            applied_phase_rad = (state / PHASE_STATES) * 2.0 * math.pi
            phase_error = applied_phase_rad - ideal_phase_rad
            af_real += math.cos(phase_error)
            af_imag += math.sin(phase_error)

        af_magnitude_sq = (af_real ** 2 + af_imag ** 2) / (NUM_PHASE_SHIFTERS ** 2)
        power_dbm = -20.0 + 20.0 * math.log10(max(af_magnitude_sq, 1e-10) ** 0.5 + 1e-10)
        power_dbm += random.gauss(0, 0.3)
        return 2.345e9, power_dbm


class _DryRunPatternSA:
    """
    Adapts _DryRunCalibSA to the measure_peak_after_hold() interface
    expected by azimuth_pattern_sweep code (for the final pattern reading).
    """

    def __init__(self, calib_sa: _DryRunCalibSA) -> None:
        self._sa = calib_sa

    def connect(self)  -> None: pass
    def close(self)    -> None: pass
    def __enter__(self): return self
    def __exit__(self, *_): pass

    def reset_max_hold(self) -> None: pass

    def measure_peak_after_hold(self, hold_sec: float) -> Tuple[float, float]:
        time.sleep(hold_sec * 0.02)   # minimal delay in dry-run
        return self._sa.measure_peak()


# ═══════════════════════════════════════════════════════════════════════════════
# BEAM STEERING SWEEP ENGINE
# ═══════════════════════════════════════════════════════════════════════════════

class BeamSteeringSweep:
    """
    Orchestrates the full sweep:
      For each azimuth angle →
        1. Move turntable to angle.
        2. Run multi-start phase calibration (max or min power).
        3. Apply the confirmed phase vector.
        4. Take a final clean pattern measurement (longer hold).
        5. Log the SteeringEntry to the lookup table CSV.
        6. Print the result immediately.

    The warm_start vector is threaded from one angle to the next so
    each angle seeds its neighbour.
    """

    def __init__(
        self,
        turntable:  object,   # Turntable | _DryRunTurntable
        sa:         object,   # SpectrumAnalyzer | _DryRunPatternSA
        array:      object,   # PhasedArrayAdapter | _DryRunPhasedArray
        logger:     SteeringTableLogger,
        mode:       str   = "max",          # 'max' or 'min'
        start_deg:  float = SWEEP_START_DEG,
        stop_deg:   float = SWEEP_STOP_DEG,
        step_deg:   float = SWEEP_STEP_DEG,
        calib_hold: float = CALIB_HOLD_SEC,
        pat_hold:   float = PATTERN_HOLD_SEC,
        coarse_step: int  = COARSE_STEP,
        num_random_starts: int = NUM_RANDOM_STARTS,
    ) -> None:
        if mode not in ("max", "min"):
            raise ValueError(f"mode must be 'max' or 'min', got '{mode}'")

        self._tt           = turntable
        self._sa           = sa
        self._array        = array
        self._logger       = logger
        self._mode         = mode
        self._maximize     = (mode == "max")
        self._start        = start_deg
        self._stop         = stop_deg
        self._step         = step_deg
        self._calib_hold   = calib_hold
        self._pat_hold     = pat_hold
        self._coarse_step  = coarse_step
        self._n_random     = num_random_starts

    def _build_angles(self) -> List[float]:
        angles = []
        ang = self._start
        while ang <= self._stop + 1e-9:
            angles.append(round(ang, 6))
            ang += self._step
        return angles

    def run(self) -> List[SteeringEntry]:
        """
        Execute the full azimuth sweep.
        Returns the list of SteeringEntry objects (complete or partial).
        """
        angles  = self._build_angles()
        n_pts   = len(angles)
        results: List[SteeringEntry] = []
        warm_start: Optional[List[int]] = None   # seeded from previous angle

        log.info("═" * 60)
        log.info("BEAM STEERING SWEEP  mode=%s  start=%.1f°  stop=%.1f°  "
                 "step=%.2f°  points=%d",
                 self._mode.upper(), self._start, self._stop, self._step, n_pts)
        log.info("Multi-start: zeros + warm_start + %d random  |  "
                 "coarse_step=%d", self._n_random, self._coarse_step)
        log.info("═" * 60)

        t_sweep_start = time.monotonic()

        # Create the calibration SA adapter once for all angles
        calib_sa = _CalibSAAdapter(self._sa, self._calib_hold)

        for i, angle in enumerate(angles):
            log.info("─" * 60)
            log.info("Angle %d / %d   target = %+.2f°", i + 1, n_pts, angle)

            # ── 1. Move turntable ──────────────────────────────────────────────
            self._tt.goto(angle)

            # Inform dry-run SA of current angle (no-op on real hardware SA)
            if hasattr(self._sa, "_sa") and hasattr(self._sa._sa, "_update_angle"):
                self._sa._sa._update_angle(angle)
            elif hasattr(self._sa, "set_angle"):
                self._sa.set_angle(angle)

            # ── 2. Phase calibration ───────────────────────────────────────────
            calibrator = PhaseCalibrator(
                array             = self._array,
                calib_sa          = calib_sa,
                maximize          = self._maximize,
                coarse_step       = self._coarse_step,
                num_random_starts = self._n_random,
                max_coarse_rounds = MAX_COARSE_ROUNDS,
                max_fine_rounds   = MAX_FINE_ROUNDS,
            )
            best_calib, n_starts = calibrator.run(warm_start=warm_start)

            # ── 3. Apply the confirmed phase vector ────────────────────────────
            self._array.set_phase_vector(best_calib.phases)

            # ── 4. Final pattern measurement (clean, longer dwell) ─────────────
            log.info("  Taking final pattern measurement (%.1f s hold) …",
                     self._pat_hold)
            freq_hz, power_dbm = self._sa.measure_peak_after_hold(self._pat_hold)

            # ── 5. Record ──────────────────────────────────────────────────────
            entry = SteeringEntry(
                angle_deg  = angle,
                phases     = list(best_calib.phases),
                power_dbm  = power_dbm,
                freq_hz    = freq_hz,
                mode       = self._mode,
                num_starts = n_starts,
            )
            self._logger.record(entry)
            results.append(entry)

            # ── 6. Print immediately ───────────────────────────────────────────
            print(entry)

            # Thread the warm start to the next angle
            warm_start = list(best_calib.phases)

        elapsed = time.monotonic() - t_sweep_start
        log.info("═" * 60)
        log.info("SWEEP COMPLETE  –  %d angles  in  %.1f s (%.1f min)",
                 n_pts, elapsed, elapsed / 60.0)

        if results:
            best = (max(results, key=lambda e: e.power_dbm) if self._maximize
                    else min(results, key=lambda e: e.power_dbm))
            log.info("Best angle: %+.2f°  →  %.2f dBm  phases=%s",
                     best.angle_deg, best.power_dbm, best.phases)

        log.info("═" * 60)
        return results


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Azimuth beam/null steering sweep: "
            "finds the optimal phase vector at each angle and saves a lookup table."
        )
    )
    parser.add_argument(
        "--mode", choices=["max", "min"], required=True,
        help="'max' for beam steering (maximise power), 'min' for null steering.",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Simulate all instruments (no hardware required).",
    )
    parser.add_argument(
        "--log-dir", type=Path, default=LOG_DIR,
        help=f"Output directory for the lookup table CSV (default: {LOG_DIR}).",
    )
    parser.add_argument(
        "--start", type=float, default=SWEEP_START_DEG,
        help=f"Sweep start angle in degrees (default: {SWEEP_START_DEG}).",
    )
    parser.add_argument(
        "--stop", type=float, default=SWEEP_STOP_DEG,
        help=f"Sweep stop angle in degrees (default: {SWEEP_STOP_DEG}).",
    )
    parser.add_argument(
        "--step", type=float, default=SWEEP_STEP_DEG,
        help=f"Angular step in degrees (default: {SWEEP_STEP_DEG}).",
    )
    parser.add_argument(
        "--calib-hold", type=float, default=CALIB_HOLD_SEC,
        help=f"SA dwell per calibration measurement, seconds (default: {CALIB_HOLD_SEC}).",
    )
    parser.add_argument(
        "--pattern-hold", type=float, default=PATTERN_HOLD_SEC,
        help=f"SA dwell for the final pattern reading, seconds (default: {PATTERN_HOLD_SEC}).",
    )
    parser.add_argument(
        "--random-starts", type=int, default=NUM_RANDOM_STARTS,
        help=f"Number of random starting vectors per angle (default: {NUM_RANDOM_STARTS}).",
    )
    parser.add_argument(
        "--coarse-step", type=int, default=COARSE_STEP,
        help=f"Coarse descent step size (default: {COARSE_STEP}, must divide 64).",
    )
    parser.add_argument(
        "--speed", type=float, default=TURNTABLE_SPEED_DEG_S,
        help=f"Turntable speed in °/s (default: {TURNTABLE_SPEED_DEG_S}).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.coarse_step <= 0 or PHASE_STATES % args.coarse_step != 0:
        log.error("--coarse-step must be a positive divisor of %d.", PHASE_STATES)
        sys.exit(1)

    # ── Output file ────────────────────────────────────────────────────────────
    args.log_dir.mkdir(parents=True, exist_ok=True)
    ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.log_dir / f"steering_table_{args.mode}_{ts}.csv"

    # ── Instrument setup ───────────────────────────────────────────────────────
    if args.dry_run:
        log.warning("DRY-RUN mode: all instruments simulated.")

        from azimuth_pattern_sweep import _DryRunTurntable

        dry_calib_sa  = _DryRunCalibSA()
        dry_array     = _DryRunPhasedArray()
        dry_array._calib_sa = dry_calib_sa   # wire up so array updates propagate
        dry_pattern_sa = _DryRunPatternSA(dry_calib_sa)

        turntable = _DryRunTurntable(speed_deg_s=args.speed)
        sa        = dry_pattern_sa
        array     = dry_array

    else:
        from azimuth_pattern_sweep import Turntable, SpectrumAnalyzer

        turntable = Turntable(
            ip           = TURNTABLE_IP,
            port         = TURNTABLE_PORT,
            cmd_delay    = TURNTABLE_CMD_DELAY,
            speed_deg_s  = args.speed,
            accel_deg_s2 = TURNTABLE_ACCEL_DEG_S2,
        )
        sa    = SpectrumAnalyzer(resource_string=SA_RESOURCE_STRING)
        array = PhasedArrayAdapter()

    # ── Run sweep ──────────────────────────────────────────────────────────────
    try:
        with turntable, sa, array, SteeringTableLogger(csv_path) as logger:
            sweep = BeamSteeringSweep(
                turntable  = turntable,
                sa         = sa,
                array      = array,
                logger     = logger,
                mode       = args.mode,
                start_deg  = args.start,
                stop_deg   = args.stop,
                step_deg   = args.step,
                calib_hold = args.calib_hold,
                pat_hold   = args.pattern_hold,
                coarse_step      = args.coarse_step,
                num_random_starts = args.random_starts,
            )
            sweep.run()

        print(f"\nLookup table saved to: {csv_path}")

    except KeyboardInterrupt:
        log.warning("Sweep interrupted by user.  Partial results saved to: %s", csv_path)
        sys.exit(1)
    except Exception as e:
        log.exception("Fatal error: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()
