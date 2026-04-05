"""
azimuth_pattern_sweep.py
─────────────────────────────────────────────────────────────────────────────
Measures an azimuth radiation pattern using:
  • FCU3/0-S turntable  – controlled via TCP socket (LAN)
  • MS2690A spectrum analyzer – controlled via VISA (USB/LAN)

The signal source is assumed to be pre-configured on the MS2690A before
this script is launched; no SA frequency/span configuration is performed.

Algorithm
─────────
For each angular step in the sweep range:
  1. Command the turntable to the target angle and wait for arrival.
  2. Reset the SA Max Hold trace to clear any prior history.
  3. Dwell for HOLD_TIME_SEC seconds, accumulating Max Hold data.
  4. Run a peak search and record the marker amplitude (dBm).
  5. Print the result immediately and flush it to the CSV log.

After the sweep a polar plot is generated.  If the sweep does not cover
the full 360 °, the missing arc is padded with NaN so the plot always
displays a complete polar axes.

Usage – standalone
──────────────────
  Adjust the CONFIGURATION section below, then:
      python azimuth_pattern_sweep.py

  Use --dry-run to simulate both instruments (no hardware needed):
      python azimuth_pattern_sweep.py --dry-run

  Use --suffix to embed a label in the output filenames:
      python azimuth_pattern_sweep.py --suffix "P0-16-32-48"
      → pattern_20260404_141019_P0-16-32-48.csv / .png

Public API (import from another script)
────────────────────────────────────────
  from azimuth_pattern_sweep import (
      Turntable, SpectrumAnalyzer,
      _DryRunTurntable, _DryRunSA,   # dry-run stubs
      run_sweep,                      # main entry point
  )

  run_sweep(turntable, sa, log_dir, file_suffix="P0-16-32-48",
            start_deg=-90, stop_deg=90, step_deg=2, hold_sec=3)

  Passing already-open instrument objects means the turntable and SA
  stay connected across multiple back-to-back sweeps — no reconnect
  overhead or dropped state between steering angles.
"""

from __future__ import annotations

import argparse
import csv
import logging
import math
import socket
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple


# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION  –  edit these values before running
# ═══════════════════════════════════════════════════════════════════════════════

# ── Turntable (FCU3/0-S, LAN) ─────────────────────────────────────────────────
TURNTABLE_IP          = "172.16.1.168"   # IP address of the FCU3 controller
TURNTABLE_PORT        = 200              # TCP port (default 200)
TURNTABLE_TCP_BUFFER  = 128             # Receive buffer size (bytes)
TURNTABLE_CMD_DELAY   = 0.15            # Seconds to wait after each write
                                         # (prevents command loss on fast bursts)

TURNTABLE_SPEED_DEG_S  = 2.0    # Rotation speed  (°/s)
TURNTABLE_ACCEL_DEG_S2 = 4.0   # Acceleration    (°/s²) — verify command below
                                 # against your FCU3 firmware if this doesn't work

# ── Sweep Parameters ──────────────────────────────────────────────────────────
SWEEP_START_DEG    = -50.0   # Start angle (°); typical values: -180, -90, 0
SWEEP_STOP_DEG     =  50.0   # Stop angle  (°); typical values:  180,  90, 360
SWEEP_STEP_DEG     =    10.0   # Angular step between measurement points (°)
                               # Smaller = finer resolution, more measurement time

HOLD_TIME_SEC      =   3.0    # Max Hold dwell time at each angle (seconds)
                               # Increase for noisier/intermittent signals

# ── Spectrum Analyzer ──────────────────────────────────────────────────────────
SA_RESOURCE_STRING = 'USB0::2907::6::6261932852::0::INSTR'  # pyvisa resource

# ── Output ────────────────────────────────────────────────────────────────────
LOG_DIR = Path("./sweeps/")           # CSV + plot saved here


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
class PatternPoint:
    """One measured point in the azimuth radiation pattern."""
    angle_deg:  float          # Nominal turntable angle (°)
    power_dbm:  float          # Max-hold peak power (dBm)
    freq_hz:    float          # Marker frequency (Hz) returned by the SA
    timestamp:  str = field(default_factory=lambda: datetime.now().isoformat())

    def __str__(self) -> str:
        return (f"  {self.angle_deg:+8.2f}°   {self.power_dbm:+.2f} dBm"
                f"   @ {self.freq_hz/1e9:.6f} GHz   [{self.timestamp}]")


# ═══════════════════════════════════════════════════════════════════════════════
# TURNTABLE CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════

class Turntable:
    """
    Controls an FCU3/0-S turntable via a raw TCP socket.

    A new socket connection is opened for every command so that the
    instrument's keep-alive timer is never tripped during long dwells.
    This mirrors the structure shown in the FCU3 manual example code.

    Command subset used
    ───────────────────
      *IDN?             – Identify instrument
      LD {n} DV         – Select device n as active (1 = turntable)
      LD {v} SF         – Set speed to v (°/s)
      LD {v} AC         – Set acceleration to v (°/s²)  [check firmware docs]
      LD {deg} DG NP GO – Go to absolute position {deg} degrees
      RP                – Query current position
      BU                – Query busy status  (1 = moving, 0 = idle)
    """

    def __init__(
        self,
        ip:           str   = TURNTABLE_IP,
        port:         int   = TURNTABLE_PORT,
        cmd_delay:    float = TURNTABLE_CMD_DELAY,
        speed_deg_s:  float = TURNTABLE_SPEED_DEG_S,
        accel_deg_s2: float = TURNTABLE_ACCEL_DEG_S2,
    ) -> None:
        self._ip          = ip
        self._port        = port
        self._cmd_delay   = cmd_delay
        self._speed       = speed_deg_s
        self._accel       = accel_deg_s2
        self._buf_size    = TURNTABLE_TCP_BUFFER

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Identify and configure the turntable (speed, acceleration, device)."""
        idn = self._query("*IDN?")
        log.info("Turntable connected: %s", idn)

        # Select device 1 (turntable axis)
        self._write("LD 1 DV")

        # Set motion parameters
        self._write(f"LD {self._speed:.3f} SF")
        log.info("Turntable speed set to %.3f °/s", self._speed)

        # NOTE: 'AC' is the typical FCU3 acceleration command; confirm against
        # your firmware version — some units use a different mnemonic.
        self._write(f"LD {self._accel:.3f} AC")
        log.info("Turntable acceleration set to %.3f °/s²", self._accel)

    def close(self) -> None:
        log.info("Turntable connection closed.")

    def __enter__(self) -> "Turntable":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # ── Private TCP helpers ────────────────────────────────────────────────────

    def _write(self, cmd: str) -> None:
        """Send a command string; do not read a reply."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self._ip, self._port))
            s.send(cmd.encode())
            log.debug("→ TT  %s", cmd)
        time.sleep(self._cmd_delay)

    def _query(self, cmd: str) -> str:
        """Send a command string and return the null-terminated reply."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self._ip, self._port))
            s.send(cmd.encode())
            raw = s.recv(self._buf_size)
        # Strip null terminator that FCU3 appends
        value = raw.decode("utf-8").rstrip("\x00").strip()
        log.debug("← TT  %s", value)
        return value

    # ── Public API ─────────────────────────────────────────────────────────────

    def get_position(self) -> float:
        """Return current turntable position in degrees."""
        return float(self._query("RP"))

    def is_busy(self) -> bool:
        """Return True while the turntable is moving."""
        return self._query("BU") == "1"

    def goto(self, angle_deg: float, poll_interval: float = 0.25) -> None:
        """
        Command an absolute move to *angle_deg* and block until arrival.

        The FCU3 uses the compound command:
            LD {angle} DG NP GO
        where DG = degrees, NP = new position, GO = execute move.
        """
        cmd = f"LD {angle_deg:.3f} DG NP GO"
        log.info("  → Moving to %+.2f°  (cmd: %s)", angle_deg, cmd)
        self._write(cmd)

        # Wait for motion to begin (BU transitions 0 → 1)
        start = time.monotonic()
        while not self.is_busy():
            if time.monotonic() - start > 3.0:
                log.warning("  Turntable did not report busy within 3 s; "
                            "assuming already at target or very short move.")
                return
            time.sleep(poll_interval)

        # Wait for motion to complete (BU transitions 1 → 0)
        while self.is_busy():
            pos = self.get_position()
            log.debug("    current position: %+.2f°", pos)
            time.sleep(poll_interval)

        final_pos = self.get_position()
        log.info("  ✓ Arrived at %+.2f°", final_pos)


# ═══════════════════════════════════════════════════════════════════════════════
# SPECTRUM ANALYZER CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════

class SpectrumAnalyzer:
    """
    Thin VISA wrapper for the MS2690A.

    The SA center frequency and span are assumed to already be set on the
    instrument.  This class only manages the Max Hold trace and marker.
    """

    def __init__(self, resource_string: str) -> None:
        self._resource_string = resource_string
        self._inst            = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def connect(self) -> None:
        import pyvisa
        rm = pyvisa.ResourceManager()
        log.info("Connecting to spectrum analyzer: %s", self._resource_string)
        self._inst = rm.open_resource(self._resource_string)
        self._inst.timeout = 15_000  # ms

        idn = self._inst.query("*IDN?").strip()
        log.info("SA connected: %s", idn)

        # Enable Max Hold trace and a normal marker
        self._inst.write("TRACe1:STORage:MODE MAXHold")
        self._inst.write("INITiate:CONTinuous ON")
        self._inst.write("CALC:MARK:STAT ON")
        self._inst.write("CALC:MARK:MODE NORM")
        log.info("SA Max Hold enabled, continuous sweep running.")

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

    def reset_max_hold(self) -> None:
        """
        Clear the Max Hold trace so the next accumulation period starts fresh.
        Re-issuing TRACe1:STORage:MODE MAXHold causes the SA to discard the
        prior held trace and begin a new accumulation from the next sweep.
        """
        self._inst.write("TRACe1:STORage:MODE MAXHold")

    def measure_peak_after_hold(self, hold_sec: float) -> Tuple[float, float]:
        """
        Reset Max Hold, dwell for *hold_sec* seconds to accumulate, then
        run a peak search and return (frequency_hz, power_dbm).
        """
        self.reset_max_hold()
        time.sleep(hold_sec)   # accumulate Max Hold data
        self._inst.write("CALC:MARK:MAX")
        freq_hz   = float(self._inst.query("CALC:MARK:X?"))
        power_dbm = float(self._inst.query("CALC:MARK:Y?"))
        return freq_hz, power_dbm


# ═══════════════════════════════════════════════════════════════════════════════
# DRY-RUN STUBS
# ═══════════════════════════════════════════════════════════════════════════════

class _DryRunTurntable:
    """Simulates turntable moves with wall-clock time proportional to angle."""

    def __init__(self, speed_deg_s: float = TURNTABLE_SPEED_DEG_S) -> None:
        self._speed   = speed_deg_s
        self._pos     = 0.0

    def connect(self)  -> None: log.info("[DRY RUN] Turntable connected (simulated).")
    def close(self)    -> None: log.info("[DRY RUN] Turntable closed.")
    def __enter__(self): self.connect(); return self
    def __exit__(self, *_): self.close()

    def get_position(self) -> float: return self._pos
    def is_busy(self) -> bool: return False  # Never actually blocks in dry-run

    def goto(self, angle_deg: float, poll_interval: float = 0.25) -> None:
        dist = abs(angle_deg - self._pos)
        travel_sec = dist / self._speed
        log.info("  [DRY RUN] → Moving %.2f° (simulated %.1f s travel)", dist, travel_sec)
        time.sleep(min(travel_sec, 0.5))   # cap sim delay to keep tests fast
        self._pos = angle_deg
        log.info("  [DRY RUN] ✓ Arrived at %+.2f°", self._pos)


class _DryRunSA:
    """Simulates an SA with a cosine-shaped pattern centred at 0 °."""

    def __init__(self) -> None:
        self._current_angle = 0.0

    def connect(self)  -> None: log.info("[DRY RUN] SA connected (simulated).")
    def close(self)    -> None: log.info("[DRY RUN] SA closed.")
    def __enter__(self): self.connect(); return self
    def __exit__(self, *_): self.close()

    def set_angle(self, angle_deg: float) -> None:
        self._current_angle = angle_deg

    def reset_max_hold(self) -> None: pass

    def measure_peak_after_hold(self, hold_sec: float) -> Tuple[float, float]:
        import random
        time.sleep(hold_sec * 0.1)   # minimal delay in dry-run
        theta  = math.radians(self._current_angle)
        # Simple two-lobe pattern with noise
        power  = -30.0 + 20.0 * (math.cos(theta) ** 2) + random.gauss(0, 0.5)
        return 2.345e9, power


# ═══════════════════════════════════════════════════════════════════════════════
# CSV LOGGER
# ═══════════════════════════════════════════════════════════════════════════════

class PatternLogger:
    """
    Writes each PatternPoint to a CSV file immediately after measurement.

    The *filepath* passed in is used verbatim, so callers control the name.
    Use ``make_file_stem()`` to build consistent timestamped names with an
    optional suffix that identifies the phase/steering vector being measured.
    """

    HEADER = ["timestamp", "angle_deg", "power_dbm", "freq_hz"]

    def __init__(self, filepath: Path) -> None:
        self._filepath = filepath
        self._file     = None
        self._writer   = None

    def open(self) -> None:
        self._file   = open(self._filepath, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self._file.flush()
        log.info("Logging pattern data to: %s", self._filepath)

    def close(self) -> None:
        if self._file:
            self._file.close()
            log.info("CSV log closed: %s", self._filepath)

    def __enter__(self) -> "PatternLogger":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def record(self, pt: PatternPoint) -> None:
        self._writer.writerow([
            pt.timestamp,
            f"{pt.angle_deg:.4f}",
            f"{pt.power_dbm:.4f}",
            f"{pt.freq_hz:.2f}",
        ])
        self._file.flush()   # write to disk immediately — do not buffer


# ═══════════════════════════════════════════════════════════════════════════════
# PATTERN SWEEP ENGINE
# ═══════════════════════════════════════════════════════════════════════════════

class PatternSweeper:
    """
    Steps the turntable through the configured angular range and records
    the Max Hold peak power at each position.
    """

    def __init__(
        self,
        turntable:   "Turntable | _DryRunTurntable",
        sa:          "SpectrumAnalyzer | _DryRunSA",
        logger:      PatternLogger,
        start_deg:   float = SWEEP_START_DEG,
        stop_deg:    float = SWEEP_STOP_DEG,
        step_deg:    float = SWEEP_STEP_DEG,
        hold_sec:    float = HOLD_TIME_SEC,
    ) -> None:
        self._tt       = turntable
        self._sa       = sa
        self._logger   = logger
        self._start    = start_deg
        self._stop     = stop_deg
        self._step     = step_deg
        self._hold     = hold_sec

    def _build_angle_list(self) -> List[float]:
        """Generate the list of nominal target angles for the sweep."""
        angles: List[float] = []
        ang = self._start
        while ang <= self._stop + 1e-9:   # small epsilon handles float rounding
            angles.append(round(ang, 6))
            ang += self._step
        return angles

    def run(self) -> List[PatternPoint]:
        """Execute the full sweep and return all measured PatternPoints."""
        angles   = self._build_angle_list()
        n_pts    = len(angles)
        results: List[PatternPoint] = []

        log.info("═" * 60)
        log.info("AZIMUTH SWEEP  start=%.1f°  stop=%.1f°  step=%.2f°  "
                 "points=%d  hold=%.1f s",
                 self._start, self._stop, self._step, n_pts, self._hold)
        est_min = n_pts * self._hold / 60.0
        log.info("Estimated minimum measurement time: ~%.1f min (excludes travel)", est_min)
        log.info("═" * 60)

        t_sweep_start = time.monotonic()

        for i, angle in enumerate(angles):
            log.info("─" * 50)
            log.info("Point %d / %d   target = %+.2f°", i + 1, n_pts, angle)

            # ── 1. Move turntable ──────────────────────────────────────────────
            self._tt.goto(angle)

            # Dry-run SA needs to know the angle for its pattern model
            if hasattr(self._sa, "set_angle"):
                self._sa.set_angle(angle)

            # ── 2 & 3. Reset Max Hold, dwell, then peak-search ─────────────────
            log.info("  Accumulating Max Hold for %.1f s …", self._hold)
            freq_hz, power_dbm = self._sa.measure_peak_after_hold(self._hold)

            # ── 4. Record ──────────────────────────────────────────────────────
            pt = PatternPoint(
                angle_deg = angle,
                power_dbm = power_dbm,
                freq_hz   = freq_hz,
            )
            self._logger.record(pt)
            results.append(pt)

            # ── 5. Print immediately to terminal ──────────────────────────────
            print(pt)

        elapsed = time.monotonic() - t_sweep_start
        log.info("═" * 60)
        log.info("SWEEP COMPLETE  –  %d points  in  %.1f s (%.1f min)",
                 n_pts, elapsed, elapsed / 60.0)
        log.info("═" * 60)

        return results


# ═══════════════════════════════════════════════════════════════════════════════
# POLAR PLOT
# ═══════════════════════════════════════════════════════════════════════════════

def generate_polar_plot(
    results:       List[PatternPoint],
    out_path:      Path,
    start_deg:     float,
    stop_deg:      float,
    step_deg:      float,
    dynamic_range: float = 40.0,
) -> None:
    """
    Render a polar plot of the azimuth radiation pattern with absolute dBm axis.

    Radial axis strategy
    ────────────────────
    Matplotlib polar axes require non-negative radii.  We therefore apply a
    linear shift so that the noise floor (peak − dynamic_range) maps to r = 0
    and the peak maps to r = dynamic_range.  The concentric ring labels are
    then back-converted to display the true dBm values.

        r = clip(power_dbm − floor_dbm, 0, dynamic_range)
        floor_dbm = peak_dbm − dynamic_range

    Un-measured angles are padded with r = 0 (i.e. they sit at the floor
    circle) and drawn as a dashed grey line so the full 360° circle is always
    visible.
    """
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        log.error("matplotlib / numpy not installed; skipping polar plot.")
        log.error("Install with:  pip install matplotlib numpy")
        return

    # ── Build a full 360° angle grid ─────────────────────────────────────────
    all_angles_deg = np.arange(-180.0, 180.0 + step_deg / 2.0, step_deg)
    power_map      = {pt.angle_deg: pt.power_dbm for pt in results}
    power_dbm_arr  = np.array([power_map.get(a, np.nan) for a in all_angles_deg])

    # Convert angles to radians (matplotlib polar convention)
    theta = np.deg2rad(all_angles_deg)

    # ── dBm → radius mapping ──────────────────────────────────────────────────
    peak_dbm  = float(np.nanmax(power_dbm_arr))
    floor_dbm = peak_dbm - dynamic_range          # e.g. peak=−9.3 → floor=−49.3

    # r = 0 at floor, r = dynamic_range at peak; clip below-floor values to 0
    radius_arr = np.clip(power_dbm_arr - floor_dbm, 0.0, dynamic_range)

    is_measured = ~np.isnan(power_dbm_arr)        # True where we have real data
    is_padded   = ~is_measured

    # ── Concentric ring positions & labels in dBm ─────────────────────────────
    # Place a ring every 10 dB from the floor up to (and including) the peak,
    # rounding the floor down to the nearest 10 dBm for clean numbers.
    floor_rounded = math.floor(floor_dbm / 10.0) * 10.0
    ring_dbm      = np.arange(floor_rounded, peak_dbm + 1.0, 10.0)
    ring_r        = np.clip(ring_dbm - floor_dbm, 0.0, dynamic_range)

    # ── Plot setup ────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(9, 9))
    ax  = fig.add_subplot(111, polar=True)

    # 0° at top, clockwise positive (azimuth / compass convention)
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)

    # ── Measured arc ──────────────────────────────────────────────────────────
    meas_theta  = theta[is_measured]
    meas_radius = radius_arr[is_measured]

    # Close the arc by appending the first point so fill() doesn't leave a gap
    if len(meas_theta) > 1:
        meas_theta_closed  = np.append(meas_theta,  meas_theta[0])
        meas_radius_closed = np.append(meas_radius, meas_radius[0])
    else:
        meas_theta_closed  = meas_theta
        meas_radius_closed = meas_radius

    ax.plot(meas_theta_closed, meas_radius_closed,
            color="royalblue", linewidth=1.5, label="Measured")
    ax.fill(meas_theta_closed, meas_radius_closed,
            alpha=0.18, color="royalblue")

    # ── Padded (un-measured) arc at the floor ─────────────────────────────────
    if is_padded.any():
        pad_theta  = theta[is_padded]
        pad_radius = np.zeros_like(pad_theta)     # sits exactly on the floor ring
        ax.plot(pad_theta, pad_radius,
                color="lightgrey", linewidth=1.0, linestyle="--",
                label="Not measured")

    # ── Radial axis: rings labelled in dBm ────────────────────────────────────
    ax.set_ylim(0, dynamic_range * 1.05)          # small headroom so peak label fits
    ax.set_yticks(ring_r)
    ax.set_yticklabels([f"{v:.0f} dBm" for v in ring_dbm], fontsize=7)
    ax.yaxis.set_tick_params(labelcolor="dimgrey")

    # ── Angular grid lines every 30° ──────────────────────────────────────────
    ax.set_thetagrids(range(0, 360, 30))

    # ── Title ─────────────────────────────────────────────────────────────────
    peak_idx   = int(np.nanargmax([pt.power_dbm for pt in results]))
    peak_angle = results[peak_idx].angle_deg
    title_lines = [
        "Azimuth Radiation Pattern",
        f"Start {start_deg:+.1f}°  →  Stop {stop_deg:+.1f}°  (step {step_deg:.2f}°)",
        f"Peak: {peak_dbm:+.2f} dBm  @ {peak_angle:+.2f}°",
        datetime.now().strftime("%Y-%m-%d  %H:%M:%S"),
    ]
    ax.set_title("\n".join(title_lines), va="bottom", pad=22, fontsize=10)
    ax.legend(loc="lower right", bbox_to_anchor=(1.3, -0.05), fontsize=8)

    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    log.info("Polar plot saved: %s", out_path)
    plt.close(fig)


# ═══════════════════════════════════════════════════════════════════════════════
# PUBLIC API  (importable by external scripts such as a sequence runner)
# ═══════════════════════════════════════════════════════════════════════════════

def make_file_stem(timestamp_str: str, file_suffix: str = "") -> str:
    """
    Build the base filename (without extension) for a sweep's outputs.

    Format:
        pattern_{timestamp}              – no suffix
        pattern_{timestamp}_{suffix}     – with suffix

    The suffix is sanitised: any character that is not alphanumeric, a hyphen,
    or an underscore is replaced with '_' so the result is safe as a filename
    on all platforms.

    Examples:
        make_file_stem("20260404_141019")                → "pattern_20260404_141019"
        make_file_stem("20260404_141019", "P0-16-32-48") → "pattern_20260404_141019_P0-16-32-48"
    """
    import re
    safe_suffix = re.sub(r"[^\w\-]", "_", file_suffix).strip("_")
    if safe_suffix:
        return f"pattern_{timestamp_str}_{safe_suffix}"
    return f"pattern_{timestamp_str}"


def run_sweep(
    turntable:   "Turntable | _DryRunTurntable",
    sa:          "SpectrumAnalyzer | _DryRunSA",
    log_dir:     Path = LOG_DIR,
    file_suffix: str  = "",
    start_deg:   float = SWEEP_START_DEG,
    stop_deg:    float = SWEEP_STOP_DEG,
    step_deg:    float = SWEEP_STEP_DEG,
    hold_sec:    float = HOLD_TIME_SEC,
) -> List[PatternPoint]:
    """
    Run a single azimuth pattern sweep using already-open instrument objects.

    This is the primary importable entry point for external scripts (e.g. a
    multi-steering-angle sequence runner).  Passing pre-connected instruments
    means the turntable and SA stay live across back-to-back calls — no
    reconnect latency or lost configuration between sweeps.

    Parameters
    ──────────
    turntable    Open Turntable (or _DryRunTurntable) instance.
    sa           Open SpectrumAnalyzer (or _DryRunSA) instance.
    log_dir      Directory for CSV and PNG outputs.
    file_suffix  String appended to the timestamped filename stem, e.g.
                 "P0-16-32-48".  Sanitised automatically.
    start_deg    Sweep start angle (°).
    stop_deg     Sweep stop angle (°).
    step_deg     Angular step size (°).
    hold_sec     Max Hold dwell time per point (s).

    Returns
    ───────
    List of PatternPoints measured during this sweep (may be partial if an
    exception occurred; the caller should handle KeyboardInterrupt).
    """
    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = make_file_stem(ts, file_suffix)
    csv_path  = log_dir / f"{stem}.csv"
    plot_path = log_dir / f"{stem}.png"

    results: List[PatternPoint] = []
    try:
        with PatternLogger(csv_path) as logger:
            sweeper = PatternSweeper(
                turntable = turntable,
                sa        = sa,
                logger    = logger,
                start_deg = start_deg,
                stop_deg  = stop_deg,
                step_deg  = step_deg,
                hold_sec  = hold_sec,
            )
            results = sweeper.run()
    finally:
        if results:
            log.info("Sweep outputs  →  %s  |  %s", csv_path.name, plot_path.name)
            generate_polar_plot(
                results    = results,
                out_path   = plot_path,
                start_deg  = start_deg,
                stop_deg   = stop_deg,
                step_deg   = step_deg,
            )
        else:
            log.warning("run_sweep: no data collected — skipping plot.")

    return results


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Azimuth radiation pattern sweep using FCU3 turntable + MS2690A."
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Simulate both instruments (no hardware required).",
    )
    parser.add_argument(
        "--log-dir", type=Path, default=LOG_DIR,
        help="Directory for CSV log and polar plot (default: current directory).",
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
        help=f"Angular step size in degrees (default: {SWEEP_STEP_DEG}).",
    )
    parser.add_argument(
        "--hold", type=float, default=HOLD_TIME_SEC,
        help=f"Max Hold dwell time per point in seconds (default: {HOLD_TIME_SEC}).",
    )
    parser.add_argument(
        "--speed", type=float, default=TURNTABLE_SPEED_DEG_S,
        help=f"Turntable rotation speed in °/s (default: {TURNTABLE_SPEED_DEG_S}).",
    )
    parser.add_argument(
        "--accel", type=float, default=TURNTABLE_ACCEL_DEG_S2,
        help=f"Turntable acceleration in °/s² (default: {TURNTABLE_ACCEL_DEG_S2}).",
    )
    parser.add_argument(
        "--suffix", type=str, default="",
        help=(
            "Optional label appended to output filenames, e.g. 'P0-16-32-48'. "
            "Produces: pattern_{timestamp}_{suffix}.csv / .png"
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # ── Instantiate hardware (or dry-run stubs) ───────────────────────────────
    if args.dry_run:
        log.warning("DRY-RUN mode: using simulated instruments.")
        turntable = _DryRunTurntable(speed_deg_s=args.speed)
        sa        = _DryRunSA()
    else:
        turntable = Turntable(
            ip           = TURNTABLE_IP,
            port         = TURNTABLE_PORT,
            cmd_delay    = TURNTABLE_CMD_DELAY,
            speed_deg_s  = args.speed,
            accel_deg_s2 = args.accel,
        )
        sa = SpectrumAnalyzer(resource_string=SA_RESOURCE_STRING)

    # ── Run sweep, delegating all file/plot logic to run_sweep() ──────────────
    try:
        with turntable, sa:
            run_sweep(
                turntable   = turntable,
                sa          = sa,
                log_dir     = args.log_dir,
                file_suffix = args.suffix,
                start_deg   = args.start,
                stop_deg    = args.stop,
                step_deg    = args.step,
                hold_sec    = args.hold,
            )
    except KeyboardInterrupt:
        log.warning("Sweep interrupted by user.")
        sys.exit(1)
    except Exception as e:
        log.exception("Fatal error during sweep: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()