"""
phased_array_pattern_sequence.py
─────────────────────────────────────────────────────────────────────────────
Automated multi-steering-angle radiation pattern measurement.

For each row in PHASE_VECTORS:
  1. Set all attenuators to 0 (maximum gain / no attenuation).
  2. Apply the phase vector to the four phase shifters (P1–P4).
  3. Run a full azimuth sweep via azimuth_pattern_sweep.run_sweep().
  4. Save the CSV and polar plot with a filename that encodes the phase
     vector, e.g.:  pattern_20260404_141019_P0-16-32-48.csv / .png

The turntable and spectrum analyzer are opened once and stay connected
across all sweeps — no reconnect latency between steering angles.

Hardware
────────
  • FT232H + SN74HCT138 phased array  (HardwareControl, hardware_control.py)
  • FCU3/0-S turntable                 (Turntable,       azimuth_pattern_sweep.py)
  • MS2690A spectrum analyzer          (SpectrumAnalyzer, azimuth_pattern_sweep.py)

Usage
─────
  Adjust the CONFIGURATION section below, then:
      python phased_array_pattern_sequence.py

  Dry-run (no hardware — simulates both turntable and SA):
      python phased_array_pattern_sequence.py --dry-run

  The --dry-run flag does NOT simulate the FT232H/phased-array; that
  hardware must be present (or its calls will be silently skipped by
  HardwareControl's own error handling) unless you are also mocking it.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

# ── Project imports ───────────────────────────────────────────────────────────
# Both files are expected to be in the same directory as this script, or on
# the Python path.
from hardware_control import HardwareControl
from azimuth_pattern_sweep import (
    Turntable,
    SpectrumAnalyzer,
    _DryRunTurntable,
    _DryRunSA,
    run_sweep,
    TURNTABLE_IP,
    TURNTABLE_PORT,
    TURNTABLE_CMD_DELAY,
    TURNTABLE_SPEED_DEG_S,
    TURNTABLE_ACCEL_DEG_S2,
    SA_RESOURCE_STRING,
    SWEEP_START_DEG,
    SWEEP_STOP_DEG,
    SWEEP_STEP_DEG,
    HOLD_TIME_SEC,
)


# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION  –  edit these values before running
# ═══════════════════════════════════════════════════════════════════════════════

# ── FT232H serial number ───────────────────────────────────────────────────────
# Set to None if only one FT232H is connected; set to its serial number string
# (e.g. "FT4XXXXXX") if multiple FT232H devices are present on the same host.
FT232H_SERIAL: Optional[str] = None

# ── Attenuator IDs ────────────────────────────────────────────────────────────
# Device IDs for the four attenuators (1-indexed, matching send_spi_command 'A').
ATTENUATOR_IDS: List[int] = [1, 2, 3, 4]

# ── Attenuation value that corresponds to "no attenuation / max gain" ─────────
# For most RFSA-type devices 0 = minimum attenuation.  Change if your
# attenuator's zero-attenuation code is different (e.g. 63 for max gain
# on an inverted-sense device).
ATTENUATOR_ZERO_VALUE: int = 0

# ── Phase vectors (4 × N array) ───────────────────────────────────────────────
# Each row is one steering angle: [P1_state, P2_state, P3_state, P4_state]
# States are 6-bit integers in the range 0–63 (RFSA3713).
#
# Replace / extend this table with your actual beam-steering phase vectors.
# The sequence is executed top-to-bottom.
PHASE_VECTORS: List[List[int]] = [
    [ 0,  0,  0,  0],   # Broadside / no steering
    [ 8, 16, 24, 32],
    [16, 32, 48,  0],
    [24, 48,  8, 32],
    [32,  0, 32,  0],
    [48, 32, 16,  0],
]

# ── Sweep parameters (overridden by CLI flags) ────────────────────────────────
SEQUENCE_START_DEG = SWEEP_START_DEG   # inherits from azimuth_pattern_sweep
SEQUENCE_STOP_DEG  = SWEEP_STOP_DEG
SEQUENCE_STEP_DEG  = SWEEP_STEP_DEG
SEQUENCE_HOLD_SEC  = HOLD_TIME_SEC

# ── Output directory ──────────────────────────────────────────────────────────
LOG_DIR = Path("./SweepSequenceLogs")


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
# HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

def phase_vector_to_suffix(phases: List[int]) -> str:
    """
    Convert a phase vector to a safe filename suffix.

    [0, 16, 32, 48]  →  "P0-16-32-48"

    The 'P' prefix makes it immediately clear the numbers are phase states,
    not dB values or angles.
    """
    return "P" + "-".join(str(v) for v in phases)


def apply_hardware_state(
    hw:            HardwareControl,
    phases:        List[int],
    attenuator_ids: List[int],
    atten_value:   int,
) -> None:
    """
    Set all attenuators to *atten_value* and all phase shifters to *phases*.

    Attenuators are set first so the array is quiet while phase values are
    loaded — avoids a brief burst at an intermediate steering angle.

    Parameters
    ──────────
    hw             Initialised HardwareControl instance.
    phases         List of 4 phase states [P1, P2, P3, P4] (0–63).
    attenuator_ids List of attenuator device IDs to zero.
    atten_value    Code sent to each attenuator (0 = minimum attenuation).
    """
    # ── Step 1: zero all attenuators ──────────────────────────────────────────
    log.info("  Setting attenuators %s → %d", attenuator_ids, atten_value)
    for aid in attenuator_ids:
        hw.set_gain(aid, atten_value)

    # ── Step 2: apply phase vector ────────────────────────────────────────────
    log.info("  Applying phase vector: %s", phases)
    for idx, state in enumerate(phases):
        device_id = idx + 1        # P1 = device_id 1, …, P4 = device_id 4
        hw.set_phase(device_id, state)
        log.debug("    P%d ← %d", device_id, state)


# ═══════════════════════════════════════════════════════════════════════════════
# SEQUENCE RUNNER
# ═══════════════════════════════════════════════════════════════════════════════

def run_sequence(
    hw:          HardwareControl,
    turntable,                     # Turntable | _DryRunTurntable
    sa,                            # SpectrumAnalyzer | _DryRunSA
    phase_vectors: List[List[int]],
    log_dir:     Path,
    start_deg:   float,
    stop_deg:    float,
    step_deg:    float,
    hold_sec:    float,
) -> None:
    """
    Iterate through *phase_vectors*, configure hardware, and run one sweep
    per entry.  The turntable and SA are assumed to be already open.

    A summary table is printed at the end showing peak power and angle for
    each steering vector.
    """
    n_total  = len(phase_vectors)
    t_start  = time.monotonic()
    summary  = []   # list of (suffix, peak_dbm, peak_angle_deg)

    for seq_idx, phases in enumerate(phase_vectors):
        suffix = phase_vector_to_suffix(phases)

        log.info("═" * 70)
        log.info(
            "SEQUENCE  %d / %d   phase vector: %s   suffix: %s",
            seq_idx + 1, n_total, phases, suffix,
        )
        log.info("═" * 70)

        # ── Configure hardware ─────────────────────────────────────────────
        apply_hardware_state(
            hw            = hw,
            phases        = phases,
            attenuator_ids = ATTENUATOR_IDS,
            atten_value   = ATTENUATOR_ZERO_VALUE,
        )

        # ── Run sweep ─────────────────────────────────────────────────────
        results = run_sweep(
            turntable   = turntable,
            sa          = sa,
            log_dir     = log_dir,
            file_suffix = suffix,
            start_deg   = start_deg,
            stop_deg    = stop_deg,
            step_deg    = step_deg,
            hold_sec    = hold_sec,
        )

        # ── Collect summary info ───────────────────────────────────────────
        if results:
            best   = max(results, key=lambda p: p.power_dbm)
            summary.append((suffix, best.power_dbm, best.angle_deg))
            log.info(
                "  ✓ Sweep complete.  Peak: %+.2f dBm @ %+.2f°",
                best.power_dbm, best.angle_deg,
            )
        else:
            summary.append((suffix, float("nan"), float("nan")))
            log.warning("  ✗ No data collected for vector %s.", suffix)

    # ── Final summary ──────────────────────────────────────────────────────
    elapsed = time.monotonic() - t_start
    width   = max(len(s[0]) for s in summary) if summary else 20

    print()
    print("=" * (width + 40))
    print(f"  SEQUENCE COMPLETE  –  {n_total} vectors  in  {elapsed:.0f} s ({elapsed/60:.1f} min)")
    print("=" * (width + 40))
    print(f"  {'Phase Vector':<{width}}   {'Peak (dBm)':>12}   {'At Angle (°)':>14}")
    print(f"  {'-'*width}   {'-'*12}   {'-'*14}")
    for suf, pwr, ang in summary:
        pwr_str = f"{pwr:+.2f}" if pwr == pwr else "  (no data)"   # NaN check
        ang_str = f"{ang:+.2f}" if ang == ang else "  —"
        print(f"  {suf:<{width}}   {pwr_str:>12}   {ang_str:>14}")
    print("=" * (width + 40))
    print()


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Automated multi-steering-angle azimuth pattern sequence. "
            "Iterates PHASE_VECTORS, applies each to the phased array, "
            "and runs an azimuth sweep per vector."
        )
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Simulate turntable and SA (FT232H must still be present, or "
             "HardwareControl will silently skip SPI writes).",
    )
    parser.add_argument(
        "--log-dir", type=Path, default=LOG_DIR,
        help="Output directory for CSV logs and polar plots.",
    )
    parser.add_argument(
        "--start", type=float, default=SEQUENCE_START_DEG,
        help=f"Sweep start angle (°)  [default: {SEQUENCE_START_DEG}].",
    )
    parser.add_argument(
        "--stop", type=float, default=SEQUENCE_STOP_DEG,
        help=f"Sweep stop angle (°)   [default: {SEQUENCE_STOP_DEG}].",
    )
    parser.add_argument(
        "--step", type=float, default=SEQUENCE_STEP_DEG,
        help=f"Angular step size (°)  [default: {SEQUENCE_STEP_DEG}].",
    )
    parser.add_argument(
        "--hold", type=float, default=SEQUENCE_HOLD_SEC,
        help=f"Max Hold dwell per point (s)  [default: {SEQUENCE_HOLD_SEC}].",
    )
    parser.add_argument(
        "--speed", type=float, default=TURNTABLE_SPEED_DEG_S,
        help=f"Turntable speed (°/s)  [default: {TURNTABLE_SPEED_DEG_S}].",
    )
    parser.add_argument(
        "--accel", type=float, default=TURNTABLE_ACCEL_DEG_S2,
        help=f"Turntable acceleration (°/s²)  [default: {TURNTABLE_ACCEL_DEG_S2}].",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # ── Validate phase vector table ────────────────────────────────────────────
    if not PHASE_VECTORS:
        log.error("PHASE_VECTORS is empty — nothing to do.")
        sys.exit(1)
    for i, pv in enumerate(PHASE_VECTORS):
        if len(pv) != 4:
            log.error("PHASE_VECTORS[%d] has %d elements; expected 4.", i, len(pv))
            sys.exit(1)
        for j, s in enumerate(pv):
            if not (0 <= s <= 63):
                log.error(
                    "PHASE_VECTORS[%d][%d] = %d is out of range 0–63.", i, j, s
                )
                sys.exit(1)

    log.info("Phase vector table: %d entries", len(PHASE_VECTORS))
    for i, pv in enumerate(PHASE_VECTORS):
        log.info("  [%2d]  %s  →  %s", i, pv, phase_vector_to_suffix(pv))

    # ── Initialise phased-array hardware ──────────────────────────────────────
    log.info("Initialising phased array hardware (FT232H serial: %s) …", FT232H_SERIAL)
    hw = HardwareControl(serial_number=FT232H_SERIAL)
    if hw.spi is None:
        log.warning(
            "HardwareControl: SPI not available.  Phase/gain writes will be "
            "silently skipped.  If this is unexpected, check FT232H connection."
        )

    # ── Instantiate turntable and SA ──────────────────────────────────────────
    if args.dry_run:
        log.warning("DRY-RUN: turntable and SA are simulated.")
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

    # ── Run the sequence with turntable + SA kept open throughout ─────────────
    try:
        with turntable, sa:
            run_sequence(
                hw            = hw,
                turntable     = turntable,
                sa            = sa,
                phase_vectors = PHASE_VECTORS,
                log_dir       = args.log_dir,
                start_deg     = args.start,
                stop_deg      = args.stop,
                step_deg      = args.step,
                hold_sec      = args.hold,
            )
    except KeyboardInterrupt:
        log.warning("Sequence interrupted by user.")
        sys.exit(1)
    except Exception as e:
        log.exception("Fatal error during sequence: %s", e)
        sys.exit(1)
    finally:
        hw.deinitialize_hardware()


if __name__ == "__main__":
    main()