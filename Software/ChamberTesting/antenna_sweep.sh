#!/usr/bin/env bash
# =============================================================================
# antenna_sweep.sh
# -----------------------------------------------------------------------------
# For each angle in ANGLES:
#   1. Rotates the turntable to that angle (turn_table_test.py)
#   2. Creates an output directory for that angle
#   3. Runs phase_sweep.py, directing its CSV output into that directory
#
# All tunable parameters are collected at the top of this script under
# "CONFIGURATION".  Any phase_sweep.py argument can also be added to the
# PHASE_SWEEP_EXTRA_ARGS array at the bottom of that section.
#
# Usage
# ─────
#   bash antenna_sweep.sh                  # run with defaults below
#   ANGLES="0 45 90 135 180" bash antenna_sweep.sh
#
# Requirements (see README section at bottom of this file)
#   • Python ≥ 3.9  with pyvisa, pyftdi / adafruit-blinka installed
#   • turn_table_test.py and phase_sweep.py in the same directory as this script
# =============================================================================

set -euo pipefail   # exit on error, unset variable, or pipe failure

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION  –  edit these before running
# ─────────────────────────────────────────────────────────────────────────────

# Sequence of turntable angles (degrees). Separate with spaces.
ANGLES="${ANGLES:-0 30 60 90 120 150 180 210 240 270 300 330}"

# Root directory that will contain one sub-directory per angle.
OUTPUT_ROOT="${OUTPUT_ROOT:-./sweep_results}"

# Python interpreter to use (must have all dependencies installed).
PYTHON="${PYTHON:-python}"

# Paths to the two scripts (default: same directory as this script).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TURNTABLE_SCRIPT="${TURNTABLE_SCRIPT:-${SCRIPT_DIR}/turn_table_test.py}"
PHASE_SWEEP_SCRIPT="${PHASE_SWEEP_SCRIPT:-${SCRIPT_DIR}/phase_sweep.py}"

# ── Turntable parameters ──────────────────────────────────────────────────────
TURNTABLE_IP="${TURNTABLE_IP:-172.16.1.186}"
TURNTABLE_PORT="${TURNTABLE_PORT:-200}"
TURNTABLE_SPEED="${TURNTABLE_SPEED:-3.0}"          # °/s
TURNTABLE_CMD_DELAY="${TURNTABLE_CMD_DELAY:-0.1}"  # seconds

# ── phase_sweep.py parameters ─────────────────────────────────────────────────
# Center frequency of the spectrum analyzer in Hz.
PS_CENTER_FREQ="${PS_CENTER_FREQ:-2.345e9}"

# SA span in Hz.
PS_SPAN="${PS_SPAN:-100e3}"

# Dwell time per phase state in seconds.
PS_DWELL="${PS_DWELL:-2.0}"

# Phase-settle delay before resetting the average, in seconds.
PS_SETTLE="${PS_SETTLE:-0.05}"

# n-values expression (leave empty to sweep all 64).
# Examples: "range(0,64,8)"  |  "[0,16,32,48]"  |  "" (all)
PS_N_VALUES="${PS_N_VALUES:-}"

# m-values expression (leave empty to sweep all 64).
PS_M_VALUES="${PS_M_VALUES:-}"

# Set to "--dry-run" to simulate hardware (no FT232H or SA needed).
PS_DRY_RUN="${PS_DRY_RUN:-}"

# Set to "--verify-spi" to pause before each sweep for SPI verification.
PS_VERIFY_SPI="${PS_VERIFY_SPI:-}"

# Any additional phase_sweep.py arguments (add as array elements).
# Example: PHASE_SWEEP_EXTRA_ARGS=("--some-flag" "--another" "value")
PHASE_SWEEP_EXTRA_ARGS=()

# ─────────────────────────────────────────────────────────────────────────────
# INTERNAL HELPERS
# ─────────────────────────────────────────────────────────────────────────────

log()  { echo "[$(date '+%H:%M:%S')] $*"; }
die()  { echo "ERROR: $*" >&2; exit 1; }

check_prerequisites() {
    [[ -f "$TURNTABLE_SCRIPT" ]] \
        || die "Turntable script not found: $TURNTABLE_SCRIPT"
    [[ -f "$PHASE_SWEEP_SCRIPT" ]] \
        || die "Phase sweep script not found: $PHASE_SWEEP_SCRIPT"
    command -v "$PYTHON" >/dev/null 2>&1 \
        || die "Python interpreter not found: $PYTHON"
}

build_phase_sweep_args() {
    # Build the argument list for phase_sweep.py.
    # The --log-dir is passed in as the first positional argument by the caller.
    local log_dir="$1"
    local args=(
        "--log-dir"     "$log_dir"
        "--center-freq" "$PS_CENTER_FREQ"
        "--span"        "$PS_SPAN"
        "--dwell"       "$PS_DWELL"
        "--settle"      "$PS_SETTLE"
    )
    [[ -n "$PS_N_VALUES"  ]] && args+=("--n-values" "$PS_N_VALUES")
    [[ -n "$PS_M_VALUES"  ]] && args+=("--m-values" "$PS_M_VALUES")
    [[ -n "$PS_DRY_RUN"   ]] && args+=("$PS_DRY_RUN")
    [[ -n "$PS_VERIFY_SPI" ]] && args+=("$PS_VERIFY_SPI")
    args+=("${PHASE_SWEEP_EXTRA_ARGS[@]+"${PHASE_SWEEP_EXTRA_ARGS[@]}"}")
    echo "${args[@]}"
}

# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

main() {
    check_prerequisites

    mkdir -p "$OUTPUT_ROOT"
    log "Sweep root: $(realpath "$OUTPUT_ROOT")"
    log "Angles    : $ANGLES"
    log "════════════════════════════════════════"

    local angle_count
    angle_count=$(echo "$ANGLES" | wc -w)
    local idx=0

    for angle in $ANGLES; do
        idx=$(( idx + 1 ))
        log "[$idx/$angle_count] ── Angle: ${angle} °"

        # ── 1. Rotate the turntable ───────────────────────────────────────────
        log "  Rotating to ${angle} °  …"
        "$PYTHON" "$TURNTABLE_SCRIPT" \
            --position  "$angle"              \
            --ip        "$TURNTABLE_IP"       \
            --port      "$TURNTABLE_PORT"     \
            --speed     "$TURNTABLE_SPEED"    \
            --cmd-delay "$TURNTABLE_CMD_DELAY"

        log "  Turntable at ${angle} °."

        # ── 2. Create output directory for this angle ─────────────────────────
        # Format angle with leading zeros and sign for clean alphabetic sorting.
        # e.g.  0 → "ang_+000"   -30 → "ang_-030"   180 → "ang_+180"
        local angle_dir
        angle_int=$(printf "%.0f" "$angle")   # round to nearest integer
        if (( angle_int >= 0 )); then
            angle_tag=$(printf "ang_%+04d" "$angle_int")
        else
            angle_tag=$(printf "ang_%04d" "$angle_int")
        fi
        angle_dir="${OUTPUT_ROOT}/${angle_tag}"
        mkdir -p "$angle_dir"
        log "  Output dir: $angle_dir"

        # ── 3. Run phase_sweep.py ─────────────────────────────────────────────
        log "  Starting phase sweep …"
        read -ra ps_args <<< "$(build_phase_sweep_args "$angle_dir")"
        "$PYTHON" "$PHASE_SWEEP_SCRIPT" "${ps_args[@]}"

        log "  Phase sweep complete for ${angle} °."
        log "────────────────────────────────────────"
    done

    log "All angles complete.  Results in: $(realpath "$OUTPUT_ROOT")"
}

main "$@"
