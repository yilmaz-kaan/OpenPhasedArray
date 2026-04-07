"""
turn_table_test.py
──────────────────────────────────────────────────────────────────────────────
Controls the turntable via TCP/IP socket.  All connection parameters and the
target position are configurable from the command line so the script can be
driven by an automated sweep wrapper (e.g. antenna_sweep.sh).

Usage
─────
    # Interactive: ask the user for a position
    python turn_table_test.py

    # Non-interactive: go directly to 90 °
    python turn_table_test.py --position 90

    # Override connection defaults
    python turn_table_test.py --ip 192.168.1.10 --port 200 --position 45

    # Adjust motion speed (°/s) and command delay
    python turn_table_test.py --position 180 --speed 5.0 --cmd-delay 0.15
"""

import argparse
import socket
import time
import datetime


# ═══════════════════════════════════════════════════════════════════════════════
# DEFAULTS  –  override via CLI flags or environment
# ═══════════════════════════════════════════════════════════════════════════════

DEFAULT_IP        = "172.16.1.186"
DEFAULT_PORT      = 200
DEFAULT_BUFFER    = 128
DEFAULT_CMD_DELAY = 0.1   # seconds between successive commands
DEFAULT_SPEED     = 3.0   # °/s


# ═══════════════════════════════════════════════════════════════════════════════
# SOCKET HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

def Write(cmd: str, *, ip: str, port: int, cmd_delay: float) -> None:
    """Send *cmd* via a new TCP connection (fire-and-forget)."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ip, port))
        s.send(cmd.encode())
        print(datetime.datetime.now().time(), "Sent: ", cmd)
        time.sleep(cmd_delay)


def Query(cmd: str, *, ip: str, port: int, buffer: int, cmd_delay: float) -> str:
    """Send *cmd* and return the device's reply (null-terminator stripped)."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ip, port))
        s.send(cmd.encode())
        data  = s.recv(buffer)
        value = data.decode("utf-8")[:-1]   # strip trailing null
        print(datetime.datetime.now().time(), "Sent: ", cmd)
        print(datetime.datetime.now().time(), "Recv: ", value)
        time.sleep(cmd_delay)
        return value


# ═══════════════════════════════════════════════════════════════════════════════
# TURNTABLE MOTION
# ═══════════════════════════════════════════════════════════════════════════════

def goto_position(
    target_deg: float,
    *,
    ip:        str   = DEFAULT_IP,
    port:      int   = DEFAULT_PORT,
    buffer:    int   = DEFAULT_BUFFER,
    cmd_delay: float = DEFAULT_CMD_DELAY,
    speed:     float = DEFAULT_SPEED,
    poll_interval: float = 0.5,
) -> None:
    """
    Rotate the turntable to *target_deg* degrees and block until motion stops.

    Parameters
    ──────────
    target_deg     Absolute target angle in degrees.
    ip             Turntable controller IP address.
    port           TCP port.
    buffer         Receive buffer size (bytes).
    cmd_delay      Delay after every Write command (seconds).
    speed          Rotation speed in °/s.
    poll_interval  How often to poll BU during motion (seconds).
    """
    kw_w = dict(ip=ip, port=port, cmd_delay=cmd_delay)
    kw_q = dict(ip=ip, port=port, buffer=buffer, cmd_delay=cmd_delay)

    # Identify device
    Query("*IDN?", **kw_q)

    # Select turntable (device 1) and report current position
    Write("LD 1 DV", **kw_w)
    Query("S?",  **kw_q)

    # Set speed (°/s encoded as a float string)
    Write(f"LD {speed:.4f} SF", **kw_w)

    Query("RP", **kw_q)

    # Build and send the go-to command
    cmd = f"LD {target_deg} DG NP GO"
    Write(cmd, **kw_w)

    # Wait for motion to start (BU → "1")
    while True:
        if Query("BU", **kw_q) == "1":
            break
        time.sleep(poll_interval)

    # Wait for motion to finish (BU → "0"), reporting position along the way
    while True:
        bu = Query("BU", **kw_q)
        Query("RP", **kw_q)
        if bu == "0":
            break
        time.sleep(poll_interval)

    print(datetime.datetime.now().time(), f"Arrived at {target_deg} °")


# ═══════════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move the turntable to a specified angle.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--position", "-p", type=float, default=None,
        metavar="DEG",
        help="Target angle in degrees.  Omit to be prompted interactively.",
    )
    parser.add_argument(
        "--ip", type=str, default=DEFAULT_IP,
        help="Turntable controller IP address.",
    )
    parser.add_argument(
        "--port", type=int, default=DEFAULT_PORT,
        help="TCP port of the turntable controller.",
    )
    parser.add_argument(
        "--buffer", type=int, default=DEFAULT_BUFFER,
        help="Receive buffer size in bytes.",
    )
    parser.add_argument(
        "--cmd-delay", type=float, default=DEFAULT_CMD_DELAY,
        metavar="SEC",
        help="Delay after each Write command to avoid dropped commands.",
    )
    parser.add_argument(
        "--speed", type=float, default=DEFAULT_SPEED,
        metavar="DEG_PER_SEC",
        help="Turntable rotation speed in °/s.",
    )
    parser.add_argument(
        "--poll-interval", type=float, default=0.5,
        metavar="SEC",
        help="How often to poll the busy flag during motion.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    position = args.position
    if position is None:
        position = float(input("Enter new target position (degrees): "))

    goto_position(
        target_deg    = position,
        ip            = args.ip,
        port          = args.port,
        buffer        = args.buffer,
        cmd_delay     = args.cmd_delay,
        speed         = args.speed,
        poll_interval = args.poll_interval,
    )


if __name__ == "__main__":
    main()