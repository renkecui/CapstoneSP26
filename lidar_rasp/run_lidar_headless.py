#!/usr/bin/env python3
"""
Headless LD19 runtime for Raspberry Pi.

Useful for SSH/systemd usage where GUI plotting is unavailable.
"""

import argparse
import signal
import sys
import time

from lidar_reader import LidarReader


RUNNING = True


def _handle_stop(_signum, _frame):
    global RUNNING
    RUNNING = False


def parse_args():
    parser = argparse.ArgumentParser(description="Headless LD19 scanner for Raspberry Pi.")
    parser.add_argument("--port", type=str, default=None, help="Serial port (or set LIDAR_PORT).")
    parser.add_argument("--min-quality", type=int, default=0, help="Minimum intensity to keep.")
    parser.add_argument("--log-every", type=int, default=10, help="Print every N scans.")
    return parser.parse_args()


def main():
    args = parse_args()
    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    reader = LidarReader(port=args.port)

    try:
        reader.connect()
        print("Headless LiDAR loop started.")
        print("Press Ctrl+C to stop.")
        scan_counter = 0
        for scan in reader.read_scans(max_scans=0, min_quality=args.min_quality):
            if not RUNNING:
                break
            scan_counter += 1
            if scan_counter % max(1, args.log_every) != 0:
                continue

            valid = [distance for _, _, distance in scan if distance > 0]
            if not valid:
                print(f"[scan {scan_counter}] points={len(scan)} valid=0")
                continue

            avg_dist = sum(valid) / len(valid)
            print(
                f"[scan {scan_counter}] points={len(scan)} valid={len(valid)} "
                f"min={min(valid):.0f}mm avg={avg_dist:.0f}mm max={max(valid):.0f}mm"
            )

            # Small sleep prevents flooding if log-every is low.
            time.sleep(0.01)

    except ConnectionError as exc:
        print(f"Connection error: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        reader.disconnect()
        print("Headless LiDAR loop stopped.")


if __name__ == "__main__":
    main()
