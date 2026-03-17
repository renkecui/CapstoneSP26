#!/usr/bin/env python3
"""
Raspberry Pi serial detection helper for LD19.
"""

import glob
import os
import sys

import serial
import serial.tools.list_ports


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports detected.")
        print("Troubleshooting:")
        print("  1) Verify LiDAR USB/power cable")
        print("  2) Try another USB cable/port")
        print("  3) Ensure user is in dialout group on Linux:")
        print("     sudo usermod -a -G dialout $USER")
        return []

    print(f"Found {len(ports)} serial port(s):\n")
    for port in sorted(ports):
        print(f"Port:         {port.device}")
        print(f"Description:  {port.description}")
        print(f"Hardware ID:  {port.hwid}")
        if port.manufacturer:
            print(f"Manufacturer: {port.manufacturer}")
        if port.product:
            print(f"Product:      {port.product}")
        if port.serial_number:
            print(f"Serial #:     {port.serial_number}")
        if port.vid is not None:
            print(f"VID:PID:      {port.vid:04X}:{port.pid:04X}")
        print("-" * 60)
    return ports


def find_lidar_port():
    env_port = os.environ.get("LIDAR_PORT")
    if env_port:
        print(f"LIDAR_PORT set: {env_port}")
        return env_port

    if sys.platform.startswith("linux"):
        patterns = (
            glob.glob("/dev/serial0")
            + glob.glob("/dev/ttyUSB*")
            + glob.glob("/dev/ttyACM*")
            + glob.glob("/dev/ttyAMA*")
        )
    elif sys.platform == "darwin":
        patterns = glob.glob("/dev/tty.usb*")
    else:
        patterns = []

    if patterns:
        print("Potential LiDAR ports:")
        for entry in sorted(patterns):
            print(f"  -> {entry}")
        return sorted(patterns)[0]

    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "USB" in port.description.upper() or "SERIAL" in port.description.upper():
            print(f"Candidate from description: {port.device}")
            return port.device

    print("Could not auto-detect LiDAR port.")
    return None


def test_port_connection(port_name, baudrate=230400):
    print(f"\nTesting {port_name} @ {baudrate}...")
    try:
        ser = serial.Serial(port=port_name, baudrate=baudrate, timeout=2)
        print("  Opened successfully")
        ser.close()
        return True
    except serial.SerialException as exc:
        print(f"  Failed to open: {exc}")
        return False


def main():
    print("=" * 60)
    print("LD19 Serial Port Detection (Raspberry Pi)")
    print("=" * 60)
    print()
    ports = list_serial_ports()
    lidar_port = find_lidar_port()
    if lidar_port:
        test_port_connection(lidar_port)
        print("\nUse this with lidar_reader.py:")
        print(f"  python lidar_reader.py --port {lidar_port}")
    elif ports:
        print("\nManually choose one of the listed ports.")


if __name__ == "__main__":
    main()
