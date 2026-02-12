#!/usr/bin/env python3
"""
test_lidar.py - Serial Port Detection Utility

Scans and lists all available serial ports to help identify
the Youyeetoo WayPonDEV LD19 LiDAR sensor connected via USB.

The LD19 uses 230400 baud, one-way communication (sends data without commands).

Usage:
    python test_lidar.py

On macOS, the LiDAR typically appears as /dev/tty.usbserial-* or /dev/tty.usbmodem-*.
On Raspberry Pi (Linux), it typically appears as /dev/ttyUSB0 or /dev/ttyACM0.
"""

import sys
import glob
import serial
import serial.tools.list_ports


def list_serial_ports():
    """
    Detect and list all available serial ports with detailed information.
    Returns a list of port info tuples.
    """
    ports = serial.tools.list_ports.comports()

    if not ports:
        print("No serial ports detected.")
        print("\nTroubleshooting tips:")
        print("  1. Check that the LiDAR USB cable is connected")
        print("  2. Try a different USB port or cable")
        print("  3. On Linux, ensure your user is in the 'dialout' group:")
        print("     sudo usermod -a -G dialout $USER")
        print("  4. On macOS, check System Preferences > Security & Privacy")
        return []

    print(f"Found {len(ports)} serial port(s):\n")
    print("-" * 60)

    for port in sorted(ports):
        print(f"  Port:         {port.device}")
        print(f"  Description:  {port.description}")
        print(f"  Hardware ID:  {port.hwid}")

        # Print additional details if available
        if port.manufacturer:
            print(f"  Manufacturer: {port.manufacturer}")
        if port.product:
            print(f"  Product:      {port.product}")
        if port.serial_number:
            print(f"  Serial #:     {port.serial_number}")
        if port.vid is not None:
            print(f"  VID:PID:      {port.vid:04X}:{port.pid:04X}")

        print("-" * 60)

    return ports


def find_lidar_port():
    """
    Attempt to auto-detect the LiDAR port.

    Searches for common USB-serial device patterns:
      - macOS:  /dev/tty.usbserial-*, /dev/tty.usbmodem*
      - Linux:  /dev/ttyUSB*, /dev/ttyACM*

    Returns the port device string if found, or None.
    """
    ports = serial.tools.list_ports.comports()

    # Known patterns for USB-to-serial adapters commonly used with LiDAR
    lidar_patterns = []

    if sys.platform == "darwin":
        # macOS: glob for USB serial devices
        lidar_patterns = glob.glob("/dev/tty.usb*")
    elif sys.platform.startswith("linux"):
        # Linux (Raspberry Pi): common USB serial paths
        lidar_patterns = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")

    if lidar_patterns:
        print("\nPotential LiDAR ports (auto-detected):")
        for pattern_match in sorted(lidar_patterns):
            print(f"  -> {pattern_match}")
        return sorted(lidar_patterns)[0]

    # Fallback: look through pyserial's port list for USB devices
    for port in ports:
        if "USB" in port.description.upper() or "SERIAL" in port.description.upper():
            print(f"\nPotential LiDAR port (from description): {port.device}")
            return port.device

    print("\nCould not auto-detect LiDAR port.")
    return None


def test_port_connection(port_name, baudrate=230400):
    """
    Attempt to open a serial connection to the given port.
    This verifies the port is accessible and not locked by another process.
    """
    print(f"\nTesting connection to {port_name} at {baudrate} baud...")
    try:
        ser = serial.Serial(
            port=port_name,
            baudrate=baudrate,
            timeout=2
        )
        print(f"  Successfully opened {port_name}")
        print(f"  Baudrate: {ser.baudrate}")
        print(f"  Bytesize: {ser.bytesize}")
        print(f"  Parity:   {ser.parity}")
        print(f"  Stopbits: {ser.stopbits}")
        ser.close()
        print("  Connection closed cleanly.")
        return True
    except serial.SerialException as e:
        print(f"  Failed to open {port_name}: {e}")
        return False


def main():
    print("=" * 60)
    print("  LiDAR Serial Port Detection Utility")
    print("  Youyeetoo WayPonDEV LiDAR Sensor")
    print("=" * 60)
    print()

    # Step 1: List all ports
    ports = list_serial_ports()

    # Step 2: Try to auto-detect LiDAR
    lidar_port = find_lidar_port()

    # Step 3: Test connection if a candidate was found
    if lidar_port:
        test_port_connection(lidar_port)
        print(f"\nTo use this port with lidar_reader.py, run:")
        print(f"  python lidar_reader.py --port {lidar_port}")
    else:
        print("\nPlease connect the LiDAR and try again.")
        if ports:
            print("You can also manually specify a port from the list above:")
            print("  python lidar_reader.py --port /dev/YOUR_PORT")


if __name__ == "__main__":
    main()
