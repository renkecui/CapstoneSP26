#!/usr/bin/env python3
"""
lidar_reader.py - Youyeetoo WayPonDEV LD19 LiDAR Reader

Main class for connecting to and reading scan data from the LD19 LiDAR sensor.
This LiDAR uses a one-way communication protocol at 230400 baud with 47-byte packets.

Packet format:
  - Header (1 byte): 0x54
  - VerLen (1 byte): 0x2C (12 points per packet)
  - Speed (2 bytes): Rotation speed in degrees/sec
  - Start Angle (2 bytes): Start angle in 0.01 degrees
  - Data (36 bytes): 12 points x 3 bytes (2 bytes distance + 1 byte intensity)
  - End Angle (2 bytes): End angle in 0.01 degrees
  - Timestamp (2 bytes): Timestamp in milliseconds
  - CRC (1 byte): CRC checksum

Usage:
    python lidar_reader.py
    python lidar_reader.py --port /dev/ttyUSB0
    python lidar_reader.py --port /dev/tty.usbserial-0001
"""

import sys
import glob
import time
import struct
import argparse
import serial


# LD19 Protocol constants
HEADER_BYTE = 0x54
VERLEN_BYTE = 0x2C
PACKET_SIZE = 47
POINTS_PER_PACKET = 12

# CRC8 lookup table for LD19 (Poly: 0x4D)
CRC_TABLE = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
    0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
    0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
    0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
    0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
    0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
    0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
    0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
    0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
    0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
    0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
    0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
]


def calc_crc8(data):
    """Calculate CRC8 checksum for LD19 protocol."""
    crc = 0
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


def auto_detect_port():
    """
    Auto-detect the LiDAR serial port based on the OS.

    Returns:
        str or None: The detected port path, or None if not found.
    """
    if sys.platform == "darwin":
        # macOS: USB serial devices appear under /dev/tty.usb*
        candidates = glob.glob("/dev/tty.usb*")
    elif sys.platform.startswith("linux"):
        # Raspberry Pi / Linux: typically /dev/ttyUSB0 or /dev/ttyACM0
        candidates = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    elif sys.platform == "win32":
        # Windows: COM ports (user should specify manually)
        candidates = []
    else:
        candidates = []

    if candidates:
        port = sorted(candidates)[0]
        print(f"Auto-detected LiDAR port: {port}")
        return port

    return None


class LidarReader:
    """
    Handles connection, data reading, and lifecycle management
    for the Youyeetoo WayPonDEV LD19 LiDAR sensor.

    Attributes:
        port (str): Serial port path (e.g., /dev/ttyUSB0).
        serial (serial.Serial): The serial connection instance.
    """

    def __init__(self, port=None, baudrate=230400, timeout=3):
        """
        Initialize the LiDAR reader.

        Args:
            port (str, optional): Serial port. If None, auto-detect is attempted.
            baudrate (int): Serial baudrate (default 230400 for LD19).
            timeout (int): Serial read timeout in seconds.
        """
        # Auto-detect port if not provided
        if port is None:
            port = auto_detect_port()
            if port is None:
                raise ConnectionError(
                    "No LiDAR port found. Please specify one with --port.\n"
                    "Run 'python test_lidar.py' to list available ports."
                )

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self._buffer = bytearray()

    def connect(self):
        """
        Establish a connection to the LiDAR sensor.

        Raises:
            ConnectionError: If the connection fails.
        """
        print(f"Connecting to LD19 LiDAR on {self.port} at {self.baudrate} baud...")
        try:
            self.serial = serial.Serial(
                self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            # Clear any stale data
            self.serial.reset_input_buffer()
            time.sleep(0.5)

            # Verify we're receiving valid packets
            print("Verifying LiDAR data stream...")
            test_data = self.serial.read(200)
            if len(test_data) < PACKET_SIZE:
                raise ConnectionError("No data received from LiDAR")

            # Find a valid packet
            header_pos = test_data.find(bytes([HEADER_BYTE, VERLEN_BYTE]))
            if header_pos == -1:
                raise ConnectionError("No valid LD19 packets found in data stream")

            print("LiDAR connected successfully!")
            print(f"  Protocol:    LD19 (one-way communication)")
            print(f"  Baudrate:    {self.baudrate}")
            print(f"  Packet size: {PACKET_SIZE} bytes")
            print(f"  Points/pkt:  {POINTS_PER_PACKET}")
            return True

        except serial.SerialException as e:
            raise ConnectionError(f"Failed to open serial port {self.port}: {e}")
        except Exception as e:
            raise ConnectionError(f"Failed to connect to LiDAR on {self.port}: {e}")

    def _read_packet(self):
        """
        Read and parse a single LD19 packet from the serial stream.

        Returns:
            dict or None: Parsed packet data, or None if no valid packet found.
        """
        # Read more data into buffer
        if self.serial.in_waiting:
            self._buffer.extend(self.serial.read(self.serial.in_waiting))

        # Need at least one full packet
        if len(self._buffer) < PACKET_SIZE:
            more_data = self.serial.read(PACKET_SIZE * 2)
            self._buffer.extend(more_data)

        # Find packet header
        while len(self._buffer) >= PACKET_SIZE:
            # Look for header bytes
            if self._buffer[0] == HEADER_BYTE and self._buffer[1] == VERLEN_BYTE:
                packet = bytes(self._buffer[:PACKET_SIZE])

                # Verify CRC
                expected_crc = packet[46]
                calculated_crc = calc_crc8(packet[:46])

                if expected_crc == calculated_crc:
                    # Parse packet
                    speed = struct.unpack('<H', packet[2:4])[0]  # degrees/sec
                    start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0  # degrees
                    end_angle = struct.unpack('<H', packet[42:44])[0] / 100.0  # degrees
                    timestamp = struct.unpack('<H', packet[44:46])[0]  # ms

                    # Parse 12 measurement points
                    points = []
                    for i in range(POINTS_PER_PACKET):
                        offset = 6 + i * 3
                        distance = struct.unpack('<H', packet[offset:offset+2])[0]  # mm
                        intensity = packet[offset + 2]  # 0-255

                        # Calculate angle for this point (linear interpolation)
                        if end_angle < start_angle:
                            # Handle wraparound at 360 degrees
                            angle_span = (360.0 - start_angle) + end_angle
                        else:
                            angle_span = end_angle - start_angle

                        angle = (start_angle + (angle_span * i / (POINTS_PER_PACKET - 1))) % 360.0

                        points.append({
                            'angle': angle,
                            'distance': distance,
                            'intensity': intensity
                        })

                    # Remove packet from buffer
                    del self._buffer[:PACKET_SIZE]

                    return {
                        'speed': speed,
                        'start_angle': start_angle,
                        'end_angle': end_angle,
                        'timestamp': timestamp,
                        'points': points
                    }
                else:
                    # CRC mismatch, skip this byte
                    del self._buffer[0]
            else:
                # Not a header, skip this byte
                del self._buffer[0]

        return None

    def read_scans(self, max_scans=0, min_quality=0):
        """
        Generator that yields complete 360-degree scans.

        Each scan is a list of (intensity, angle, distance) tuples:
          - intensity (int): Measurement intensity (0-255)
          - angle (float): Angle in degrees (0-360)
          - distance (float): Distance in millimeters (0 if invalid)

        Args:
            max_scans (int): Maximum number of scans to read (0 = unlimited).
            min_quality (int): Minimum intensity threshold to include a measurement.

        Yields:
            list: A scan as a list of (intensity, angle, distance) tuples.
        """
        if self.serial is None:
            raise RuntimeError("LiDAR not connected. Call connect() first.")

        scan_count = 0
        current_scan = []
        last_angle = 0.0

        while True:
            packet = self._read_packet()
            if packet is None:
                continue

            for point in packet['points']:
                angle = point['angle']
                distance = point['distance']
                intensity = point['intensity']

                # Detect new scan (angle wrapped around)
                if angle < last_angle - 180:
                    # We've completed a full rotation
                    if current_scan:
                        # Filter by intensity if threshold is set
                        if min_quality > 0:
                            current_scan = [(i, a, d) for i, a, d in current_scan if i >= min_quality]

                        scan_count += 1
                        yield current_scan

                        if max_scans > 0 and scan_count >= max_scans:
                            return

                    current_scan = []

                current_scan.append((intensity, angle, distance))
                last_angle = angle

    def print_scan(self, scan, scan_number=None):
        """
        Pretty-print a single scan's data.

        Args:
            scan (list): List of (intensity, angle, distance) tuples.
            scan_number (int, optional): The scan index for display.
        """
        header = f"Scan #{scan_number}" if scan_number else "Scan"
        print(f"\n{'=' * 50}")
        print(f"  {header} - {len(scan)} measurements")
        print(f"{'=' * 50}")
        print(f"  {'Intensity':>9}  {'Angle (°)':>10}  {'Distance (mm)':>14}")
        print(f"  {'-' * 9}  {'-' * 10}  {'-' * 14}")

        for intensity, angle, distance in scan:
            print(f"  {intensity:>9}  {angle:>10.2f}  {distance:>14.1f}")

    def disconnect(self):
        """
        Safely disconnect from the LiDAR.
        """
        if self.serial is not None:
            try:
                print("\nDisconnecting from LiDAR...")
                self.serial.close()
                print("LiDAR disconnected cleanly.")
            except Exception as e:
                print(f"Warning during disconnect: {e}")
            finally:
                self.serial = None


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Read scan data from a Youyeetoo WayPonDEV LD19 LiDAR sensor."
    )
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Serial port (e.g., /dev/ttyUSB0). Auto-detects if not specified."
    )
    parser.add_argument(
        "--scans",
        type=int,
        default=5,
        help="Number of scans to read (0 for unlimited, default: 5)."
    )
    parser.add_argument(
        "--min-quality",
        type=int,
        default=0,
        help="Minimum measurement intensity to display (0-255, default: 0)."
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print every measurement point (can be very verbose)."
    )
    return parser.parse_args()


def main():
    args = parse_args()
    reader = LidarReader(port=args.port)

    try:
        reader.connect()
        print(f"\nReading {args.scans if args.scans > 0 else 'unlimited'} scan(s)...\n")

        for i, scan in enumerate(reader.read_scans(
            max_scans=args.scans,
            min_quality=args.min_quality
        ), start=1):
            if args.verbose:
                # Print full scan details
                reader.print_scan(scan, scan_number=i)
            else:
                # Print a compact summary
                distances = [d for _, _, d in scan if d > 0]
                if distances:
                    avg_dist = sum(distances) / len(distances)
                    min_dist = min(distances)
                    max_dist = max(distances)
                    print(
                        f"Scan {i:>3}: {len(scan):>4} points | "
                        f"Avg: {avg_dist:>7.1f} mm | "
                        f"Min: {min_dist:>7.1f} mm | "
                        f"Max: {max_dist:>7.1f} mm"
                    )
                else:
                    print(f"Scan {i:>3}: {len(scan):>4} points | No valid distances")

        print("\nDone reading scans.")

    except ConnectionError as e:
        print(f"\nConnection error: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    except Exception as e:
        print(f"\nUnexpected error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        # Always clean up
        reader.disconnect()


if __name__ == "__main__":
    main()
