#!/usr/bin/env python3
"""
Raspberry Pi tuned LD19 LiDAR reader.

Compatible with the existing project API:
  - LidarReader.connect()
  - LidarReader.read_scans()
  - LidarReader.disconnect()
"""

import argparse
import glob
import os
import struct
import sys
import time

import serial


HEADER_BYTE = 0x54
VERLEN_BYTE = 0x2C
PACKET_SIZE = 47
POINTS_PER_PACKET = 12

CRC_TABLE = [
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
    0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
    0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
    0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
    0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
    0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
    0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
    0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
    0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
    0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
    0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8,
]


def calc_crc8(data):
    crc = 0
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


def auto_detect_port():
    env_port = os.environ.get("LIDAR_PORT")
    if env_port:
        return env_port

    if sys.platform.startswith("linux"):
        # Raspberry Pi-first order.
        candidates = (
            glob.glob("/dev/serial0")
            + glob.glob("/dev/ttyUSB*")
            + glob.glob("/dev/ttyACM*")
            + glob.glob("/dev/ttyAMA*")
        )
    elif sys.platform == "darwin":
        candidates = glob.glob("/dev/tty.usb*")
    else:
        candidates = []

    if candidates:
        port = sorted(candidates)[0]
        print(f"Auto-detected LiDAR port: {port}")
        return port

    return None


class LidarReader:
    def __init__(self, port=None, baudrate=230400, timeout=3):
        if port is None:
            port = auto_detect_port()
            if port is None:
                raise ConnectionError(
                    "No LiDAR port found. Set LIDAR_PORT or pass --port.\n"
                    "Run 'python test_lidar.py' to list available ports."
                )
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self._buffer = bytearray()

    def connect(self):
        print(f"Connecting to LD19 on {self.port} @ {self.baudrate} baud...")
        try:
            self.serial = serial.Serial(
                self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            self.serial.reset_input_buffer()
            time.sleep(0.4)
            test_data = self.serial.read(200)
            if len(test_data) < PACKET_SIZE:
                raise ConnectionError("No LiDAR data received")
            header_pos = test_data.find(bytes([HEADER_BYTE, VERLEN_BYTE]))
            if header_pos == -1:
                raise ConnectionError("No valid LD19 packet header found")
            print("LiDAR connected.")
            return True
        except serial.SerialException as exc:
            raise ConnectionError(f"Failed opening {self.port}: {exc}") from exc
        except Exception as exc:
            raise ConnectionError(f"Failed connecting {self.port}: {exc}") from exc

    def _read_packet(self):
        if self.serial.in_waiting:
            self._buffer.extend(self.serial.read(self.serial.in_waiting))

        if len(self._buffer) < PACKET_SIZE:
            self._buffer.extend(self.serial.read(PACKET_SIZE * 2))

        while len(self._buffer) >= PACKET_SIZE:
            if self._buffer[0] == HEADER_BYTE and self._buffer[1] == VERLEN_BYTE:
                packet = bytes(self._buffer[:PACKET_SIZE])
                if packet[46] == calc_crc8(packet[:46]):
                    speed = struct.unpack("<H", packet[2:4])[0]
                    start_angle = struct.unpack("<H", packet[4:6])[0] / 100.0
                    end_angle = struct.unpack("<H", packet[42:44])[0] / 100.0
                    timestamp = struct.unpack("<H", packet[44:46])[0]
                    points = []
                    for i in range(POINTS_PER_PACKET):
                        offset = 6 + i * 3
                        distance = struct.unpack("<H", packet[offset:offset + 2])[0]
                        intensity = packet[offset + 2]
                        if end_angle < start_angle:
                            angle_span = (360.0 - start_angle) + end_angle
                        else:
                            angle_span = end_angle - start_angle
                        angle = (start_angle + (angle_span * i / (POINTS_PER_PACKET - 1))) % 360.0
                        points.append({"angle": angle, "distance": distance, "intensity": intensity})

                    del self._buffer[:PACKET_SIZE]
                    return {
                        "speed": speed,
                        "start_angle": start_angle,
                        "end_angle": end_angle,
                        "timestamp": timestamp,
                        "points": points,
                    }
                del self._buffer[0]
            else:
                del self._buffer[0]
        return None

    def read_scans(self, max_scans=0, min_quality=0):
        if self.serial is None:
            raise RuntimeError("LiDAR not connected. Call connect() first.")

        scan_count = 0
        current_scan = []
        last_angle = 0.0

        while True:
            packet = self._read_packet()
            if packet is None:
                continue

            for point in packet["points"]:
                angle = point["angle"]
                distance = point["distance"]
                intensity = point["intensity"]
                if angle < last_angle - 180:
                    if current_scan:
                        if min_quality > 0:
                            current_scan = [(i, a, d) for i, a, d in current_scan if i >= min_quality]
                        scan_count += 1
                        yield current_scan
                        if max_scans > 0 and scan_count >= max_scans:
                            return
                    current_scan = []
                current_scan.append((intensity, angle, distance))
                last_angle = angle

    def disconnect(self):
        if self.serial is not None:
            try:
                self.serial.close()
            finally:
                self.serial = None


def parse_args():
    parser = argparse.ArgumentParser(description="Read LD19 scan data (Raspberry Pi profile).")
    parser.add_argument("--port", type=str, default=None, help="Serial port (or set LIDAR_PORT).")
    parser.add_argument("--scans", type=int, default=5, help="Number of scans (0 for unlimited).")
    parser.add_argument("--min-quality", type=int, default=0, help="Minimum intensity filter.")
    return parser.parse_args()


def main():
    args = parse_args()
    reader = LidarReader(port=args.port)
    try:
        reader.connect()
        for idx, scan in enumerate(reader.read_scans(max_scans=args.scans, min_quality=args.min_quality), start=1):
            distances = [d for _, _, d in scan if d > 0]
            if not distances:
                print(f"Scan {idx:>3}: {len(scan):>4} points | No valid distances")
                continue
            avg_dist = sum(distances) / len(distances)
            print(
                f"Scan {idx:>3}: {len(scan):>4} points | "
                f"Avg: {avg_dist:>7.1f} mm | Min: {min(distances):>7.1f} mm | Max: {max(distances):>7.1f} mm"
            )
    except ConnectionError as exc:
        print(f"Connection error: {exc}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        reader.disconnect()


if __name__ == "__main__":
    main()
