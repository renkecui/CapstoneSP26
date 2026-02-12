#!/usr/bin/env python3
"""
visualize_lidar.py - Real-Time LD19 LiDAR Scan Visualization

Displays a live polar scatter plot of LiDAR scan data from the
Youyeetoo WayPonDEV LD19 LiDAR sensor using matplotlib.

Uses a background thread to read scans so the GUI stays responsive.

Usage:
    python3 visualize_lidar.py
    python3 visualize_lidar.py --port /dev/tty.usbserial-0001
    python3 visualize_lidar.py --max-distance 3000

Controls:
    - Close the plot window or press Ctrl+C to stop.
"""

import sys
import math
import argparse
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from lidar_reader import LidarReader


class LidarVisualizer:
    """
    Real-time polar plot visualization for LD19 LiDAR scan data.

    A background thread continuously reads scans from the sensor and
    stores the latest one. The matplotlib animation loop renders
    whatever scan data is available, keeping the GUI responsive.
    """

    def __init__(self, port=None, max_distance=6000, min_quality=0):
        """
        Initialize the visualizer.

        Args:
            port (str, optional): Serial port. Auto-detects if None.
            max_distance (float): Maximum distance in mm for the plot radius.
            min_quality (int): Minimum intensity threshold (0-255).
        """
        self.port = port
        self.max_distance = max_distance
        self.min_quality = min_quality
        self.reader = None

        # Thread-safe scan data shared between reader thread and GUI
        self._lock = threading.Lock()
        self._latest_scan = None   # Most recent complete scan
        self._scan_count = 0       # Total scans received
        self._running = False      # Controls the reader thread

    def connect(self):
        """Connect to the LiDAR sensor via LidarReader."""
        self.reader = LidarReader(port=self.port)
        self.reader.connect()

    def _reader_thread(self):
        """
        Background thread: continuously reads scans and stores the latest.
        Runs until self._running is set to False.
        """
        try:
            for scan in self.reader.read_scans(max_scans=0):
                if not self._running:
                    break

                # Filter by intensity threshold and remove zero-distance points
                filtered = [
                    (intensity, angle, distance)
                    for intensity, angle, distance in scan
                    if intensity >= self.min_quality and distance > 0
                ]

                # Store the latest scan (thread-safe)
                with self._lock:
                    self._latest_scan = filtered
                    self._scan_count += 1

        except Exception as e:
            if self._running:
                print(f"\nReader thread error: {e}")
        finally:
            self._running = False

    def _setup_plot(self):
        """
        Create the polar plot figure with styling suited for LiDAR data.

        Returns:
            tuple: (fig, ax, scatter_artist)
        """
        plt.style.use("dark_background")

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection="polar")

        # -- Polar axis configuration --
        # 0 degrees at top = forward direction of the robot
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)            # Clockwise
        ax.set_ylim(0, self.max_distance)

        # Distance ring labels
        ring_count = 5
        ring_distances = np.linspace(0, self.max_distance, ring_count + 1)[1:]
        ax.set_rgrids(
            ring_distances,
            labels=[f"{int(d / 1000 * 100) / 100:.1f}m" if d >= 1000 else f"{int(d)}mm"
                    for d in ring_distances],
            fontsize=8,
            alpha=0.5,
        )
        ax.set_rlabel_position(135)

        # Title
        ax.set_title(
            "LD19 LiDAR - Waiting for data...",
            fontsize=14,
            pad=20,
            color="white",
        )

        # Create empty scatter — intensity mapped to color (0-255 for LD19)
        scatter = ax.scatter(
            [], [], s=5, c=[], cmap="plasma",
            vmin=0, vmax=255, alpha=0.85,
        )

        # Colorbar
        cbar = fig.colorbar(scatter, ax=ax, pad=0.08, shrink=0.75, aspect=30)
        cbar.set_label("Intensity", fontsize=10)

        # Mark the sensor origin
        ax.plot(0, 0, marker="o", color="red", markersize=8, zorder=5)
        ax.annotate(
            "LIDAR", xy=(0, 0),
            xytext=(0.15, self.max_distance * 0.08),
            fontsize=7, color="red", ha="center",
        )

        fig.tight_layout()
        return fig, ax, scatter

    def _update_frame(self, frame):
        """
        Animation callback — grab the latest scan data and redraw.
        This never blocks because the heavy serial I/O runs in a thread.
        """
        with self._lock:
            scan = self._latest_scan
            count = self._scan_count

        if scan is None or len(scan) == 0:
            return (self._scatter,)

        intensities, angles_deg, distances = zip(*scan)

        # Convert degrees -> radians for polar plot
        angles_rad = np.array([math.radians(a) for a in angles_deg])
        dists = np.array(distances)
        intens = np.array(intensities)

        # Update the scatter plot
        self._scatter.set_offsets(np.column_stack((angles_rad, dists)))
        self._scatter.set_array(intens)

        # Update title with live stats
        valid = dists[dists > 0]
        if len(valid) > 0:
            self._ax.set_title(
                f"LD19 LiDAR  |  {len(scan)} pts  |  "
                f"min {valid.min():.0f} mm  |  max {valid.max():.0f} mm  |  "
                f"scan #{count}",
                fontsize=12, pad=20, color="white",
            )

        return (self._scatter,)

    def run(self):
        """
        Start the real-time visualization.

        1. Connects to the LiDAR.
        2. Launches a background thread to continuously read scans.
        3. Opens the matplotlib window with an animation loop.
        4. Cleans up on close or Ctrl+C.
        """
        try:
            self.connect()

            # Set up plot
            fig, self._ax, self._scatter = self._setup_plot()

            # Start the background reader thread
            self._running = True
            thread = threading.Thread(target=self._reader_thread, daemon=True)
            thread.start()

            # Animate at ~15 FPS (interval=66ms)
            self._ani = FuncAnimation(
                fig,
                self._update_frame,
                interval=66,
                blit=False,
                cache_frame_data=False,
            )

            print("\nVisualization running. Close the window or press Ctrl+C to stop.")
            plt.show()

        except ConnectionError as e:
            print(f"\nConnection error: {e}", file=sys.stderr)
            sys.exit(1)
        except KeyboardInterrupt:
            print("\n\nInterrupted by user.")
        except Exception as e:
            print(f"\nUnexpected error: {e}", file=sys.stderr)
            sys.exit(1)
        finally:
            self.shutdown()

    def shutdown(self):
        """Stop the reader thread, disconnect LiDAR, close plot."""
        self._running = False

        if self.reader is not None:
            self.reader.disconnect()
            self.reader = None

        plt.close("all")


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Real-time LD19 LiDAR scan visualization (polar plot)."
    )
    parser.add_argument(
        "--port", type=str, default=None,
        help="Serial port (e.g., /dev/tty.usbserial-0001). Auto-detects if omitted."
    )
    parser.add_argument(
        "--max-distance", type=float, default=4000,
        help="Maximum plot radius in mm (default: 4000)."
    )
    parser.add_argument(
        "--min-quality", type=int, default=0,
        help="Minimum measurement intensity to display (0-255, default: 0)."
    )
    return parser.parse_args()


def main():
    args = parse_args()
    viz = LidarVisualizer(
        port=args.port,
        max_distance=args.max_distance,
        min_quality=args.min_quality,
    )
    viz.run()


if __name__ == "__main__":
    main()
