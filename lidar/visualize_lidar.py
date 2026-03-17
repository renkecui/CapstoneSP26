#!/usr/bin/env python3
"""
visualize_lidar.py - Real-Time LD19 LiDAR Scan Visualization with Clearance Detection

Displays a live polar scatter plot of LiDAR scan data from the
Youyeetoo WayPonDEV LD19 LiDAR sensor using matplotlib.

Features:
    - Real-time obstacle detection divided into configurable sectors
    - Visual overlay showing clear (green) vs blocked (red) areas
    - Clearance threshold ring indicating safe distance
    - Navigation advice based on sector clearance status
    - Callback support for integration with other systems (e.g., motor control)

Uses a background thread to read scans so the GUI stays responsive.

Usage:
    python3 visualize_lidar.py
    python3 visualize_lidar.py --port /dev/tty.usbserial-0001
    python3 visualize_lidar.py --clearance-threshold 500 --sectors 8
    python3 visualize_lidar.py --print-clearance

Options:
    --clearance-threshold  Distance in mm for obstacle detection (default: 1000)
    --sectors              Number of sectors: 4, 8, or 12 (default: 8)
    --no-sectors           Hide the sector overlay
    --print-clearance      Print clearance info to console

Controls:
    - Close the plot window or press Ctrl+C to stop.

Programmatic Integration:
    from visualize_lidar import LidarVisualizer, ClearanceInfo

    def on_clearance_update(clearance: ClearanceInfo):
        advice = clearance.get_navigation_advice()
        if not advice["can_go_forward"]:
            print(f"Obstacle ahead! Try: {advice['suggested_direction']}")

    viz = LidarVisualizer(
        clearance_threshold=800,
        clearance_callback=on_clearance_update
    )
    viz.run()
"""

import sys
import math
import argparse
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from lidar_reader import LidarReader


class ClearanceInfo:
    """
    Holds clearance information for all sectors.

    Attributes:
        sector_count (int): Number of sectors (e.g., 8 for 45° each)
        threshold (float): Distance threshold in mm for "clear" classification
        sectors (list): List of dicts with sector info:
            - start_angle: Start angle in degrees
            - end_angle: End angle in degrees
            - min_distance: Minimum obstacle distance in this sector (mm)
            - is_clear: True if min_distance > threshold
            - direction: Human-readable direction name
    """

    DIRECTION_NAMES = {
        8: ["Forward", "Front-Right", "Right", "Rear-Right",
            "Rear", "Rear-Left", "Left", "Front-Left"],
        12: ["Forward", "FWD-Right", "Front-Right", "Right-Front",
             "Right", "Right-Rear", "Rear-Right", "RWD-Left",
             "Rear-Left", "Left-Rear", "Left", "Left-Front"],
        4: ["Forward", "Right", "Rear", "Left"],
    }

    def __init__(self, sector_count=8, threshold=1000):
        self.sector_count = sector_count
        self.threshold = threshold
        self.sectors = []
        self._init_sectors()

    def _init_sectors(self):
        """Initialize sector definitions."""
        sector_size = 360 / self.sector_count
        # Get direction names, or generate generic ones
        names = self.DIRECTION_NAMES.get(
            self.sector_count,
            [f"Sector {i}" for i in range(self.sector_count)]
        )

        self.sectors = []
        for i in range(self.sector_count):
            # Center sectors around forward (0°)
            # Sector 0 is centered at 0° (forward)
            start = (i * sector_size - sector_size / 2) % 360
            end = (start + sector_size) % 360
            self.sectors.append({
                "index": i,
                "start_angle": start,
                "end_angle": end,
                "min_distance": float('inf'),
                "is_clear": True,
                "direction": names[i],
            })

    def update_from_scan(self, scan_data):
        """
        Update clearance info from a LiDAR scan.

        Args:
            scan_data: List of (intensity, angle, distance) tuples
        """
        sector_size = 360 / self.sector_count

        # Reset distances
        for sector in self.sectors:
            sector["min_distance"] = float('inf')

        # Assign each point to its sector and track minimum distance
        for _intensity, angle, distance in scan_data:
            if distance <= 0:
                continue

            # Determine which sector this angle belongs to
            # Offset by half-sector so sector 0 is centered at 0°
            adjusted_angle = (angle + sector_size / 2) % 360
            sector_idx = int(adjusted_angle / sector_size) % self.sector_count

            if distance < self.sectors[sector_idx]["min_distance"]:
                self.sectors[sector_idx]["min_distance"] = distance

        # Update is_clear status
        for sector in self.sectors:
            if sector["min_distance"] == float('inf'):
                # No readings in this sector - treat as unknown/blocked for safety
                sector["is_clear"] = False
                sector["min_distance"] = 0
            else:
                sector["is_clear"] = sector["min_distance"] > self.threshold

    def get_clear_directions(self):
        """Return list of direction names that are clear."""
        return [s["direction"] for s in self.sectors if s["is_clear"]]

    def get_blocked_directions(self):
        """Return list of direction names that are blocked."""
        return [s["direction"] for s in self.sectors if not s["is_clear"]]

    def get_navigation_advice(self):
        """
        Return simple navigation advice based on clearance.

        Returns:
            dict with keys:
                - can_go_forward: bool
                - suggested_direction: str or None
                - clear_directions: list of str
                - blocked_directions: list of str
        """
        clear = self.get_clear_directions()
        blocked = self.get_blocked_directions()
        can_forward = self.sectors[0]["is_clear"] if self.sectors else False

        # Suggest a direction if forward is blocked
        suggested = None
        if not can_forward and clear:
            # Prefer front-left or front-right if available
            for preferred in ["Front-Left", "Front-Right", "Left", "Right"]:
                if preferred in clear:
                    suggested = preferred
                    break
            if not suggested:
                suggested = clear[0]

        return {
            "can_go_forward": can_forward,
            "suggested_direction": suggested,
            "clear_directions": clear,
            "blocked_directions": blocked,
        }

    def __str__(self):
        """Human-readable summary."""
        lines = [f"Clearance ({self.threshold}mm threshold):"]
        for s in self.sectors:
            status = "CLEAR" if s["is_clear"] else "BLOCKED"
            dist = f"{s['min_distance']:.0f}mm" if s['min_distance'] < float('inf') else "N/A"
            lines.append(f"  {s['direction']:12} : {status:7} (min: {dist})")
        return "\n".join(lines)


class LidarVisualizer:
    """
    Real-time polar plot visualization for LD19 LiDAR scan data.

    A background thread continuously reads scans from the sensor and
    stores the latest one. The matplotlib animation loop renders
    whatever scan data is available, keeping the GUI responsive.
    """

    def __init__(self, port=None, max_distance=6000, min_quality=0,
                 clearance_threshold=1000, sector_count=8,
                 show_sectors=True, clearance_callback=None,
                 print_clearance=False):
        """
        Initialize the visualizer.

        Args:
            port (str, optional): Serial port. Auto-detects if None.
            max_distance (float): Maximum distance in mm for the plot radius.
            min_quality (int): Minimum intensity threshold (0-255).
            clearance_threshold (float): Distance in mm below which a sector is
                considered blocked (default 1000mm = 1m).
            sector_count (int): Number of sectors to divide the view into
                (default 8 = 45° per sector).
            show_sectors (bool): Whether to show colored sector overlays.
            clearance_callback (callable): Optional function called each frame with
                ClearanceInfo object. Use this to integrate with other systems.
            print_clearance (bool): Whether to print clearance info to console.
        """
        self.port = port
        self.max_distance = max_distance
        self.min_quality = min_quality
        self.reader = None

        # Clearance detection settings
        self.clearance_threshold = clearance_threshold
        self.sector_count = sector_count
        self.show_sectors = show_sectors
        self.clearance_callback = clearance_callback
        self.print_clearance = print_clearance

        # Clearance info object
        self.clearance = ClearanceInfo(
            sector_count=sector_count,
            threshold=clearance_threshold
        )

        # Thread-safe scan data shared between reader thread and GUI
        self._lock = threading.Lock()
        self._latest_scan = None   # Most recent complete scan
        self._scan_count = 0       # Total scans received
        self._running = False      # Controls the reader thread
        self._sector_patches = []  # Matplotlib wedge patches for sectors

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

        # -- Create sector wedges for clearance visualization --
        if self.show_sectors:
            self._sector_patches = []
            sector_size = 360 / self.sector_count

            for sector in self.clearance.sectors:
                # Calculate angles for the wedge
                # Matplotlib polar uses radians, 0 at right, counter-clockwise
                # But we set theta_zero_location("N") and direction(-1)
                # So we need to convert our angles appropriately
                start_angle = sector["start_angle"]
                end_angle = sector["end_angle"]

                # Handle wraparound (e.g., 337.5 to 22.5)
                if end_angle < start_angle:
                    end_angle += 360

                # Create a filled wedge using bar (works better with polar projection)
                theta = math.radians((start_angle + end_angle) / 2)
                width = math.radians(sector_size)

                # Initial color (will be updated in _update_frame)
                bar = ax.bar(
                    theta, self.clearance_threshold,
                    width=width, bottom=0,
                    color='green', alpha=0.15,
                    edgecolor='white', linewidth=0.5,
                    zorder=1
                )
                self._sector_patches.append(bar[0])

                # Add direction label at the edge
                label_theta = theta
                label_r = self.max_distance * 0.92
                ax.text(
                    label_theta, label_r,
                    sector["direction"][:3],  # Abbreviated (e.g., "Fwd", "Rig")
                    ha='center', va='center',
                    fontsize=7, color='white', alpha=0.7,
                    zorder=10
                )

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
            zorder=3  # Above sector wedges
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

        # Add clearance threshold ring
        threshold_angles = np.linspace(0, 2 * np.pi, 100)
        ax.plot(
            threshold_angles,
            [self.clearance_threshold] * len(threshold_angles),
            color='yellow', linewidth=1.5, linestyle='--',
            alpha=0.6, zorder=2, label=f'Clearance threshold ({self.clearance_threshold}mm)'
        )

        # Add legend
        ax.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0), fontsize=8)

        fig.tight_layout()
        return fig, ax, scatter

    def _update_frame(self, _frame):
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

        # -- Update clearance analysis --
        self.clearance.update_from_scan(scan)

        # Update sector wedge colors based on clearance
        if self.show_sectors and self._sector_patches:
            for i, sector in enumerate(self.clearance.sectors):
                patch = self._sector_patches[i]
                if sector["is_clear"]:
                    patch.set_facecolor('green')
                    patch.set_alpha(0.15)
                else:
                    patch.set_facecolor('red')
                    patch.set_alpha(0.25)

        # Call the callback if provided (for integration with other systems)
        if self.clearance_callback is not None:
            try:
                self.clearance_callback(self.clearance)
            except Exception as e:
                print(f"Clearance callback error: {e}")

        # Print clearance info to console if enabled
        if self.print_clearance and count % 10 == 0:  # Every 10th scan to reduce spam
            advice = self.clearance.get_navigation_advice()
            clear_dirs = ", ".join(advice["clear_directions"]) or "None"
            fwd_status = "OK" if advice["can_go_forward"] else "BLOCKED"
            print(f"\r[Scan {count}] Forward: {fwd_status} | Clear: {clear_dirs}", end="")

        # Update title with live stats and clearance summary
        valid = dists[dists > 0]
        if len(valid) > 0:
            advice = self.clearance.get_navigation_advice()
            fwd_indicator = "FWD OK" if advice["can_go_forward"] else "FWD BLOCKED"
            clear_count = len(advice["clear_directions"])
            self._ax.set_title(
                f"LD19 LiDAR  |  {len(scan)} pts  |  "
                f"min {valid.min():.0f} mm  |  max {valid.max():.0f} mm  |  "
                f"{fwd_indicator}  |  {clear_count}/{self.sector_count} clear",
                fontsize=11, pad=20, color="white",
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
        description="Real-time LD19 LiDAR scan visualization with clearance detection."
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
    parser.add_argument(
        "--clearance-threshold", type=float, default=1000,
        help="Distance in mm below which a sector is blocked (default: 1000)."
    )
    parser.add_argument(
        "--sectors", type=int, default=8, choices=[4, 8, 12],
        help="Number of sectors to divide the view (4, 8, or 12; default: 8)."
    )
    parser.add_argument(
        "--no-sectors", action="store_true",
        help="Disable sector overlay visualization."
    )
    parser.add_argument(
        "--print-clearance", action="store_true",
        help="Print clearance info to console."
    )
    return parser.parse_args()


def main():
    args = parse_args()
    viz = LidarVisualizer(
        port=args.port,
        max_distance=args.max_distance,
        min_quality=args.min_quality,
        clearance_threshold=args.clearance_threshold,
        sector_count=args.sectors,
        show_sectors=not args.no_sectors,
        print_clearance=args.print_clearance,
    )
    viz.run()


if __name__ == "__main__":
    main()
