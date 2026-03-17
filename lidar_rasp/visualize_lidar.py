#!/usr/bin/env python3
"""
Raspberry Pi visualization entrypoint.

Runs the original visualization script so behavior stays consistent with the
existing project. Keep the original `lidar/visualize_lidar.py` in the repo.
"""

import runpy
import sys
from pathlib import Path


def main():
    project_root = Path(__file__).resolve().parent.parent
    legacy_lidar_dir = project_root / "lidar"
    legacy_visualizer = legacy_lidar_dir / "visualize_lidar.py"

    if not legacy_visualizer.exists():
        raise FileNotFoundError(
            "Expected visualization script at lidar/visualize_lidar.py but it was not found."
        )

    # Ensure imports inside visualize_lidar.py resolve as in the original layout.
    sys.path.insert(0, str(legacy_lidar_dir))
    runpy.run_path(str(legacy_visualizer), run_name="__main__")


if __name__ == "__main__":
    main()
