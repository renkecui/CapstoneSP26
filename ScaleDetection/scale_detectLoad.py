#!/usr/bin/env python3
import time
import statistics
from collections import deque

try:
    from HX711 import HX711
except ImportError:
    raise SystemExit(
        "Could not import HX711. Try: pip3 install HX711\n"
        "If you installed a different hx711 library, paste its usage and I’ll adapt the script."
    )

# ----------------------------
# Configuration
# ----------------------------

# Raspberry Pi BCM pin numbers (NOT physical board pins)
DOUT_PIN = 5   # change to your wiring
SCK_PIN  = 6   # change to your wiring

# Detection parameters
OBJECT_THRESHOLD_KG = 2.0     # object-present threshold (kg). Tune this.
RELEASE_THRESHOLD_KG = 1.0    # must drop below this to count as empty (hysteresis)

# Debounce times (seconds): require stable condition for this long before switching state
DETECT_DEBOUNCE_S  = 0.40
RELEASE_DEBOUNCE_S = 0.60

# Filtering
SAMPLES_PER_READING = 8       # how many raw reads to take for each "reading"
MEDIAN_WINDOW = 5             # median filter window (readings)
MOVING_AVG_WINDOW = 10        # moving average window after median

# Calibration
# You must calibrate to convert raw HX711 units into kg.
# Set SCALE_FACTOR after calibration. If unknown, start with 1.0 and run calibration mode.
SCALE_FACTOR = 1.0  # raw_units_per_kg (will be computed via calibration)

# ----------------------------
# Helper functions
# ----------------------------

def read_raw_average(hx: HX711, n: int) -> float:
    """Read HX711 n times and return the average (raw units)."""
    vals = []
    for _ in range(n):
        vals.append(hx.get_weight(1))  # many libs treat arg as "times"; some ignore it
    return sum(vals) / len(vals)

def stable_time_gate(condition: bool, last_flip_time: float, debounce_s: float) -> tuple[bool, float]:
    """
    Returns (allowed_to_flip, updated_last_flip_time_candidate)
    If condition is True continuously for debounce_s, allow flip.
    """
    now = time.time()
    if condition:
        if last_flip_time == 0.0:
            return False, now  # start timing
        if (now - last_flip_time) >= debounce_s:
            return True, last_flip_time
        return False, last_flip_time
    else:
        return False, 0.0  # reset timer if condition not met

def calibrate(hx: HX711) -> float:
    """
    Interactive calibration:
    1) Tare empty scale
    2) Ask for known weight (kg)
    3) Compute scale factor (raw_units_per_kg)
    """
    print("\n--- Calibration Mode ---")
    input("Remove everything from the scale. Press Enter to tare...")
    hx.tare()
    time.sleep(0.5)

    known_kg = float(input("Place a known weight on the scale.\nEnter its weight in kg: ").strip())
    print("Reading... hold still for ~3 seconds")
    readings = []
    t_end = time.time() + 3.0
    while time.time() < t_end:
        readings.append(read_raw_average(hx, SAMPLES_PER_READING))
        time.sleep(0.1)

    raw = statistics.median(readings)
    if known_kg <= 0:
        raise ValueError("Known weight must be > 0")

    scale_factor = raw / known_kg
    print(f"\nCalibration result:")
    print(f"  raw median units = {raw:.3f}")
    print(f"  scale factor     = {scale_factor:.6f} raw_units_per_kg")
    print("Set SCALE_FACTOR to this value in the script.\n")
    return scale_factor

# ----------------------------
# Main
# ----------------------------

def main():
    hx = HX711(DOUT_PIN, SCK_PIN)

    # Depending on your HX711 library, you may need these:
    # hx.set_reading_format("MSB", "MSB")
    # hx.set_reference_unit(1)
    # hx.reset()

    print("Taring scale (empty)...")
    hx.tare()
    time.sleep(0.5)

    # Optional: run calibration interactively
    # Uncomment the next line if you haven't calibrated yet:
    # calibrate(hx); return

    # If your library supports reference unit, you can use it:
    # hx.set_reference_unit(SCALE_FACTOR)

    median_buf = deque(maxlen=MEDIAN_WINDOW)
    avg_buf = deque(maxlen=MOVING_AVG_WINDOW)

    object_present = False
    detect_timer = 0.0
    release_timer = 0.0

    print("\nRunning. Ctrl+C to stop.")
    print(f"Thresholds: detect >= {OBJECT_THRESHOLD_KG} kg, release <= {RELEASE_THRESHOLD_KG} kg\n")

    try:
        while True:
            raw = read_raw_average(hx, SAMPLES_PER_READING)

            # Convert raw -> kg.
            # If your library uses set_reference_unit, replace with hx.get_weight(...)
            kg = raw / SCALE_FACTOR

            # Median filter
            median_buf.append(kg)
            med = statistics.median(median_buf)

            # Moving average over median output
            avg_buf.append(med)
            smooth = sum(avg_buf) / len(avg_buf)

            # Hysteresis conditions
            want_present = smooth >= OBJECT_THRESHOLD_KG
            want_empty = smooth <= RELEASE_THRESHOLD_KG

            if not object_present:
                allowed, detect_timer = stable_time_gate(want_present, detect_timer, DETECT_DEBOUNCE_S)
                if allowed:
                    object_present = True
                    release_timer = 0.0
                    print(f"[{time.strftime('%H:%M:%S')}] OBJECT DETECTED  (weight ~ {smooth:.2f} kg)")
            else:
                allowed, release_timer = stable_time_gate(want_empty, release_timer, RELEASE_DEBOUNCE_S)
                if allowed:
                    object_present = False
                    detect_timer = 0.0
                    print(f"[{time.strftime('%H:%M:%S')}] SCALE EMPTY      (weight ~ {smooth:.2f} kg)")

            # Optional live readout (comment out if noisy)
            # print(f"\r{smooth:7.2f} kg  {'PRESENT' if object_present else 'empty  '}", end="")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Some libraries need GPIO cleanup; others handle internally.
        try:
            hx.power_down()
            time.sleep(0.1)
            hx.power_up()
        except Exception:
            pass

if __name__ == "__main__":
    main()
