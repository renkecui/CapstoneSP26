"""
ir_obstacle.py
──────────────
Python equivalent of the Arduino IR + LED sketch.

Arduino → Raspberry Pi pin mapping:
    IR sensor (Arduino pin 2)  →  GPIO17 (Pi Pin 11)
    LED       (Arduino pin 13) →  GPIO27 (Pi Pin 13)

Wiring:
    IR sensor:
        VCC  →  Pi Pin 1  (3.3V)
        GND  →  Pi Pin 6  (GND)
        OUT  →  Pi Pin 11 (GPIO17)

    LED:
        Anode (+)  →  330Ω resistor  →  Pi Pin 13 (GPIO27)
        Cathode (-)  →  Pi Pin 14    (GND)

Usage:
    python3 ir_obstacle.py
"""

import time

try:
    import RPi.GPIO as GPIO
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("[WARN] RPi.GPIO not found — running in simulation mode.\n")


# ──────────────────────────────────────────────
# Pin config — mirrors Arduino sketch
# Arduino pin 2  (IR)  → GPIO17
# Arduino pin 13 (LED) → GPIO27
# ──────────────────────────────────────────────

IR_PIN  = 17   # GPIO17 — Pi Pin 11
LED_PIN = 27   # GPIO27 — Pi Pin 13


# ──────────────────────────────────────────────
# Simulation stub
# ──────────────────────────────────────────────

class GPIOSim:
    BCM  = "BCM"
    IN   = "IN"
    OUT  = "OUT"
    LOW  = 0
    HIGH = 1

    def __init__(self):
        self._tick = 0

    def setmode(self, mode): pass
    def setup(self, pin, mode): pass
    def cleanup(self): print("  [SIM] GPIO cleanup.")

    def input(self, pin):
        self._tick += 1
        return self.LOW if (self._tick // 20) % 2 == 1 else self.HIGH

    def output(self, pin, state):
        if pin == LED_PIN:
            print(f"  [SIM] LED {'ON  💡' if state == self.HIGH else 'OFF   '}")


# ──────────────────────────────────────────────
# Main — direct Python translation of Arduino sketch
# ──────────────────────────────────────────────

def main():
    gpio = GPIO if HARDWARE_AVAILABLE else GPIOSim()

    # setup()
    gpio.setmode(gpio.BCM)
    gpio.setup(IR_PIN,  gpio.IN)
    gpio.setup(LED_PIN, gpio.OUT)

    mode = "HARDWARE" if HARDWARE_AVAILABLE else "SIMULATION"
    print(f"IR obstacle detector running [{mode}]")
    print(f"  IR sensor → GPIO{IR_PIN}  (Pi Pin 11)")
    print(f"  LED       → GPIO{LED_PIN}  (Pi Pin 13)")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            # loop()
            ir_read = gpio.input(IR_PIN)

            gpio.output(LED_PIN, gpio.LOW)   # LED off by default

            if ir_read == 0:                 # obstacle detected (LOW = obstacle)
                gpio.output(LED_PIN, gpio.HIGH)  # turn LED on
                print("⚠  Obstacle detected — LED ON ")
            else:
                print("✓  Clear             — LED OFF")

            time.sleep(0.05)  # ~20Hz loop, same cadence as Arduino

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        gpio.output(LED_PIN, gpio.LOW)  # make sure LED is off on exit
        gpio.cleanup()
        print("Done.")


if __name__ == "__main__":
    main()
