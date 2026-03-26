"""
motor_test.py
─────────────
Simple motor test for Yahboom Rosmaster.
Spins all 4 motors forward, then stops.
"""

import time
from Rosmaster_Lib import Rosmaster

bot = Rosmaster()
bot.create_receive_threading()
time.sleep(0.5)  # wait for connection to stabilise

print("Spinning all motors forward...")
bot.set_motor(50, 50, 50, 50)
time.sleep(2)

print("Stopping.")
bot.set_motor(0, 0, 0, 0)