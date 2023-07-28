"""
Calibration:
20550 ticks
114.5 cm
"""

WHEEL_BASE = 0.3300  # m
MAX_TICK_SPEED = 5500  # max tick speed is about 5800
DIST_PER_TICK = 1.145 / 20550  # m
MAX_SPEED = MAX_TICK_SPEED * DIST_PER_TICK  # m/s
# MAX_SPEED = 0.15 # m/s, probably slightly higher, but to guarantee heading...
