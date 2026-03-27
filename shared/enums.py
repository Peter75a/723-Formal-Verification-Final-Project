# shared/enums.py

"""
This module defines all global enumerations and constants
that must be shared across v-group, i-group, and simulator.

IMPORTANT:
These values must remain consistent across all modules.
"""


# Cardinal directions used by vehicles
DIRECTIONS = ["N", "S", "E", "W"]


# Allowed vehicle actions per simulation step
ACTIONS = [
    "MOVE",        # move forward along the segment
    "STAY",        # remain in the current slot
    "TURN_LEFT",   # turn left at intersection
    "STRAIGHT"     # go straight through intersection
]


# Traffic signal phases
# NS: North-South direction has green
# EW: East-West direction has green
SIGNAL_PHASES = ["NS", "EW"]


# Slot boundaries for each road segment
MIN_SLOT = 0
MAX_SLOT = 29