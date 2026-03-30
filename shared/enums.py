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
# Context:
#   STAY       — remain in the current slot (on any segment)
#   MOVE       — advance one slot forward along the current segment
#   STRAIGHT   — cross an intersection without turning
#   TURN_LEFT  — turn left at an intersection (always allowed if signal permits)
#   TURN_RIGHT — turn right at an intersection (forbidden when signal is red)
ACTIONS = [
    "MOVE",
    "STAY",
    "STRAIGHT",
    "TURN_LEFT",
    "TURN_RIGHT",
]


# Traffic signal phases
# Each phase is a single cardinal direction: the one light that is green.
# N: North-bound has green
# S: South-bound has green
# E: East-bound has green
# W: West-bound has green
SIGNAL_PHASES = ["N", "S", "E", "W"]


# Slot boundaries for each road segment
MIN_SLOT = 0
MAX_SLOT = 29

# Vehicle speed constants (miles per hour)
# A car can only travel at full speed or be stopped — no intermediate speed.
SPEED_STOP = 0
SPEED_MOVE = 30   # mph

# Square endpoint nodes (start/destination nodes)
# Each vehicle departs from START_NODE, visits all WAYPOINTS, then returns.
START_NODE = "A"
WAYPOINTS = ["B", "C", "D"]   # must all be visited (in any order)