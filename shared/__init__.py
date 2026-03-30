# shared/__init__.py

"""
Shared module package.

This package contains:
- State definitions  (state.py)
- Action definitions (actions.py)
- Topology           (topology.py)
- Enums / constants  (enums.py)

All modules must import from here to ensure consistency.
"""

from shared.enums import (
    DIRECTIONS,
    ACTIONS,
    SIGNAL_PHASES,
    MIN_SLOT,
    MAX_SLOT,
    SPEED_STOP,
    SPEED_MOVE,
    START_NODE,
    WAYPOINTS,
)

from shared.state import CarState, SignalState, GlobalState

from shared.actions import CarAction, SignalAction

from shared.topology import (
    Topology, Segment, Intersection,
    SLOTS_PER_SEGMENT, ENDPOINT_SLOTS, WAYPOINT_INTERSECTIONS,
)

__all__ = [
    # enums
    "DIRECTIONS", "ACTIONS", "SIGNAL_PHASES",
    "MIN_SLOT", "MAX_SLOT",
    "SPEED_STOP", "SPEED_MOVE",
    "START_NODE", "WAYPOINTS",
    # state
    "CarState", "SignalState", "GlobalState",
    # actions
    "CarAction", "SignalAction",
    # topology
    "Topology", "Segment", "Intersection",
    "SLOTS_PER_SEGMENT", "ENDPOINT_SLOTS", "WAYPOINT_INTERSECTIONS",
]