# shared/__init__.py

"""
Shared module package.

This package contains:
- State definitions      (state.py)
- Action definitions     (actions.py)
- Topology               (topology.py)
- Enums / constants      (enums.py)
- Protocol structures    (protocol.py)
- Validation helpers     (validators.py)
- Utility helpers        (utils.py)
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
    Topology,
    Segment,
    Intersection,
    SLOTS_PER_SEGMENT,
    ENDPOINT_SLOTS,
    WAYPOINT_INTERSECTIONS,
)

from shared.protocol import (
    VehicleInput,
    VehicleOutput,
    InfrastructureInput,
    InfrastructureOutput,
    StepViolationReport,
    SimulationSummary,
)

from shared.validators import (
    validate_global_state,
    validate_signal_actions,
    validate_car_actions,
    assert_valid_global_state,
    assert_valid_signal_actions,
    assert_valid_car_actions,
)

__all__ = [
    "DIRECTIONS",
    "ACTIONS",
    "SIGNAL_PHASES",
    "MIN_SLOT",
    "MAX_SLOT",
    "SPEED_STOP",
    "SPEED_MOVE",
    "START_NODE",
    "WAYPOINTS",

    "CarState",
    "SignalState",
    "GlobalState",

    "CarAction",
    "SignalAction",

    "Topology",
    "Segment",
    "Intersection",
    "SLOTS_PER_SEGMENT",
    "ENDPOINT_SLOTS",
    "WAYPOINT_INTERSECTIONS",

    "VehicleInput",
    "VehicleOutput",
    "InfrastructureInput",
    "InfrastructureOutput",
    "StepViolationReport",
    "SimulationSummary",

    "validate_global_state",
    "validate_signal_actions",
    "validate_car_actions",
    "assert_valid_global_state",
    "assert_valid_signal_actions",
    "assert_valid_car_actions",
]