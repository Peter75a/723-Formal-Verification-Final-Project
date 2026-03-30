# shared/state.py

"""
This module defines the global system state.

This state is the SINGLE SOURCE OF TRUTH for the entire simulation.
Neither v-group nor i-group is allowed to modify it directly.
Only the simulator updates this state.
"""


from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class CarState:
    """
    Represents the state of a single vehicle.

    Attributes:
        car_id (int):                  Unique identifier
        segment_id (str):              Current road segment (e.g. "I00_I01")
        slot (int):                    Position within the segment (0 to 29)
        direction (str):               Direction of travel (N/S/E/W)
        speed (int):                   Current speed in mph; 0 = stopped, 30 = moving
        active (bool):                 Whether the vehicle is still in simulation
        destinations_visited (List):   Subset of {B, C, D} already visited
        current_destination (str):     Next node the car is heading to
    """
    car_id: int
    segment_id: str
    slot: int
    direction: str
    speed: int = 30                                          # SPEED_MOVE by default
    active: bool = True
    destinations_visited: List[str] = field(default_factory=list)
    current_destination: str = "B"                          # initial target waypoint


@dataclass
class SignalState:
    """
    Represents the signal state at an intersection.

    Attributes:
        intersection_id (str):  Unique identifier
        green_direction (str):  Current green phase ("N" | "S" | "E" | "W")
    """
    intersection_id: str
    green_direction: str


@dataclass
class GlobalState:
    """
    Represents the complete system state at a given simulation step.

    Attributes:
        step (int):                       Current simulation time step
        cars (Dict[int, CarState]):       All vehicle states keyed by car_id
        signals (Dict[str, SignalState]): All signal states keyed by intersection_id
        congestion (Dict[str, int]):      Number of stopped cars near each intersection
                                          (pre-computed by simulator for v-group use)
    """
    step: int
    cars: Dict[int, CarState]
    signals: Dict[str, SignalState]
    congestion: Dict[str, int] = field(default_factory=dict)