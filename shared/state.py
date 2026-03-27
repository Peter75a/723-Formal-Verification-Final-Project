# shared/state.py

"""
This module defines the global system state.

This state is the SINGLE SOURCE OF TRUTH for the entire simulation.
Neither v-group nor i-group is allowed to modify it directly.
Only the simulator updates this state.
"""


from dataclasses import dataclass
from typing import Dict


@dataclass
class CarState:
    """
    Represents the state of a single vehicle.

    Attributes:
        car_id (int): Unique identifier
        segment_id (str): Current road segment
        slot (int): Position within the segment (0 to 29)
        direction (str): Direction of travel (N/S/E/W)
        active (bool): Whether the vehicle is still in simulation
    """
    car_id: int
    segment_id: str
    slot: int
    direction: str
    active: bool = True


@dataclass
class SignalState:
    """
    Represents the signal state at an intersection.

    Attributes:
        intersection_id (str): Unique identifier
        green_direction (str): Current green phase ("NS" or "EW")
    """
    intersection_id: str
    green_direction: str


@dataclass
class GlobalState:
    """
    Represents the complete system state at a given simulation step.

    Attributes:
        step (int): Current simulation time step
        cars (Dict[int, CarState]): All vehicle states
        signals (Dict[str, SignalState]): All signal states
    """
    step: int
    cars: Dict[int, CarState]
    signals: Dict[str, SignalState]