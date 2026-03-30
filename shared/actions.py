# shared/actions.py

"""
This module defines action data structures used by:
- v-group (vehicle actions)
- i-group (signal actions)

These are the ONLY outputs that each group can produce.
"""


from dataclasses import dataclass


@dataclass
class CarAction:
    """
    Represents the action taken by a vehicle at a given step.

    Attributes:
        car_id (int): Unique identifier of the vehicle
        action (str): One of ACTIONS defined in enums.py
    """
    car_id: int
    action: str


@dataclass
class SignalAction:
    """
    Represents the signal decision at an intersection.

    Attributes:
        intersection_id (str): Unique identifier of intersection
        green_direction (str): One of SIGNAL_PHASES ("N" | "S" | "E" | "W")
    """
    intersection_id: str
    green_direction: str