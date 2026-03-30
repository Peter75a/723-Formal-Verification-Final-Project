# shared/protocol.py

"""
This module defines shared message / protocol structures for communication
between simulator, v-group, and i-group.

Goal:
    Make the interface between groups explicit and type-safe.

Typical step flow:
    1. Simulator provides current GlobalState
    2. v-group returns vehicle actions
    3. i-group returns signal actions
    4. Simulator applies actions and produces next GlobalState
    5. Verifiers generate a step report
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from shared.state import GlobalState, CarState, SignalState
from shared.actions import CarAction, SignalAction


# ---------------------------------------------------------------------------
# Group input / output protocol objects
# ---------------------------------------------------------------------------

@dataclass
class VehicleInput:
    """
    Input passed from simulator/shared environment to v-group.

    Attributes:
        step (int):              Current simulation step
        state (GlobalState):     Full current global state
    """
    step: int
    state: GlobalState


@dataclass
class VehicleOutput:
    """
    Output returned by v-group to simulator.

    Attributes:
        step (int):                  Step number this decision belongs to
        car_actions (List[CarAction]): Vehicle actions for this step
    """
    step: int
    car_actions: List[CarAction] = field(default_factory=list)


@dataclass
class InfrastructureInput:
    """
    Input passed from simulator/shared environment to i-group.

    Attributes:
        step (int):              Current simulation step
        state (GlobalState):     Full current global state
    """
    step: int
    state: GlobalState


@dataclass
class InfrastructureOutput:
    """
    Output returned by i-group to simulator.

    Attributes:
        step (int):                        Step number this decision belongs to
        signal_actions (List[SignalAction]): Signal decisions for this step
    """
    step: int
    signal_actions: List[SignalAction] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Optional compact protocol messages
# ---------------------------------------------------------------------------

@dataclass
class VehicleSnapshot:
    """
    Lightweight snapshot of a vehicle for protocol/debug/reporting use.

    Attributes:
        car_id (int):            Vehicle ID
        segment_id (str):        Current segment ID
        slot (int):              Current slot index
        direction (str):         Current travel direction
        speed (int):             Current speed (0 or 30)
        active (bool):           Whether the car is still active
    """
    car_id: int
    segment_id: str
    slot: int
    direction: str
    speed: int
    active: bool


@dataclass
class SignalSnapshot:
    """
    Lightweight snapshot of one signal state.

    Attributes:
        intersection_id (str):   Intersection ID
        green_direction (str):   Current green phase ("N" | "S" | "E" | "W")
    """
    intersection_id: str
    green_direction: str


# ---------------------------------------------------------------------------
# Verification / reporting protocol
# ---------------------------------------------------------------------------

@dataclass
class StepViolationReport:
    """
    Per-step verification result.

    Attributes:
        step (int):                          Step number
        collisions (int):                    Collision count in this step
        red_light_violations (int):          Red-light violation count
        wrong_way_violations (int):          Wrong-way driving count
        illegal_turn_violations (int):       Optional illegal-turn / U-turn count
        multi_crossing_violations (int):     Optional same-intersection multi-cross count
        notes (List[str]):                   Optional debug notes
    """
    step: int
    collisions: int = 0
    red_light_violations: int = 0
    wrong_way_violations: int = 0
    illegal_turn_violations: int = 0
    multi_crossing_violations: int = 0
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, int]:
        """
        Convert the report to a dict compatible with the current controller style.
        """
        return {
            "step": self.step,
            "collisions": self.collisions,
            "red_light_violations": self.red_light_violations,
            "wrong_way_violations": self.wrong_way_violations,
            "illegal_turn_violations": self.illegal_turn_violations,
            "multi_crossing_violations": self.multi_crossing_violations,
        }


@dataclass
class SimulationSummary:
    """
    Aggregate simulation summary for reporting.

    Attributes:
        total_steps (int):                    Total simulated steps
        total_collisions (int):               Total collisions
        total_red_light_violations (int):     Total red-light violations
        total_wrong_way_violations (int):     Total wrong-way violations
        total_illegal_turn_violations (int):  Total illegal-turn violations
        total_multi_crossing_violations (int): Total same-intersection multi-cross violations
    """
    total_steps: int = 0
    total_collisions: int = 0
    total_red_light_violations: int = 0
    total_wrong_way_violations: int = 0
    total_illegal_turn_violations: int = 0
    total_multi_crossing_violations: int = 0


# ---------------------------------------------------------------------------
# Helper conversion functions
# ---------------------------------------------------------------------------

def car_state_to_snapshot(car: CarState) -> VehicleSnapshot:
    """
    Convert a CarState into a lightweight VehicleSnapshot.
    """
    return VehicleSnapshot(
        car_id=car.car_id,
        segment_id=car.segment_id,
        slot=car.slot,
        direction=car.direction,
        speed=car.speed,
        active=car.active,
    )


def signal_state_to_snapshot(signal: SignalState) -> SignalSnapshot:
    """
    Convert a SignalState into a lightweight SignalSnapshot.
    """
    return SignalSnapshot(
        intersection_id=signal.intersection_id,
        green_direction=signal.green_direction,
    )


def build_vehicle_input(state: GlobalState) -> VehicleInput:
    """
    Convenience helper to create a VehicleInput from GlobalState.
    """
    return VehicleInput(
        step=state.step,
        state=state,
    )


def build_infrastructure_input(state: GlobalState) -> InfrastructureInput:
    """
    Convenience helper to create an InfrastructureInput from GlobalState.
    """
    return InfrastructureInput(
        step=state.step,
        state=state,
    )