# shared/validators.py

"""
This module provides shared validation helpers for:
- state objects
- action objects
- topology references

These checks help keep i-group, v-group, and simulator consistent.
"""

from typing import Iterable, List, Optional

from shared.enums import (
    DIRECTIONS,
    ACTIONS,
    SIGNAL_PHASES,
    SPEED_STOP,
    SPEED_MOVE,
)
from shared.state import CarState, SignalState, GlobalState
from shared.actions import CarAction, SignalAction
from shared.topology import Topology


# ---------------------------------------------------------------------------
# Basic scalar validators
# ---------------------------------------------------------------------------

def is_valid_direction(direction: str) -> bool:
    """Return True iff direction is one of N/S/E/W."""
    return direction in DIRECTIONS


def is_valid_action(action: str) -> bool:
    """Return True iff action is a supported vehicle action."""
    return action in ACTIONS


def is_valid_signal_phase(phase: str) -> bool:
    """Return True iff phase is one of N/S/E/W."""
    return phase in SIGNAL_PHASES


def is_valid_speed(speed: int) -> bool:
    """Return True iff speed is one of the supported discrete values."""
    return speed in (SPEED_STOP, SPEED_MOVE)


def is_valid_slot(slot: int, segment_length: int) -> bool:
    """Return True iff slot is within [0, segment_length - 1]."""
    return 0 <= slot < segment_length


# ---------------------------------------------------------------------------
# Topology-aware validators
# ---------------------------------------------------------------------------

def segment_exists(topology: Topology, segment_id: str) -> bool:
    """Return True iff segment_id exists in the topology."""
    return topology.get_segment_by_id(segment_id) is not None


def intersection_exists(topology: Topology, intersection_id: str) -> bool:
    """Return True iff intersection_id exists in the topology."""
    return topology.get_intersection(intersection_id) is not None


def car_state_is_valid(car: CarState, topology: Topology) -> bool:
    """
    Validate one CarState against topology and shared enums.

    Checks:
      - valid direction
      - valid speed
      - existing segment
      - slot within segment bounds
    """
    if not is_valid_direction(car.direction):
        return False

    if not is_valid_speed(car.speed):
        return False

    seg = topology.get_segment_by_id(car.segment_id)
    if seg is None:
        return False

    if not is_valid_slot(car.slot, seg.length):
        return False

    return True


def signal_state_is_valid(signal: SignalState, topology: Topology) -> bool:
    """
    Validate one SignalState.

    Checks:
      - intersection exists
      - green direction is a valid phase
    """
    if not intersection_exists(topology, signal.intersection_id):
        return False

    if not is_valid_signal_phase(signal.green_direction):
        return False

    return True


def car_action_is_valid(action: CarAction, state: GlobalState) -> bool:
    """
    Validate one CarAction against current state.

    Checks:
      - action is supported
      - referenced car exists in current state
    """
    if not is_valid_action(action.action):
        return False

    return action.car_id in state.cars


def signal_action_is_valid(action: SignalAction, topology: Topology) -> bool:
    """
    Validate one SignalAction against topology.

    Checks:
      - intersection exists
      - green_direction is a valid phase
    """
    if not intersection_exists(topology, action.intersection_id):
        return False

    if not is_valid_signal_phase(action.green_direction):
        return False

    return True


# ---------------------------------------------------------------------------
# Aggregate validators
# ---------------------------------------------------------------------------

def validate_global_state(state: GlobalState, topology: Topology) -> List[str]:
    """
    Return a list of validation errors for a GlobalState.
    Empty list means the state is valid.
    """
    errors: List[str] = []

    if state.step < 0:
        errors.append(f"Invalid step: {state.step}")

    for car_id, car in state.cars.items():
        if car.car_id != car_id:
            errors.append(
                f"CarState key mismatch: dict key={car_id}, car.car_id={car.car_id}"
            )

        if not car_state_is_valid(car, topology):
            errors.append(
                f"Invalid CarState for car_id={car_id}: "
                f"segment={car.segment_id}, slot={car.slot}, "
                f"direction={car.direction}, speed={car.speed}"
            )

    for iid, signal in state.signals.items():
        if signal.intersection_id != iid:
            errors.append(
                f"SignalState key mismatch: dict key={iid}, "
                f"signal.intersection_id={signal.intersection_id}"
            )

        if not signal_state_is_valid(signal, topology):
            errors.append(
                f"Invalid SignalState for intersection={iid}: "
                f"green_direction={signal.green_direction}"
            )

    return errors


def validate_signal_actions(actions: Iterable[SignalAction],
                            topology: Topology) -> List[str]:
    """
    Validate a collection of SignalAction objects.
    Returns a list of validation errors.
    """
    errors: List[str] = []
    seen = set()

    for action in actions:
        if not signal_action_is_valid(action, topology):
            errors.append(
                f"Invalid SignalAction: "
                f"intersection_id={action.intersection_id}, "
                f"green_direction={action.green_direction}"
            )

        if action.intersection_id in seen:
            errors.append(
                f"Duplicate SignalAction for intersection {action.intersection_id}"
            )
        seen.add(action.intersection_id)

    return errors


def validate_car_actions(actions: Iterable[CarAction],
                         state: GlobalState) -> List[str]:
    """
    Validate a collection of CarAction objects.
    Returns a list of validation errors.
    """
    errors: List[str] = []
    seen = set()

    for action in actions:
        if not car_action_is_valid(action, state):
            errors.append(
                f"Invalid CarAction: car_id={action.car_id}, action={action.action}"
            )

        if action.car_id in seen:
            errors.append(f"Duplicate CarAction for car_id={action.car_id}")
        seen.add(action.car_id)

    return errors


# ---------------------------------------------------------------------------
# Strict assertion helpers (optional to use in simulator/tests)
# ---------------------------------------------------------------------------

def assert_valid_global_state(state: GlobalState, topology: Topology) -> None:
    """
    Raise ValueError if GlobalState is invalid.
    """
    errors = validate_global_state(state, topology)
    if errors:
        raise ValueError("GlobalState validation failed:\n" + "\n".join(errors))


def assert_valid_signal_actions(actions: Iterable[SignalAction],
                                topology: Topology) -> None:
    """
    Raise ValueError if any SignalAction is invalid.
    """
    errors = validate_signal_actions(actions, topology)
    if errors:
        raise ValueError("SignalAction validation failed:\n" + "\n".join(errors))


def assert_valid_car_actions(actions: Iterable[CarAction],
                             state: GlobalState) -> None:
    """
    Raise ValueError if any CarAction is invalid.
    """
    errors = validate_car_actions(actions, state)
    if errors:
        raise ValueError("CarAction validation failed:\n" + "\n".join(errors))