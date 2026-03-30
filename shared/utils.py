# shared/utils.py

"""
Shared utility helpers used across simulator, i-group, and v-group.

These helpers are intentionally lightweight and topology-aware.
"""

from typing import Dict, List, Optional, Tuple

from shared.state import CarState, GlobalState
from shared.topology import Topology, Segment


# ---------------------------------------------------------------------------
# Car / segment position helpers
# ---------------------------------------------------------------------------

def get_car_segment(topology: Topology, car: CarState) -> Optional[Segment]:
    """
    Return the Segment object for the car's current segment_id.
    """
    return topology.get_segment_by_id(car.segment_id)


def is_last_slot(car: CarState, topology: Topology) -> bool:
    """
    Return True iff the car is at the last slot of its current segment.
    """
    seg = get_car_segment(topology, car)
    return seg is not None and car.slot == seg.length - 1


def is_first_slot(car: CarState) -> bool:
    """
    Return True iff the car is at slot 0 of its current segment.
    """
    return car.slot == 0


def distance_to_segment_end(car: CarState, topology: Topology) -> Optional[int]:
    """
    Return number of slots remaining until the segment end.
    Returns None if the segment does not exist.
    """
    seg = get_car_segment(topology, car)
    if seg is None:
        return None
    return (seg.length - 1) - car.slot


def moved_to_new_segment(prev_car: CarState, curr_car: CarState) -> bool:
    """
    Return True iff the car changed segment between two states.
    """
    return prev_car.segment_id != curr_car.segment_id


def slot_advanced(prev_car: CarState, curr_car: CarState) -> bool:
    """
    Return True iff the car advanced to a larger slot on the same segment.
    """
    return (
        prev_car.segment_id == curr_car.segment_id and
        curr_car.slot > prev_car.slot
    )


def stayed_in_place(prev_car: CarState, curr_car: CarState) -> bool:
    """
    Return True iff segment and slot are unchanged.
    """
    return (
        prev_car.segment_id == curr_car.segment_id and
        prev_car.slot == curr_car.slot
    )


# ---------------------------------------------------------------------------
# Intersection crossing helpers
# ---------------------------------------------------------------------------

def segment_end_intersection(topology: Topology, segment_id: str) -> Optional[str]:
    """
    Return the end node if it is an intersection; otherwise None.

    Example:
        "I00_I01" -> "I01"
        "I00_A"   -> None
    """
    seg = topology.get_segment_by_id(segment_id)
    if seg is None:
        return None
    return seg.end if topology.is_intersection(seg.end) else None


def crossed_intersection(prev_car: CarState,
                         curr_car: CarState,
                         topology: Topology) -> bool:
    """
    Return True iff the car appears to have crossed an intersection between
    prev and curr state.

    Definition used here:
      - previous segment ended at an intersection
      - car was at last slot previously
      - current segment is different
    """
    if not moved_to_new_segment(prev_car, curr_car):
        return False

    if not is_last_slot(prev_car, topology):
        return False

    prev_seg = topology.get_segment_by_id(prev_car.segment_id)
    if prev_seg is None:
        return False

    return topology.is_intersection(prev_seg.end)


def get_crossed_intersection(prev_car: CarState,
                             curr_car: CarState,
                             topology: Topology) -> Optional[str]:
    """
    Return the intersection ID crossed between prev and curr state, or None.
    """
    if not crossed_intersection(prev_car, curr_car, topology):
        return None

    prev_seg = topology.get_segment_by_id(prev_car.segment_id)
    if prev_seg is None:
        return None

    return prev_seg.end


def incoming_outgoing_directions(prev_car: CarState,
                                 curr_car: CarState,
                                 topology: Topology) -> Optional[Tuple[str, str]]:
    """
    Return (incoming_direction, outgoing_direction) if the car crossed
    an intersection and both segments exist; otherwise None.
    """
    prev_seg = topology.get_segment_by_id(prev_car.segment_id)
    curr_seg = topology.get_segment_by_id(curr_car.segment_id)

    if prev_seg is None or curr_seg is None:
        return None

    if not crossed_intersection(prev_car, curr_car, topology):
        return None

    return (prev_seg.direction, curr_seg.direction)


# ---------------------------------------------------------------------------
# Occupancy / counting helpers
# ---------------------------------------------------------------------------

def build_segment_slot_occupancy(state: GlobalState) -> Dict[Tuple[str, int], List[int]]:
    """
    Build an occupancy map:
        (segment_id, slot) -> list of car_ids occupying that location
    Only active cars are included.
    """
    occupancy: Dict[Tuple[str, int], List[int]] = {}

    for car_id, car in state.cars.items():
        if not car.active:
            continue
        key = (car.segment_id, car.slot)
        occupancy.setdefault(key, []).append(car_id)

    return occupancy


def count_colliding_slots(state: GlobalState) -> int:
    """
    Count how many segment-slot locations are occupied by more than one active car.
    """
    occupancy = build_segment_slot_occupancy(state)
    return sum(1 for car_ids in occupancy.values() if len(car_ids) > 1)


def active_car_ids(state: GlobalState) -> List[int]:
    """
    Return a sorted list of active car IDs.
    """
    return sorted(car_id for car_id, car in state.cars.items() if car.active)


def cars_on_segment(state: GlobalState, segment_id: str) -> List[CarState]:
    """
    Return active cars currently on the given segment, sorted by slot.
    """
    cars = [
        car for car in state.cars.values()
        if car.active and car.segment_id == segment_id
    ]
    return sorted(cars, key=lambda c: c.slot)


def approaching_cars(state: GlobalState,
                     topology: Topology,
                     intersection_id: str,
                     threshold: int) -> List[CarState]:
    """
    Return active cars that are on incoming segments of the given intersection
    and within `threshold` slots of the segment end.
    """
    inter = topology.get_intersection(intersection_id)
    if inter is None:
        return []

    result: List[CarState] = []
    incoming = set(inter.incoming)

    for car in state.cars.values():
        if not car.active:
            continue
        if car.segment_id not in incoming:
            continue

        seg = topology.get_segment_by_id(car.segment_id)
        if seg is None:
            continue

        if car.slot >= seg.length - threshold:
            result.append(car)

    return result


# ---------------------------------------------------------------------------
# Formatting helpers (useful for debugging / reports)
# ---------------------------------------------------------------------------

def format_car(car: CarState) -> str:
    """
    Return a compact human-readable string for a CarState.
    """
    return (
        f"Car(id={car.car_id}, seg={car.segment_id}, slot={car.slot}, "
        f"dir={car.direction}, speed={car.speed}, active={car.active})"
    )


def format_signal_map(state: GlobalState) -> str:
    """
    Return a compact string for all signal states in sorted order.
    """
    parts = []
    for iid in sorted(state.signals):
        sig = state.signals[iid]
        parts.append(f"{iid}:{sig.green_direction}")
    return ", ".join(parts)


def format_occupancy(state: GlobalState) -> str:
    """
    Return a compact string of active vehicle positions.
    """
    cars = [
        format_car(car)
        for car in sorted(state.cars.values(), key=lambda c: c.car_id)
        if car.active
    ]
    return "\n".join(cars)