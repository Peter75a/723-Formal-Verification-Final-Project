# v_group/perception.py

"""
Perception module for the v-group vehicle controller.

Provides sensor abstractions that a vehicle uses to observe:
  - Whether the signal ahead is green or red for its direction
  - Whether another car is directly ahead on the same segment (blocking)
  - Whether any car is already crossing the target intersection this step
  - Congestion level at upcoming intersections

All methods are stateless helpers — they take GlobalState and CarState as
inputs and return observations. No state is mutated here.
"""

from typing import Optional, List

from shared.state import GlobalState, CarState
from shared.topology import Topology, SLOTS_PER_SEGMENT


# Maximum look-ahead distance in slots (0.5 mile = 30 slots)
VISIBILITY_RANGE = SLOTS_PER_SEGMENT


class Perception:
    """
    Encapsulates all sensing logic for a single vehicle.

    Usage:
        perc = Perception(topology)
        blocked = perc.is_blocked_ahead(car, state)
        green   = perc.is_signal_green(car, state)
    """

    def __init__(self, topology: Topology):
        self.topology = topology

    # ------------------------------------------------------------------
    # Signal sensing
    # ------------------------------------------------------------------

    def is_signal_green(self, car: CarState, state: GlobalState) -> bool:
        """
        Return True if the signal at the intersection at the end of the car's
        current segment is green for the car's direction.

        If the car's segment does not end at an intersection (e.g. endpoint A),
        returns True (no signal → no restriction).

        Args:
            car:   The car being evaluated.
            state: Current global simulation state.

        Returns:
            True if the car may proceed through the upcoming intersection.
        """
        seg = self.topology.get_segment_by_id(car.segment_id)
        if seg is None:
            return True  # unknown segment → conservative allow

        end_node = seg.end
        if not self.topology.is_intersection(end_node):
            return True  # segment ends at endpoint A, no signal

        signal = state.signals.get(end_node)
        if signal is None:
            return True  # no signal data → allow (fail-safe)

        return signal.green_direction == car.direction

    def get_green_direction(self, intersection_id: str,
                            state: GlobalState) -> Optional[str]:
        """
        Return the currently green direction at the given intersection, or None.
        """
        signal = state.signals.get(intersection_id)
        return signal.green_direction if signal else None

    # ------------------------------------------------------------------
    # Forward visibility — same segment
    # ------------------------------------------------------------------

    def is_blocked_ahead(self, car: CarState, state: GlobalState) -> bool:
        """
        Return True if another active car is DIRECTLY ahead of `car` on the
        same segment within VISIBILITY_RANGE slots, with no third car between them.

        "Directly ahead" means: the nearest car in front (higher slot number)
        on the same segment, within the visibility window.

        Per spec: "At a time-step, if car P sees another car Q in front of P
        along the same direction, car P cannot move to the next time step."

        Args:
            car:   The observing car.
            state: Current global state.

        Returns:
            True if car must stop due to a car ahead.
        """
        # Collect slots of all OTHER active cars on the same segment
        ahead_slots = sorted([
            other.slot
            for other in state.cars.values()
            if (other.active
                and other.car_id != car.car_id
                and other.segment_id == car.segment_id
                and other.slot > car.slot
                and other.slot - car.slot <= VISIBILITY_RANGE)
        ])

        if not ahead_slots:
            return False

        # The nearest car ahead is directly in front — no third car in between
        # by definition since we took the smallest slot above car.slot.
        # (A third car would have to be between car.slot and ahead_slots[0].)
        return True

    def nearest_car_ahead_slot(self, car: CarState,
                               state: GlobalState) -> Optional[int]:
        """
        Return the slot of the nearest car directly ahead on the same segment,
        or None if none is visible.
        """
        ahead = [
            other.slot
            for other in state.cars.values()
            if (other.active
                and other.car_id != car.car_id
                and other.segment_id == car.segment_id
                and other.slot > car.slot
                and other.slot - car.slot <= VISIBILITY_RANGE)
        ]
        return min(ahead) if ahead else None

    # ------------------------------------------------------------------
    # Intersection visibility
    # ------------------------------------------------------------------

    def cars_at_intersection_boundary(self, intersection_id: str,
                                      state: GlobalState,
                                      exclude_car_id: int = -1) -> List[CarState]:
        """
        Return active cars that are at slot (length-1) of any incoming segment
        of the given intersection — i.e., cars also queued at the intersection.

        Per spec: "At an intersection, a car can see other cars at the same
        intersection."

        Args:
            intersection_id: Target intersection ID.
            state:           Current global state.
            exclude_car_id:  Car ID to exclude from results (the observing car).

        Returns:
            List of CarState objects at the intersection boundary.
        """
        inter = self.topology.get_intersection(intersection_id)
        if inter is None:
            return []

        result = []
        for seg_id in inter.incoming:
            seg = self.topology.get_segment_by_id(seg_id)
            if seg is None:
                continue
            last_slot = seg.length - 1
            for car in state.cars.values():
                if (car.active
                        and car.car_id != exclude_car_id
                        and car.segment_id == seg_id
                        and car.slot == last_slot):
                    result.append(car)
        return result

    def intersection_will_be_used(self, intersection_id: str,
                                  state: GlobalState,
                                  exclude_car_id: int = -1) -> bool:
        """
        Return True if any other car at the boundary of the given intersection
        has a GREEN signal for its direction — meaning it will actually cross
        this step and not merely wait at red.

        Used to enforce the "at most 1 car crosses per step" rule.

        Args:
            intersection_id:  Target intersection.
            state:            Current global state.
            exclude_car_id:   The car making the query (excluded from check).

        Returns:
            True if the intersection will be occupied by a crossing car this step.
        """
        signal = state.signals.get(intersection_id)
        green_dir = signal.green_direction if signal else None

        for car in self.cars_at_intersection_boundary(
                intersection_id, state, exclude_car_id):
            # Only count cars whose signal is green (they will actually cross)
            if green_dir is not None and car.direction == green_dir:
                return True
        return False

    # ------------------------------------------------------------------
    # Congestion sensing
    # ------------------------------------------------------------------

    def congestion_at(self, intersection_id: str, state: GlobalState) -> int:
        """
        Return the number of stopped cars near the given intersection
        as pre-computed by the simulator.

        Args:
            intersection_id: Intersection to query.
            state:           Current global state.

        Returns:
            Number of stopped cars (0 if unknown).
        """
        return state.congestion.get(intersection_id, 0)

    def is_congested(self, intersection_id: str,
                     state: GlobalState,
                     threshold: int = 3) -> bool:
        """
        Return True if the congestion count at the intersection is at or above
        the threshold. Used by the planner to weight edges.

        Args:
            intersection_id: Target intersection.
            state:           Current global state.
            threshold:       Minimum stopped-car count to consider congested.

        Returns:
            True if congested.
        """
        return self.congestion_at(intersection_id, state) >= threshold