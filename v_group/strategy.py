# v_group/strategy.py

"""
Action decision strategy for the v-group vehicle controller.

Given the current GlobalState, this module decides a CarAction for every
active car in one simulation step. The decision pipeline is:

  1. Car inactive               → skip (no action emitted)
  2. Car not at segment end     → check visibility; STAY or MOVE
  3. Car at segment end (slot 29 / last slot):
       a. No upcoming intersection (endpoint A) → handle return-to-A
       b. Red signal for car's direction         → STAY
       c. Intersection in use this step          → STAY
       d. Plan next segment; determine turn type → STRAIGHT / TURN_LEFT / TURN_RIGHT
       e. Right turn at red light                → STAY
       f. Emit crossing action

The strategy is stateless per call; no history is stored here.
"""

from typing import Dict, List, Optional, Tuple

from shared.state import GlobalState, CarState
from shared.actions import CarAction
from shared.topology import Topology, WAYPOINT_INTERSECTIONS
from shared.enums import ACTIONS

from v_group.perception import Perception
from v_group.planner import RoutePlanner


# Turn-direction lookup:
#   (incoming_direction, outgoing_direction) -> action string
_TURN_ACTION: Dict[Tuple[str, str], str] = {
    # Straight
    ("E", "E"): "STRAIGHT",
    ("W", "W"): "STRAIGHT",
    ("N", "N"): "STRAIGHT",
    ("S", "S"): "STRAIGHT",
    # Left turns
    ("E", "N"): "TURN_LEFT",
    ("W", "S"): "TURN_LEFT",
    ("N", "W"): "TURN_LEFT",
    ("S", "E"): "TURN_LEFT",
    # Right turns
    ("E", "S"): "TURN_RIGHT",
    ("W", "N"): "TURN_RIGHT",
    ("N", "E"): "TURN_RIGHT",
    ("S", "W"): "TURN_RIGHT",
}


class ActionStrategy:
    """
    Computes a list of CarAction objects for all active cars in one step.

    Usage:
        strategy = ActionStrategy(topology)
        actions = strategy.decide_all(state)
    """

    def __init__(self, topology: Topology):
        self.topology  = topology
        self.perception = Perception(topology)
        self.planner    = RoutePlanner(topology)

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def decide_all(self, state: GlobalState) -> List[CarAction]:
        """
        Return one CarAction per active car.

        The intersection-use guard is applied globally: once a car is
        committed to crossing an intersection this step, that intersection
        is marked as "in use" so subsequent cars yield.

        Args:
            state: Current global simulation state.

        Returns:
            List of CarAction (one per active car).
        """
        actions: List[CarAction] = []
        # Track which intersections will be crossed this step (by car_id)
        intersections_in_use: Dict[str, int] = {}

        for car_id, car in state.cars.items():
            if not car.active:
                continue
            action = self._decide_one(car, state, intersections_in_use)
            actions.append(action)

        return actions

    # ------------------------------------------------------------------
    # Per-car decision
    # ------------------------------------------------------------------

    def _decide_one(self, car: CarState, state: GlobalState,
                    intersections_in_use: Dict[str, int]) -> CarAction:
        """
        Decide the action for a single active car.

        Args:
            car:                  The car to decide for.
            state:                Current global state.
            intersections_in_use: Mutable dict of {intersection_id: car_id}
                                  for intersections committed this step.

        Returns:
            CarAction for this car.
        """
        seg = self.topology.get_segment_by_id(car.segment_id)
        if seg is None:
            return CarAction(car_id=car.car_id, action="STAY")

        at_last_slot = (car.slot == seg.length - 1)

        # ── Not at segment end: simple move or stay ──────────────────
        if not at_last_slot:
            if self.perception.is_blocked_ahead(car, state):
                return CarAction(car_id=car.car_id, action="STAY")
            return CarAction(car_id=car.car_id, action="MOVE")

        # ── At last slot: approaching intersection ────────────────────
        end_node = seg.end

        # Case: endpoint A (no intersection) — car has returned home
        if not self.topology.is_intersection(end_node):
            # The car is finishing the A_I00 segment in reverse — just MOVE
            # (simulator will deactivate it when it reaches A)
            return CarAction(car_id=car.car_id, action="MOVE")

        # Case: real intersection ahead ───────────────────────────────
        intersection_id = end_node

        # (a) Red light for this direction → STAY
        if not self.perception.is_signal_green(car, state):
            return CarAction(car_id=car.car_id, action="STAY")

        # (b) Intersection already committed to another car this step → STAY
        if intersection_id in intersections_in_use:
            return CarAction(car_id=car.car_id, action="STAY")

        # (c) Another car at intersection boundary → yield
        if self.perception.intersection_will_be_used(
                intersection_id, state, exclude_car_id=car.car_id):
            return CarAction(car_id=car.car_id, action="STAY")

        # (d) Determine next segment from route plan ──────────────────
        next_seg = self._choose_next_segment(car, intersection_id, state)
        if next_seg is None:
            # No valid next segment found — hold position
            return CarAction(car_id=car.car_id, action="STAY")

        # (e) Determine turn type
        outgoing_direction = next_seg.direction
        turn_key = (car.direction, outgoing_direction)
        turn_action = _TURN_ACTION.get(turn_key)

        if turn_action is None:
            # U-turn or unmapped — forbidden
            return CarAction(car_id=car.car_id, action="STAY")

        # Right-turn-at-red is already impossible here: step (a) ensured the
        # signal is green for this car's direction before reaching this point.

        # (f) Destination slot check — slot 0 of the outgoing segment must be free.
        # Per spec, a car at the intersection can see cars already in slot 0 of
        # an outgoing segment (they are still within the intersection zone).
        if self.perception.is_destination_slot_blocked(
                next_seg.segment_id, state, exclude_car_id=car.car_id):
            return CarAction(car_id=car.car_id, action="STAY")

        # ── Commit to crossing ────────────────────────────────────────
        intersections_in_use[intersection_id] = car.car_id
        return CarAction(car_id=car.car_id, action=turn_action)

    # ------------------------------------------------------------------
    # Next-segment selection
    # ------------------------------------------------------------------

    def _choose_next_segment(self, car: CarState,
                             intersection_id: str,
                             state: GlobalState):
        """
        Choose the best outgoing segment from `intersection_id` that moves
        the car toward its current destination, obeying turn rules.

        Args:
            car:              The car making the decision.
            intersection_id:  The intersection the car is about to cross.
            state:            Current global state.

        Returns:
            A Segment object, or None if no valid option is found.
        """
        # Determine destination intersection
        dest_name = car.current_destination
        dest_inter = self.planner.destination_intersection(dest_name)

        # Special case: car is AT its current destination waypoint
        # → update to next destination (handled by simulator, but we
        #   still need to keep moving; choose next waypoint now)
        if intersection_id == dest_inter:
            # Determine next waypoint on the fly
            remaining = [w for w in ["B", "C", "D"]
                         if w not in car.destinations_visited
                         and w != dest_name]
            if not remaining:
                # All waypoints visited — go home to A.
                # If already at I00, take the I00->A endpoint segment directly.
                if intersection_id == "I00":
                    home_seg = self.topology.get_segment("I00", "A")
                    if home_seg and self.topology.is_valid_turn(
                            car.direction, home_seg.direction):
                        return home_seg
                dest_inter = "I00"  # navigate to I00 first, then take A
            else:
                # Nearest remaining
                distances = self.planner._bfs_distances(intersection_id, state)
                next_wp = min(
                    remaining,
                    key=lambda w: distances.get(WAYPOINT_INTERSECTIONS[w],
                                                float("inf"))
                )
                dest_inter = WAYPOINT_INTERSECTIONS[next_wp]

        if dest_inter is None:
            dest_inter = "I00"

        # BFS: find next intersection toward dest_inter
        next_inter = self.planner.next_intersection(car, dest_inter, state)
        if next_inter is None:
            return None

        # Find the outgoing segment from intersection_id to next_inter
        inter = self.topology.get_intersection(intersection_id)
        if inter is None:
            return None

        for seg_id in inter.outgoing:
            seg = self.topology.get_segment_by_id(seg_id)
            if seg is None:
                continue
            if seg.end != next_inter:
                continue
            # Check turn validity (no U-turns)
            if not self.topology.is_valid_turn(car.direction, seg.direction):
                continue
            return seg

        # Fallback: try any valid outgoing direction (not U-turn)
        for seg_id in inter.outgoing:
            seg = self.topology.get_segment_by_id(seg_id)
            if seg is None:
                continue
            if not self.topology.is_intersection(seg.end):
                continue
            if not self.topology.is_valid_turn(car.direction, seg.direction):
                continue
            return seg

        return None