# v_group/controller.py

"""
Vehicle Controller — top-level v-group interface.

Responsibilities:
  1. decide_actions(state)   → produce CarAction list for all active cars
  2. verify_step(prev, curr) → independently detect collisions, red-light
                               violations, and wrong-way driving
  3. get_stats()             → return cumulative violation counts

The simulator calls decide_actions() each step, then calls verify_step()
after applying all moves. Verification results must match i-group's output.
"""

from typing import Dict, List, Tuple

from shared.topology import Topology
from shared.state import GlobalState
from shared.actions import CarAction

from v_group.perception import Perception
from v_group.planner import RoutePlanner
from v_group.strategy import ActionStrategy


class VehicleController:
    """
    Main v-group controller.

    Usage:
        topo = Topology(); topo.build()
        ctrl = VehicleController(topo)

        # each simulation step:
        actions = ctrl.decide_actions(state)
        # ... simulator applies actions to produce next_state ...
        report  = ctrl.verify_step(state, next_state)
    """

    def __init__(self, topology: Topology):
        self.topology   = topology
        self.perception = Perception(topology)
        self.planner    = RoutePlanner(topology)
        self.strategy   = ActionStrategy(topology)

        # Cumulative violation counters (must match i-group's verify_step)
        self.total_collisions:                int = 0
        self.total_red_light_violations:      int = 0
        self.total_wrong_way_violations:      int = 0
        self.total_illegal_turn_violations:   int = 0
        self.total_multi_crossing_violations: int = 0

        # Throughput tracking
        self.total_completed_tours: int = 0

    # ------------------------------------------------------------------
    # Action computation  (called BEFORE vehicles move)
    # ------------------------------------------------------------------

    def decide_actions(self, state: GlobalState) -> List[CarAction]:
        """
        Compute the action for every active car.

        Args:
            state: Current global state.

        Returns:
            List of CarAction, one per active car.
        """
        return self.strategy.decide_all(state)

    # ------------------------------------------------------------------
    # Verification  (called AFTER vehicles move)
    # ------------------------------------------------------------------

    def verify_step(self,
                    prev_state: GlobalState,
                    curr_state: GlobalState) -> Dict[str, int]:
        """
        Independently verify safety constraints between two consecutive steps.

        This mirrors InfrastructureController.verify_step() — results should
        agree with i-group on every step.

        Args:
            prev_state: State at step t   (signals in effect when cars moved).
            curr_state: State at step t+1 (positions after cars moved).

        Returns:
            Dict with keys "step", "collisions", "red_light_violations",
            "wrong_way_violations", "illegal_turn_violations",
            "multi_crossing_violations" for this single step.
        """
        collisions     = self._check_collisions(curr_state)
        red_viols      = self._check_red_light_violations(prev_state, curr_state)
        wrong_way      = self._check_wrong_way(curr_state)
        illegal_turns  = self._check_illegal_turns(prev_state, curr_state)
        multi_cross    = self._check_multi_crossing(prev_state, curr_state)

        self.total_collisions           += collisions
        self.total_red_light_violations += red_viols
        self.total_wrong_way_violations += wrong_way
        self.total_illegal_turn_violations  += illegal_turns
        self.total_multi_crossing_violations += multi_cross

        return {
            "step":                      curr_state.step,
            "collisions":                collisions,
            "red_light_violations":      red_viols,
            "wrong_way_violations":      wrong_way,
            "illegal_turn_violations":   illegal_turns,
            "multi_crossing_violations": multi_cross,
        }

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    def get_stats(self) -> Dict[str, int]:
        """Return cumulative violation counts over all verified steps."""
        return {
            "total_collisions":                self.total_collisions,
            "total_red_light_violations":      self.total_red_light_violations,
            "total_wrong_way_violations":      self.total_wrong_way_violations,
            "total_illegal_turn_violations":   self.total_illegal_turn_violations,
            "total_multi_crossing_violations": self.total_multi_crossing_violations,
            "total_completed_tours":           self.total_completed_tours,
        }

    def record_completed_tour(self) -> None:
        """Call when a car completes its full A→B→C→D→A tour."""
        self.total_completed_tours += 1

    # ------------------------------------------------------------------
    # Internal checkers  (mirror of i_group/controller.py)
    # ------------------------------------------------------------------

    def _check_collisions(self, state: GlobalState) -> int:
        """
        Count slots occupied by more than one active car.
        Each such slot is one collision event.
        """
        occupancy: Dict[Tuple[str, int], int] = {}
        for car in state.cars.values():
            if not car.active:
                continue
            key = (car.segment_id, car.slot)
            occupancy[key] = occupancy.get(key, 0) + 1

        return sum(1 for count in occupancy.values() if count > 1)

    def _check_red_light_violations(self,
                                     prev: GlobalState,
                                     curr: GlobalState) -> int:
        """
        Detect cars that crossed an intersection while the signal was red.

        Mirrors i_group.controller._check_red_light_violations exactly.
        """
        violations = 0

        for car_id, curr_car in curr.cars.items():
            if not curr_car.active:
                continue
            prev_car = prev.cars.get(car_id)
            if prev_car is None or not prev_car.active:
                continue

            # Car must have been at the end of its previous segment
            prev_seg = self.topology.get_segment_by_id(prev_car.segment_id)
            if prev_seg is None:
                continue
            if prev_car.slot != prev_seg.length - 1:
                continue

            # Car must have moved to a new segment
            if prev_car.segment_id == curr_car.segment_id:
                continue

            # The segment must end at a real intersection
            intersection_id = prev_seg.end
            if not self.topology.is_intersection(intersection_id):
                continue

            # Check if the signal was green for this car's direction
            signal = prev.signals.get(intersection_id)
            if signal is None:
                continue

            if signal.green_direction != prev_car.direction:
                violations += 1

        return violations

    def _check_wrong_way(self, state: GlobalState) -> int:
        """
        Count active cars whose travel direction does not match
        their current segment's defined direction.
        """
        violations = 0
        for car in state.cars.values():
            if not car.active:
                continue
            seg = self.topology.get_segment_by_id(car.segment_id)
            if seg and car.direction != seg.direction:
                violations += 1
        return violations

    def _check_illegal_turns(self,
                              prev: GlobalState,
                              curr: GlobalState) -> int:
        """
        Detect U-turns: a car that changed segments and whose new direction
        is the direct opposite of its old direction (i.e. reversed travel).

        Opposite pairs: E↔W, N↔S.
        """
        _OPPOSITE = {"E": "W", "W": "E", "N": "S", "S": "N"}
        violations = 0

        for car_id, curr_car in curr.cars.items():
            if not curr_car.active:
                continue
            prev_car = prev.cars.get(car_id)
            if prev_car is None or not prev_car.active:
                continue
            # Must have crossed a segment boundary
            if prev_car.segment_id == curr_car.segment_id:
                continue
            if _OPPOSITE.get(prev_car.direction) == curr_car.direction:
                violations += 1

        return violations

    def _check_multi_crossing(self,
                               prev: GlobalState,
                               curr: GlobalState) -> int:
        """
        Detect steps where more than one car crossed the same intersection.

        A car crosses intersection X if it was at the last slot of an incoming
        segment of X and is now on a different segment.

        Per spec: "between two consecutive time steps, there is at most 1 car
        crossing an intersection."
        """
        from collections import Counter
        crossings: Counter = Counter()

        for car_id, curr_car in curr.cars.items():
            if not curr_car.active:
                continue
            prev_car = prev.cars.get(car_id)
            if prev_car is None or not prev_car.active:
                continue
            if prev_car.segment_id == curr_car.segment_id:
                continue

            prev_seg = self.topology.get_segment_by_id(prev_car.segment_id)
            if prev_seg is None:
                continue
            if prev_car.slot != prev_seg.length - 1:
                continue
            if not self.topology.is_intersection(prev_seg.end):
                continue

            crossings[prev_seg.end] += 1

        return sum(1 for count in crossings.values() if count > 1)