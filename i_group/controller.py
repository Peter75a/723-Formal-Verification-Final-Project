# i_group/controller.py

"""
Infrastructure Controller — top-level i-group interface.

Responsibilities:
  1. compute_signals(state)  → decide green/red for every intersection
  2. verify_step(prev, curr) → detect collisions, red-light violations,
                               and wrong-way driving between two steps
  3. get_stats()             → return cumulative violation counts

The simulator calls compute_signals() each step to get signal decisions,
then calls verify_step() after applying vehicle moves.
"""

from typing import Dict, List, Tuple

from shared.topology import Topology
from shared.state import GlobalState
from shared.actions import SignalAction

from i_group.scheduler import SignalScheduler
from i_group.policy import SignalPolicy, _direction_matches_phase


class InfrastructureController:
    """
    Main i-group controller.

    Usage:
        topo = Topology(); topo.build()
        ctrl = InfrastructureController(topo)

        # each simulation step:
        signals = ctrl.compute_signals(state)
        # ... simulator applies vehicle moves to produce next_state ...
        report  = ctrl.verify_step(state, next_state)
    """

    def __init__(self, topology: Topology,
                 min_green: int = 10, max_green: int = 40):
        self.topology  = topology
        self.scheduler = SignalScheduler(min_green, max_green)
        self.scheduler.initialize(topology.all_intersection_ids())
        self.policy    = SignalPolicy(topology, self.scheduler)

        # Cumulative violation counters
        self.total_collisions:          int = 0
        self.total_red_light_violations: int = 0
        self.total_wrong_way_violations: int = 0

    # ------------------------------------------------------------------
    # Signal computation  (called BEFORE vehicles move)
    # ------------------------------------------------------------------

    def compute_signals(self, state: GlobalState) -> List[SignalAction]:
        """
        Compute the green phase for every intersection.

        Args:
            state: Current global state (cars + existing signals).

        Returns:
            List of SignalAction, one per intersection.
        """
        return [
            SignalAction(
                intersection_id=iid,
                green_direction=self.policy.decide(iid, state),
            )
            for iid in self.topology.all_intersection_ids()
        ]

    # ------------------------------------------------------------------
    # Verification  (called AFTER vehicles move)
    # ------------------------------------------------------------------

    def verify_step(self,
                    prev_state: GlobalState,
                    curr_state: GlobalState) -> Dict[str, int]:
        """
        Check for violations between two consecutive simulation steps.

        Args:
            prev_state: State at step t   (signals in effect when cars moved).
            curr_state: State at step t+1 (positions after cars moved).

        Returns:
            Dict with keys "step", "collisions", "red_light_violations",
            "wrong_way_violations" — all counts for this single step.
        """
        collisions   = self._check_collisions(curr_state)
        red_viols    = self._check_red_light_violations(prev_state, curr_state)
        wrong_way    = self._check_wrong_way(curr_state)

        self.total_collisions           += collisions
        self.total_red_light_violations += red_viols
        self.total_wrong_way_violations += wrong_way

        return {
            "step":                  curr_state.step,
            "collisions":            collisions,
            "red_light_violations":  red_viols,
            "wrong_way_violations":  wrong_way,
        }

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    def get_stats(self) -> Dict[str, int]:
        """Return cumulative violation counts over all verified steps."""
        return {
            "total_collisions":           self.total_collisions,
            "total_red_light_violations": self.total_red_light_violations,
            "total_wrong_way_violations": self.total_wrong_way_violations,
        }

    # ------------------------------------------------------------------
    # Internal checkers
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

        A violation is recorded when:
          - At step t,   the car was at the LAST slot of an incoming segment
                         (segment ends at a circle intersection).
          - At step t+1, the car is on a DIFFERENT segment.
          - The signal at that intersection at step t was red for the car's direction.
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

            # Car must have actually moved to a new segment
            if prev_car.segment_id == curr_car.segment_id:
                continue

            # The segment must end at a real intersection (not at endpoint A)
            intersection_id = prev_seg.end
            if not self.topology.is_intersection(intersection_id):
                continue

            # Check if the signal was green for this car's direction
            signal = prev.signals.get(intersection_id)
            if signal is None:
                continue

            if not _direction_matches_phase(prev_car.direction, signal.green_direction):
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