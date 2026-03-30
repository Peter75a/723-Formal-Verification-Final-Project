# i_group/policy.py

"""
Signal phase decision policy.

Decides which direction (NS or EW) should be green at each intersection,
using an adaptive strategy based on the number of cars approaching.
"""

from typing import TYPE_CHECKING

from shared.topology import Topology, SLOTS_PER_SEGMENT
from shared.state import GlobalState

if TYPE_CHECKING:
    from i_group.scheduler import SignalScheduler


# Cars within this many slots of the intersection end count as "approaching"
APPROACH_THRESHOLD = 15   # 15 slots ≈ 0.25 mile


def _direction_matches_phase(direction: str, phase: str) -> bool:
    """Return True if a car travelling `direction` is covered by `phase`."""
    if phase == "NS":
        return direction in ("N", "S")
    if phase == "EW":
        return direction in ("E", "W")
    return False


class SignalPolicy:
    """
    Adaptive signal policy for one or all intersections.

    Each call to `decide()` returns the green phase for one intersection
    and advances the scheduler timer for that intersection.

    Strategy:
      1. If the current phase has hit MAX_GREEN → force switch.
      2. If MIN_GREEN has been served AND the other direction has more
         approaching cars → voluntary switch.
      3. Otherwise → keep current phase, tick the timer.
    """

    def __init__(self, topology: Topology, scheduler: "SignalScheduler"):
        self.topology = topology
        self.scheduler = scheduler

    # ------------------------------------------------------------------
    # Main interface
    # ------------------------------------------------------------------

    def decide(self, intersection_id: str, state: GlobalState) -> str:
        """
        Return the green phase ("NS" or "EW") for this intersection at this step.
        Side-effect: advances or resets the scheduler timer.

        Args:
            intersection_id: e.g. "I11"
            state:           Current global simulation state.

        Returns:
            "NS" or "EW"
        """
        current = self.scheduler.get_phase(intersection_id)

        # Mandatory switch — phase has been on too long
        if self.scheduler.must_switch(intersection_id):
            self.scheduler.switch(intersection_id)
            return self.scheduler.get_phase(intersection_id)

        # Voluntary switch — look at traffic if minimum time served
        if self.scheduler.can_switch(intersection_id):
            ns_count = self._count_approaching(intersection_id, "NS", state)
            ew_count = self._count_approaching(intersection_id, "EW", state)

            current_count = ns_count if current == "NS" else ew_count
            other_count   = ew_count if current == "NS" else ns_count

            if other_count > current_count:
                self.scheduler.switch(intersection_id)
                return self.scheduler.get_phase(intersection_id)

        # Keep current phase, advance timer
        self.scheduler.tick(intersection_id)
        return current

    # ------------------------------------------------------------------
    # Helper
    # ------------------------------------------------------------------

    def _count_approaching(self, intersection_id: str,
                           phase: str, state: GlobalState) -> int:
        """
        Count active cars on incoming segments of `intersection_id` that:
          - travel in a direction covered by `phase`
          - are within APPROACH_THRESHOLD slots of the intersection

        A car at slot >= (segment_length - APPROACH_THRESHOLD) is "close enough".
        """
        inter = self.topology.get_intersection(intersection_id)
        if inter is None:
            return 0

        # Collect incoming segment IDs matching this phase
        relevant = {
            seg_id
            for seg_id in inter.incoming
            if _direction_matches_phase(
                self.topology.get_segment_direction(seg_id) or "", phase
            )
        }

        count = 0
        for car in state.cars.values():
            if not car.active:
                continue
            if car.segment_id not in relevant:
                continue
            seg = self.topology.get_segment_by_id(car.segment_id)
            if seg and car.slot >= seg.length - APPROACH_THRESHOLD:
                count += 1
        return count