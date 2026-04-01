# i_group/scheduler.py

"""
Manages per-intersection signal phase timers.

Tracks how long the current green phase has been active at each intersection
and determines when a phase switch is allowed or required.
"""

from typing import Dict, List


class SignalScheduler:
    """
    Per-intersection phase timer.

    Rules:
      - A phase must stay green for at least MIN_GREEN steps (prevents thrashing).
      - A phase must switch after MAX_GREEN steps (prevents starvation).
      - Between those bounds, the policy decides whether to switch.
    """

    DEFAULT_MIN_GREEN = 10   # steps before a voluntary switch is allowed
    DEFAULT_MAX_GREEN = 40   # steps before a switch is forced

    def __init__(self,
                 min_green: int = DEFAULT_MIN_GREEN,
                 max_green: int = DEFAULT_MAX_GREEN):
        if min_green >= max_green:
            raise ValueError("min_green must be less than max_green")
        self.min_green = min_green
        self.max_green = max_green

        # intersection_id -> current green phase ("N" | "S" | "E" | "W")
        self._phase: Dict[str, str] = {}
        # intersection_id -> steps elapsed in current phase
        self._timer: Dict[str, int] = {}

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def initialize(self, intersection_ids: List[str],
                   default_phase: str = "N") -> None:
        """
        Set up all intersections with a default phase and timer = 0.

        Args:
            intersection_ids: All intersection IDs in the topology.
            default_phase:    Starting green direction ("N" | "S" | "E" | "W").
        """
        for iid in intersection_ids:
            self._phase[iid] = default_phase
            self._timer[iid] = 0

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def get_phase(self, intersection_id: str) -> str:
        """Return the current green phase for this intersection."""
        return self._phase[intersection_id]

    def get_timer(self, intersection_id: str) -> int:
        """Return steps elapsed in the current green phase."""
        return self._timer[intersection_id]

    def can_switch(self, intersection_id: str) -> bool:
        """Return True if minimum green time has been served (voluntary switch OK)."""
        return self._timer[intersection_id] >= self.min_green

    def must_switch(self, intersection_id: str) -> bool:
        """Return True if maximum green time is reached (switch is mandatory)."""
        return self._timer[intersection_id] >= self.max_green

    # ------------------------------------------------------------------
    # Mutations
    # ------------------------------------------------------------------

    def tick(self, intersection_id: str) -> None:
        """Advance the phase timer by one step (call when NOT switching)."""
        self._timer[intersection_id] += 1
    
    # update here
    def switch_to(self, intersection_id: str, new_phase: str) -> None:
        """
        Directly set the green phase to a specific direction and reset the timer.
        This method is used by the adaptive policy when it selects a specific
        target direction based on current traffic demand.
        """
        if new_phase not in ("N", "S", "E", "W"):
            raise ValueError(f"Invalid phase: {new_phase}")
        self._phase[intersection_id] = new_phase
        self._timer[intersection_id] = 0

    def switch(self, intersection_id: str) -> None:
        """
        Cycle the green phase (N → E → S → W → N) and reset the timer.
        Call this when a phase change is decided.
        """
        _cycle = {"N": "E", "E": "S", "S": "W", "W": "N"}
        current = self._phase[intersection_id]
        self._phase[intersection_id] = _cycle[current]
        self._timer[intersection_id] = 0

    # ------------------------------------------------------------------
    # Debug
    # ------------------------------------------------------------------

    def summary(self) -> str:
        lines = ["SignalScheduler state:"]
        for iid in sorted(self._phase):
            lines.append(
                f"  {iid}: phase={self._phase[iid]}  timer={self._timer[iid]}"
                f"  (can_switch={self.can_switch(iid)}, must_switch={self.must_switch(iid)})"
            )
        return "\n".join(lines)