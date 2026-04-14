# simulator/simulator.py

"""
Simulator -- integrates i-group and v-group in a unified step loop.

Step order each tick:
  1. i_ctrl.compute_signals(state)       -> signal_actions
  2. Write signal_actions into state.signals
  3. v_ctrl.decide_actions(state)        -> car_actions
  4. deep-copy state as prev_state
  5. apply_car_actions(...)              -> new_state
  6. update new_state.congestion
  7. v_ctrl.verify_step(prev, new)       -> v_report
  8. i_ctrl.verify_step(prev, new)       -> i_report
  9. Assert both reports agree; log step summary
 10. respawn: if a car just finished, inject a new car when A_I00 slot 0 is free

Car lifecycle:
  - Cars spawn at segment "A_I00", slot 0, direction "E"
  - When a car's active flag becomes False, it completed the tour
  - Simulator tries to maintain `num_cars` active cars by respawning
  - Throughput = completed tours / simulated hours elapsed

How to run (from project root):
    python -m simulator.test_integration
"""

import copy
import os
from typing import Dict, List, Optional

from shared.topology import Topology
from shared.state import GlobalState, CarState, SignalState
from shared.enums import SPEED_MOVE

from simulator.movement_rules import apply_car_actions, compute_congestion


# Seconds per simulation step (spec: each control step = 2 s)
STEP_SECONDS = 2
STEPS_PER_HOUR = 3600 // STEP_SECONDS   # 1800 steps = 1 simulated hour

# A_I00 is a 2-slot segment; at most 2 cars can be on it simultaneously.
_ENTRY_SEGMENT = "A_I00"
_ENTRY_SLOT    = 0

# Rotate starting destinations so cars don't all chase the same waypoint.
_DEST_CYCLE = ["B", "C", "D"]


# ---------------------------------------------------------------------------
# Logger: tees output to stdout and optionally a log file
# ---------------------------------------------------------------------------

class _Logger:
    """
    Writes every message to stdout AND, optionally, to a log file.

    Usage (as context manager):
        with _Logger("logs/test1.txt") as logger:
            logger.log("hello")   # prints + writes to file
    """

    def __init__(self, file_path: Optional[str] = None):
        self._fh = None
        if file_path:
            os.makedirs(os.path.dirname(os.path.abspath(file_path)), exist_ok=True)
            self._fh = open(file_path, "w", encoding="utf-8")

    def log(self, msg: str = "") -> None:
        """Print to stdout and write to file (if open)."""
        print(msg)
        if self._fh:
            self._fh.write(msg + "\n")

    def close(self) -> None:
        if self._fh:
            self._fh.close()
            self._fh = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


# ---------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------

class Simulator:
    """
    Runs the integrated traffic simulation.

    Args:
        topo:   Built Topology instance.
        v_ctrl: VehicleController (v-group).
        i_ctrl: InfrastructureController (i-group).
    """

    def __init__(self, topo: Topology, v_ctrl, i_ctrl):
        self.topo   = topo
        self.v_ctrl = v_ctrl
        self.i_ctrl = i_ctrl

        self.completed_tours: int = 0
        self.mismatch_count:  int = 0

        # Respawn bookkeeping
        self._next_car_id: int = 1
        self._target_cars: int = 0   # set by run()

        # Active logger (set for the duration of run())
        self._logger: Optional[_Logger] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(self, steps: int, num_cars: int,
            verbose: bool = True,
            log_file: Optional[str] = None) -> Dict:
        """
        Run the simulation for a given number of steps.

        Args:
            steps:    Number of time steps to simulate.
            num_cars: Number of cars to keep active at all times.
                      Capped to 2 for initial spawn (A_I00 capacity);
                      remaining cars enter via respawn as slot 0 frees up.
            verbose:  If True, print per-step log (signals, positions,
                      violation status). Set False for quiet/throughput runs.
            log_file: Optional path for a log file. When given, ALL output
                      that would go to stdout is ALSO written to this file.
                      For verbose=True runs this includes every step.
                      For verbose=False runs this includes only the summary.

        Returns:
            Summary dict with throughput and cumulative violation counts.
        """
        self._target_cars = num_cars
        state = self._make_initial_state(num_cars)

        with _Logger(log_file) as logger:
            self._logger = logger

            if verbose:
                logger.log("=" * 60)
                logger.log("  ECEN 723 Traffic Simulation -- Phase B Integration")
                logger.log(f"  {num_cars} active car(s) target, {steps} steps")
                if log_file:
                    logger.log(f"  Log file: {os.path.abspath(log_file)}")
                logger.log("=" * 60)

            for _ in range(steps):
                state = self._step(state, verbose=verbose)

            simulated_hours = (steps * STEP_SECONDS) / 3600
            throughput = (self.completed_tours / simulated_hours
                          if simulated_hours > 0 else 0)

            v_stats = self.v_ctrl.get_stats()
            i_stats = self.i_ctrl.get_stats()

            summary = {
                "steps":             steps,
                "num_cars":          num_cars,
                "completed_tours":   self.completed_tours,
                "throughput_per_hr": round(throughput, 2),
                "mismatches":        self.mismatch_count,
                "v_stats":           v_stats,
                "i_stats":           i_stats,
            }

            self._print_summary(summary)

            if log_file:
                logger.log(f"\n(Full output saved to: {os.path.abspath(log_file)})")

            self._logger = None

        return summary

    # ------------------------------------------------------------------
    # Internal: single step
    # ------------------------------------------------------------------

    def _step(self, state: GlobalState, verbose: bool = True) -> GlobalState:
        step = state.step

        # ── 1. i-group computes signals ────────────────────────────────
        signal_actions = self.i_ctrl.compute_signals(state)

        # ── 2. Write signal decisions into state so v-group sees them ──
        state.signals = {
            a.intersection_id: SignalState(
                intersection_id=a.intersection_id,
                green_direction=a.green_direction,
            )
            for a in signal_actions
        }

        # ── 3. v-group decides car actions (sees updated signals) ──────
        car_actions = self.v_ctrl.decide_actions(state)

        # ── 4. Deep-copy current state as prev_state ───────────────────
        prev_state = copy.deepcopy(state)

        # ── 5. Apply car actions -> new state ──────────────────────────
        new_state = apply_car_actions(state, car_actions, signal_actions, self.topo)

        # ── 6. Update congestion map for next step's v-group read ──────
        new_state.congestion = compute_congestion(new_state, self.topo)

        # ── 7 & 8. Both groups verify ──────────────────────────────────
        v_report = self.v_ctrl.verify_step(prev_state, new_state)
        i_report = self.i_ctrl.verify_step(prev_state, new_state)

        # ── 9. Check agreement & log ───────────────────────────────────
        agreed = self._check_agreement(v_report, i_report)
        if verbose:
            self._log_step(step, signal_actions, car_actions,
                           new_state, v_report, i_report, agreed)

        # Track completed tours
        for car_id, car in new_state.cars.items():
            prev_car = prev_state.cars.get(car_id)
            if prev_car and prev_car.active and not car.active:
                self.completed_tours += 1
                self.v_ctrl.record_completed_tour()
                if verbose and self._logger:
                    self._logger.log(
                        f"   *** Car {car_id} completed full tour! "
                        f"(total={self.completed_tours}) ***"
                    )

        # ── 10. Respawn: inject new cars to maintain fleet size ────────
        self._try_respawn(new_state)

        return new_state

    # ------------------------------------------------------------------
    # Internal: respawning
    # ------------------------------------------------------------------

    def _try_respawn(self, state: GlobalState) -> None:
        """
        Inject new cars at A_I00 slot 0 to keep active car count
        at self._target_cars.

        A_I00 has 2 slots. We only inject at slot 0 (the entry point).
        If slot 0 is occupied, we wait for the next step.
        """
        active_count = sum(1 for c in state.cars.values() if c.active)
        while active_count < self._target_cars:
            slot0_taken = any(
                c.active and c.segment_id == _ENTRY_SEGMENT and c.slot == _ENTRY_SLOT
                for c in state.cars.values()
            )
            if slot0_taken:
                break   # can't inject now; try next step

            car_id = self._next_car_id
            self._next_car_id += 1
            dest = _DEST_CYCLE[(car_id - 1) % len(_DEST_CYCLE)]
            state.cars[car_id] = CarState(
                car_id=car_id,
                segment_id=_ENTRY_SEGMENT,
                slot=_ENTRY_SLOT,
                direction="E",
                speed=SPEED_MOVE,
                destinations_visited=[],
                current_destination=dest,
            )
            active_count += 1

    # ------------------------------------------------------------------
    # Internal: helpers
    # ------------------------------------------------------------------

    def _make_initial_state(self, num_cars: int) -> GlobalState:
        """
        Spawn the initial fleet.

        A_I00 has 2 slots (indices 0 and 1). We place at most 2 cars
        initially; remaining cars are injected via respawn logic as
        slot 0 becomes free.
        """
        signals = {
            iid: SignalState(intersection_id=iid, green_direction="E")
            for iid in self.topo.all_intersection_ids()
        }

        initial_count = min(num_cars, 2)   # A_I00 only has 2 slots
        cars = {}
        for i in range(initial_count):
            car_id = self._next_car_id
            self._next_car_id += 1
            dest = _DEST_CYCLE[(car_id - 1) % len(_DEST_CYCLE)]
            cars[car_id] = CarState(
                car_id=car_id,
                segment_id=_ENTRY_SEGMENT,
                slot=i,               # slot 0, then slot 1
                direction="E",
                speed=SPEED_MOVE,
                destinations_visited=[],
                current_destination=dest,
            )

        return GlobalState(step=0, cars=cars, signals=signals, congestion={})

    _AGREE_KEYS = ("collisions", "red_light_violations", "wrong_way_violations",
                   "multi_crossing_violations")

    def _check_agreement(self, v_report: Dict, i_report: Dict) -> bool:
        agreed = all(v_report.get(k, 0) == i_report.get(k, 0)
                     for k in self._AGREE_KEYS)
        if not agreed:
            self.mismatch_count += 1
        return agreed

    def _log_step(self, step, signal_actions, car_actions,
                  new_state, v_report, i_report, agreed):
        lg = self._logger.log if self._logger else print

        lg(f"\n-- Step {step} " + "-" * 44)

        # Signals at key intersections
        sig_map = {a.intersection_id: a.green_direction for a in signal_actions}
        for iid in ["I00", "I01", "I11"]:
            lg(f"   Signal {iid}: green={sig_map.get(iid, '?')}")

        # Car positions after move
        action_map = {a.car_id: a.action for a in car_actions}
        for car_id, car in sorted(new_state.cars.items()):
            if car.active:
                act     = action_map.get(car_id, "STAY")
                visited = ",".join(car.destinations_visited) or "-"
                lg(f"   Car {car_id}: {act:12s}  "
                   f"seg={car.segment_id}  slot={car.slot:2d}  "
                   f"visited=[{visited}]  next={car.current_destination}")
            else:
                lg(f"   Car {car_id}: DONE (inactive)")

        # Violation status
        v_clean = all(v_report.get(k, 0) == 0 for k in self._AGREE_KEYS)
        i_clean = all(i_report.get(k, 0) == 0 for k in self._AGREE_KEYS)
        status  = "CLEAN"  if (v_clean and i_clean) else "VIOLATIONS"
        agree   = "AGREE"  if agreed                else "MISMATCH"
        lg(f"   Verify: {status} | v/i-group: {agree}")

        if not (v_clean and i_clean):
            for k in self._AGREE_KEYS:
                v_val = v_report.get(k, 0)
                i_val = i_report.get(k, 0)
                if v_val or i_val:
                    lg(f"     {k}: v={v_val} i={i_val}")

    def _print_summary(self, summary: Dict):
        lg = self._logger.log if self._logger else print
        simulated_seconds = summary["steps"] * STEP_SECONDS

        lg("\n" + "=" * 60)
        lg("Final Summary")
        lg(f"  Steps simulated   : {summary['steps']} "
           f"({simulated_seconds}s = {simulated_seconds / 3600:.3f} hr)")
        lg(f"  Target fleet size : {summary['num_cars']} cars")
        lg(f"  Completed tours   : {summary['completed_tours']}")
        lg(f"  Throughput        : {summary['throughput_per_hr']} tours/hr")
        lg(f"  Report mismatches : {summary['mismatches']}")
        lg(f"  v_group stats     : {summary['v_stats']}")
        lg(f"  i_group stats     : {summary['i_stats']}")
        lg("=" * 60)
