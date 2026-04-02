# i_group/test_i_group.py

"""
Test suite for the i-group module (InfrastructureController).

Algorithm under test
--------------------
Adaptive actuated signal control (policy.py):
  - Max-Pressure   : always green the direction with the most approaching cars
  - Round-Robin    : tiebreaker cycles N → S → E → W for fairness
  - Min-Green      : phase is locked for at least MIN_GREEN steps (no thrashing)
  - Max-Green      : phase is forced to switch after MAX_GREEN steps (no starvation)
  - Topology guard : only directions with real incoming roads are ever emitted

Test sections
-------------
  A. Signal Generation & Topology   (tests  1 –  2)
  B. Violation Detection             (tests  3 –  9)
  C. Policy — Min / Max Green        (tests 10 – 11)
  D. Policy — Adaptive Strategy      (tests 12 – 13)
  E. Safety Guards & Validation      (tests 14 – 15)
  F. Edge Cases & Boundary Conditions(tests 16 – 21)
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from shared.topology import Topology
from shared.state import GlobalState, CarState, SignalState
from shared.validators import validate_signal_actions
from i_group.controller import InfrastructureController


# ============================================================================
# Shared helpers
# ============================================================================

def build_controller(min_green: int = 10, max_green: int = 40):
    """Return a fresh (InfrastructureController, Topology) pair."""
    topo = Topology()
    topo.build()
    return InfrastructureController(topo, min_green=min_green,
                                    max_green=max_green), topo


def all_signals(topo, phase: str = "N") -> dict:
    """Return a signals dict with every intersection set to *phase*."""
    return {
        iid: SignalState(intersection_id=iid, green_direction=phase)
        for iid in topo.all_intersection_ids()
    }


def valid_incoming_dirs(topo, intersection_id: str) -> set:
    """Return the set of directions that have an incoming road at *intersection_id*."""
    inter = topo.get_intersection(intersection_id)
    return {
        topo.get_segment_direction(s)
        for s in inter.incoming
        if topo.get_segment_direction(s) is not None
    }


# ============================================================================
# A. Signal Generation & Topology
# ============================================================================

def test_compute_signals_coverage():
    """
    [A-1] compute_signals() must return exactly 9 SignalActions — one per
    intersection — with no duplicates, and every green_direction must be:
      (a) structurally valid per the shared validator (cardinal string,
          intersection exists, no duplicate actions), AND
      (b) topology-valid for that specific intersection (has an incoming road).

    Check (b) catches the original bug where edge intersections could emit "N"
    despite having no northward exit.  Check (a) uses the official shared
    validator so results are consistent with the grading standard.
    """
    ctrl, topo = build_controller()
    state = GlobalState(step=0, cars={}, signals=all_signals(topo))

    valid_dirs = {iid: valid_incoming_dirs(topo, iid)
                  for iid in topo.all_intersection_ids()}

    actions = ctrl.compute_signals(state)
    returned_ids = {a.intersection_id for a in actions}

    # (a) Official shared validator — catches non-cardinal strings,
    #     unknown intersections, and duplicate actions in one call.
    errors = validate_signal_actions(actions, topo)
    assert not errors, \
        f"validate_signal_actions reported errors:\n" + "\n".join(errors)

    assert len(actions) == 9, \
        f"Expected 9 SignalActions, got {len(actions)}"
    assert returned_ids == set(topo.all_intersection_ids()), \
        f"Missing intersections: {set(topo.all_intersection_ids()) - returned_ids}"

    # (b) Topology-valid incoming direction per intersection
    for a in actions:
        assert a.green_direction in valid_dirs[a.intersection_id], \
            (f"Topology-invalid phase '{a.green_direction}' at {a.intersection_id}: "
             f"valid = {sorted(valid_dirs[a.intersection_id])}")

    print("PASS  [A-1] test_compute_signals_coverage")


def test_no_invalid_direction_signals():
    """
    [A-2] No intersection may emit a signal for a direction that has no
    incoming road, across 50 consecutive steps.

    Running 50 steps (> MAX_GREEN = 40) exercises both the voluntary-switch
    and the mandatory-switch code paths.
    """
    ctrl, topo = build_controller()
    state = GlobalState(step=0, cars={}, signals=all_signals(topo))

    valid_dirs = {iid: valid_incoming_dirs(topo, iid)
                  for iid in topo.all_intersection_ids()}

    for step in range(50):
        actions = ctrl.compute_signals(state)
        for a in actions:
            assert a.green_direction in valid_dirs[a.intersection_id], (
                f"FAIL step={step}: {a.intersection_id} emitted "
                f"'{a.green_direction}' — valid: {sorted(valid_dirs[a.intersection_id])}"
            )
        state = GlobalState(
            step=step + 1,
            cars={},
            signals={a.intersection_id: SignalState(
                         intersection_id=a.intersection_id,
                         green_direction=a.green_direction)
                     for a in actions},
        )

    print("PASS  [A-2] test_no_invalid_direction_signals")


# ============================================================================
# B. Violation Detection
# ============================================================================

def test_no_violations_clean_state():
    """
    [B-1] Two cars advancing legally on separate segments must produce zero
    counts across all four violation categories.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    prev = GlobalState(step=0, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=5,  direction="E"),
        2: CarState(car_id=2, segment_id="I01_I02", slot=10, direction="E"),
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=6,  direction="E"),
        2: CarState(car_id=2, segment_id="I01_I02", slot=11, direction="E"),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["collisions"]                == 0, f"Unexpected collision: {report}"
    assert report["red_light_violations"]      == 0, f"Unexpected red-light: {report}"
    assert report["wrong_way_violations"]      == 0, f"Unexpected wrong-way: {report}"
    assert report["multi_crossing_violations"] == 0, f"Unexpected multi-crossing: {report}"

    print("PASS  [B-1] test_no_violations_clean_state")


def test_collision_two_cars_same_slot():
    """
    [B-2] Two cars ending up at the same (segment, slot) must count as
    exactly 1 collision event.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    prev = GlobalState(step=0, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=5, direction="E"),
        2: CarState(car_id=2, segment_id="I00_I01", slot=7, direction="E"),
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=6, direction="E"),
        2: CarState(car_id=2, segment_id="I00_I01", slot=6, direction="E"),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["collisions"] == 1, \
        f"Expected 1 collision, got {report['collisions']}"

    print("PASS  [B-2] test_collision_two_cars_same_slot")


def test_collision_three_cars_same_slot_one_event():
    """
    [B-3] Three cars sharing the same (segment, slot) is still ONE collision
    event — the slot is multiply occupied, not two separate pairs.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    prev = GlobalState(step=0, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=5, direction="E"),
        2: CarState(car_id=2, segment_id="I00_I01", slot=7, direction="E"),
        3: CarState(car_id=3, segment_id="I00_I01", slot=9, direction="E"),
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=6, direction="E"),
        2: CarState(car_id=2, segment_id="I00_I01", slot=6, direction="E"),
        3: CarState(car_id=3, segment_id="I00_I01", slot=6, direction="E"),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["collisions"] == 1, \
        f"Expected 1 collision event for 3 cars at the same slot, got {report['collisions']}"

    print("PASS  [B-3] test_collision_three_cars_same_slot_one_event")


def test_red_light_violation():
    """
    [B-4] A car that crosses an intersection while its direction is red must
    be counted as exactly 1 red-light violation.

    Setup: signal at I01 is "N" (green) → East-bound cars face red.
    Car was at the last slot (29) of I00_I01 and moved onto I01_I02.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")   # N green → E / S / W are red

    prev = GlobalState(step=0, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=29, direction="E"),
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: CarState(car_id=1, segment_id="I01_I02", slot=0, direction="E"),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["red_light_violations"] == 1, \
        f"Expected 1 red-light violation, got {report['red_light_violations']}"

    print("PASS  [B-4] test_red_light_violation")


def test_wrong_way_violation():
    """
    [B-5] A car whose travel direction mismatches the segment's defined
    direction must be counted as exactly 1 wrong-way violation.

    Segment I00_I01 has direction "E"; a car on it with direction "W" is
    going the wrong way.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    prev = GlobalState(step=0, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=5, direction="W"),
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=5, direction="W"),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["wrong_way_violations"] == 1, \
        f"Expected 1 wrong-way violation, got {report['wrong_way_violations']}"

    print("PASS  [B-5] test_wrong_way_violation")


def test_multi_crossing_violation():
    """
    [B-6] Two cars crossing the same intersection in the same step must
    produce exactly 1 multi-crossing violation (one event per intersection,
    regardless of how many cars crossed it).

    Both car 1 (East-bound, I00_I01→I01_I02) and car 2
    (West-bound, I02_I01→I01_I11) cross I01 simultaneously.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    prev = GlobalState(step=0, cars={
        1: CarState(1, "I00_I01", 29, "E"),
        2: CarState(2, "I02_I01", 29, "W"),
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: CarState(1, "I01_I02", 0, "E"),
        2: CarState(2, "I01_I11", 0, "S"),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["multi_crossing_violations"] == 1, \
        f"Expected 1 multi-crossing violation, got {report['multi_crossing_violations']}"

    print("PASS  [B-6] test_multi_crossing_violation")


def test_get_stats_accumulates():
    """
    [B-7] get_stats() must accumulate violation counts correctly across
    multiple calls to verify_step().

    Step A produces 1 collision; Step B produces 1 red-light violation.
    Expected totals: collisions=1, red_light=1, wrong_way=0.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    # Step A — collision
    ctrl.verify_step(
        GlobalState(step=0, cars={
            1: CarState(car_id=1, segment_id="I00_I01", slot=5, direction="E"),
            2: CarState(car_id=2, segment_id="I00_I01", slot=7, direction="E"),
        }, signals=sigs),
        GlobalState(step=1, cars={
            1: CarState(car_id=1, segment_id="I00_I01", slot=6, direction="E"),
            2: CarState(car_id=2, segment_id="I00_I01", slot=6, direction="E"),
        }, signals=sigs),
    )

    # Step B — red-light violation
    ctrl.verify_step(
        GlobalState(step=1, cars={
            3: CarState(car_id=3, segment_id="I00_I01", slot=29, direction="E"),
        }, signals=sigs),
        GlobalState(step=2, cars={
            3: CarState(car_id=3, segment_id="I01_I02", slot=0, direction="E"),
        }, signals=sigs),
    )

    stats = ctrl.get_stats()
    assert stats["total_collisions"]           == 1, \
        f"Expected 1 total collision, got {stats['total_collisions']}"
    assert stats["total_red_light_violations"] == 1, \
        f"Expected 1 total red-light violation, got {stats['total_red_light_violations']}"
    assert stats["total_wrong_way_violations"] == 0, \
        f"Expected 0 wrong-way violations, got {stats['total_wrong_way_violations']}"

    print("PASS  [B-7] test_get_stats_accumulates")


# ============================================================================
# C. Policy — Min / Max Green Constraints
# ============================================================================

def test_min_green_prevents_early_switch():
    """
    [C-1] When the phase timer is below min_green, the policy must NOT switch
    even under heavy opposing traffic pressure.

    I11 phase = "N", timer = 5 (< min_green = 10).
    Five East-bound cars within approach threshold.
    Expected: I11 remains on "N".
    """
    ctrl, topo = build_controller()   # min_green=10, max_green=40

    ctrl.scheduler._timer["I11"] = 5  # below min_green

    cars = {
        i: CarState(car_id=i, segment_id="I10_I11", slot=15 + i, direction="E")
        for i in range(5)
    }
    state = GlobalState(step=0, cars=cars, signals=all_signals(topo))

    actions = ctrl.compute_signals(state)
    action_map = {a.intersection_id: a.green_direction for a in actions}

    assert action_map["I11"] == "N", \
        f"Expected 'N' to be held (timer 5 < min_green 10), got '{action_map['I11']}'"

    print("PASS  [C-1] test_min_green_prevents_early_switch")


def test_max_green_forces_switch():
    """
    [C-2] When the phase timer reaches max_green, the policy MUST switch
    regardless of traffic.

    I11 phase = "N", timer = 40 (= max_green = 40), no cars.
    Expected: I11 changes to any direction other than "N".
    """
    ctrl, topo = build_controller()   # min_green=10, max_green=40

    ctrl.scheduler._timer["I11"] = 40  # equals max_green

    state = GlobalState(step=0, cars={}, signals=all_signals(topo))

    actions = ctrl.compute_signals(state)
    action_map = {a.intersection_id: a.green_direction for a in actions}

    assert action_map["I11"] != "N", \
        f"Expected phase to change from 'N' (timer=40 >= max_green=40), got 'N'"

    print("PASS  [C-2] test_max_green_forces_switch")


# ============================================================================
# D. Policy — Adaptive Strategy (Max-Pressure + Round-Robin)
# ============================================================================

def test_max_pressure_selects_busiest_direction():
    """
    [D-1] With min_green=0, the voluntary switch fires on step 0.
    I11 (fully connected) starts on "N".  Three East-bound cars are within
    the approach threshold on I10_I11 (incoming E direction); no other
    direction has any cars.

    Expected: I11 switches to "E" — the direction with the highest pressure.

    Note: APPROACH_THRESHOLD = 15, segment length = 30.
          Cars must be at slot >= 15 to be counted.
    """
    topo = Topology()
    topo.build()
    ctrl = InfrastructureController(topo, min_green=0, max_green=40)

    cars = {
        i: CarState(car_id=i, segment_id="I10_I11", slot=15 + i, direction="E")
        for i in range(3)
    }
    state = GlobalState(step=0, cars=cars, signals=all_signals(topo))

    actions = ctrl.compute_signals(state)
    action_map = {a.intersection_id: a.green_direction for a in actions}

    assert action_map["I11"] == "E", \
        f"Expected 'E' (3 approaching cars), got '{action_map['I11']}'"

    print("PASS  [D-1] test_max_pressure_selects_busiest_direction")


def test_round_robin_tiebreaker_rotates_directions():
    """
    [D-2] With equal pressure (zero cars) and max_green=2, the policy is
    forced to switch frequently.  When all counts are 0, cycle_priority()
    drives the selection.

    Two assertions are made at I11 (fully connected):
      1. Coverage  — all four directions appear over 12 steps (no starvation).
      2. Order     — each phase transition follows the defined cycle
                     N → S → E → W → N (matches _CYCLE in scheduler.py).

    Expected 12-step sequence: N N S S S E E E W W W N
    Expected transitions     : N→S, S→E, E→W, W→N
    """
    topo = Topology()
    topo.build()
    ctrl = InfrastructureController(topo, min_green=1, max_green=2)

    state = GlobalState(step=0, cars={}, signals=all_signals(topo))
    collected = []   # full phase sequence for I11

    for step in range(12):
        actions = ctrl.compute_signals(state)
        action_map = {a.intersection_id: a.green_direction for a in actions}
        collected.append(action_map["I11"])
        state = GlobalState(
            step=step + 1,
            cars={},
            signals={a.intersection_id: SignalState(
                         intersection_id=a.intersection_id,
                         green_direction=a.green_direction)
                     for a in actions},
        )

    # 1. Coverage — every direction must appear
    assert set(collected) == {"N", "S", "E", "W"}, (
        f"Expected all 4 directions at I11 over 12 steps, got {sorted(set(collected))}"
    )

    # 2. Order — transitions must follow N→S→E→W→N
    expected_cycle = ["N", "S", "E", "W"]
    transitions = [
        (collected[i - 1], collected[i])
        for i in range(1, len(collected))
        if collected[i] != collected[i - 1]
    ]
    for i, (from_phase, to_phase) in enumerate(transitions):
        expected_from = expected_cycle[i % 4]
        expected_to   = expected_cycle[(i + 1) % 4]
        assert from_phase == expected_from and to_phase == expected_to, (
            f"Transition {i + 1}: expected {expected_from}→{expected_to}, "
            f"got {from_phase}→{to_phase}  (full sequence: {collected})"
        )

    print("PASS  [D-2] test_round_robin_tiebreaker_rotates_directions")


# ============================================================================
# E. Safety Guards & Validation
# ============================================================================

def test_switch_to_rejects_topology_invalid_phase():
    """
    [E-1] switch_to() must raise ValueError when the requested phase has no
    incoming road at the intersection, as enforced by the valid_phases guard.

    I00 has incoming directions E, N, W — not S.
    Passing valid_phases={"E","N","W"} and new_phase="S" must raise.
    A valid phase ("E") must succeed without error.
    """
    from i_group.scheduler import SignalScheduler

    scheduler = SignalScheduler(min_green=10, max_green=40)
    scheduler.initialize(["I00"])

    topo = Topology()
    topo.build()
    valid = valid_incoming_dirs(topo, "I00")   # {"E", "N", "W"}

    # Valid phase must succeed
    scheduler.switch_to("I00", "E", valid_phases=valid)

    # Invalid phase must raise
    raised = False
    try:
        scheduler.switch_to("I00", "S", valid_phases=valid)
    except ValueError:
        raised = True

    assert raised, \
        "Expected ValueError when switching I00 to 'S' with valid_phases={'E','N','W'}"

    print("PASS  [E-1] test_switch_to_rejects_topology_invalid_phase")



def test_invalid_initial_phase_corrected_immediately():
    """
    [E-2] The scheduler initialises every intersection to "N" by default.
    Edge intersections (top row, side columns) have no incoming North road,
    so "N" is topology-invalid for them.

    Before the fix this persisted until must_switch fired at step MAX_GREEN=40.
    After the fix, decide() corrects the phase on the very first call — even
    with zero cars and zero traffic pressure.

    This test verifies step-0 correction for all edge intersections.
    """
    ctrl, topo = build_controller()
    state = GlobalState(step=0, cars={}, signals=all_signals(topo))

    valid_dirs = {iid: valid_incoming_dirs(topo, iid)
                  for iid in topo.all_intersection_ids()}

    edge_intersections = [iid for iid in topo.all_intersection_ids()
                          if "N" not in valid_dirs[iid]]
    assert edge_intersections, "Expected at least some edge intersections without N"

    actions = ctrl.compute_signals(state)
    action_map = {a.intersection_id: a.green_direction for a in actions}

    for iid in edge_intersections:
        phase = action_map[iid]
        assert phase in valid_dirs[iid], (
            f"Step 0, zero traffic: {iid} returned invalid phase '{phase}' "
            f"(valid: {sorted(valid_dirs[iid])}) — correction did not fire."
        )
        assert phase != "N", (
            f"Step 0: {iid} returned 'N' which has no incoming road — "
            f"correction was not immediate."
        )

    print("PASS  [E-2] test_invalid_initial_phase_corrected_immediately")


# ============================================================================
# F. Edge Cases & Boundary Conditions
# ============================================================================

def test_red_light_missing_signal_entry_is_skipped():
    """
    [F-1] If prev.signals has no entry for an intersection (e.g. step 0 before
    compute_signals has ever been called), the red-light check cannot determine
    the phase and must skip the crossing — zero false positives.

    A car that crosses I01 while prev.signals = {} must not be flagged.
    """
    ctrl, _ = build_controller()

    prev = GlobalState(step=0, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=29, direction="E"),
    }, signals={})   # no signal entries — simulates step 0

    curr = GlobalState(step=1, cars={
        1: CarState(car_id=1, segment_id="I01_I02", slot=0, direction="E"),
    }, signals={})

    report = ctrl.verify_step(prev, curr)

    assert report["red_light_violations"] == 0, (
        f"Expected 0 red-light violations when signal data is absent, "
        f"got {report['red_light_violations']}"
    )

    print("PASS  [F-1] test_red_light_missing_signal_entry_is_skipped")


def test_scheduler_rejects_invalid_green_bounds():
    """
    [F-2] SignalScheduler.__init__ must raise ValueError when
    min_green >= max_green.  Both equal and greater values must be rejected
    to prevent divide-by-zero and infinite-hold scenarios.
    """
    from i_group.scheduler import SignalScheduler

    raised_equal = False
    try:
        SignalScheduler(min_green=10, max_green=10)
    except ValueError:
        raised_equal = True

    raised_greater = False
    try:
        SignalScheduler(min_green=20, max_green=10)
    except ValueError:
        raised_greater = True

    assert raised_equal,   "Expected ValueError for min_green == max_green"
    assert raised_greater, "Expected ValueError for min_green > max_green"

    print("PASS  [F-2] test_scheduler_rejects_invalid_green_bounds")


def test_car_below_threshold_not_counted():
    """
    [F-3] _count_approaching() only counts cars at:
        slot >= segment_length - APPROACH_THRESHOLD  (= 30 - 15 = 15)

    A car at slot 14 (one below the threshold) must NOT be counted and must
    NOT trigger a phase switch — even with min_green=0.
    """
    topo = Topology()
    topo.build()
    ctrl = InfrastructureController(topo, min_green=0, max_green=40)

    # Car at slot 14 — just below the approach threshold of 15
    cars = {1: CarState(car_id=1, segment_id="I10_I11", slot=14, direction="E")}
    state = GlobalState(step=0, cars=cars, signals=all_signals(topo))

    actions = ctrl.compute_signals(state)
    action_map = {a.intersection_id: a.green_direction for a in actions}

    assert action_map["I11"] != "E", (
        f"Car at slot 14 (below threshold 15) should not trigger a switch to 'E', "
        f"got '{action_map['I11']}'"
    )

    print("PASS  [F-3] test_car_below_threshold_not_counted")


def test_inactive_car_not_counted():
    """
    [F-4] _count_approaching() must skip cars where active=False.
    An inactive car within the approach threshold must not influence
    the pressure calculation or cause a phase switch.
    """
    topo = Topology()
    topo.build()
    ctrl = InfrastructureController(topo, min_green=0, max_green=40)

    # Inactive car within threshold — must be ignored
    cars = {
        1: CarState(car_id=1, segment_id="I10_I11", slot=20,
                    direction="E", active=False)
    }
    state = GlobalState(step=0, cars=cars, signals=all_signals(topo))

    actions = ctrl.compute_signals(state)
    action_map = {a.intersection_id: a.green_direction for a in actions}

    assert action_map["I11"] != "E", (
        f"Inactive car must not influence pressure — expected no switch to 'E', "
        f"got '{action_map['I11']}'"
    )

    print("PASS  [F-4] test_inactive_car_not_counted")


def test_new_car_in_curr_state_not_flagged():
    """
    [F-5] A car that first appears in curr_state (no matching entry in
    prev_state) has no prior position to compare against.
    verify_step() must skip it silently — zero violations of any kind.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    prev = GlobalState(step=0, cars={}, signals=sigs)   # no cars in prev

    curr = GlobalState(step=1, cars={
        1: CarState(car_id=1, segment_id="I01_I02", slot=0, direction="E"),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["collisions"]                == 0, f"Unexpected collision: {report}"
    assert report["red_light_violations"]      == 0, f"Unexpected red-light: {report}"
    assert report["wrong_way_violations"]      == 0, f"Unexpected wrong-way: {report}"
    assert report["multi_crossing_violations"] == 0, f"Unexpected multi-crossing: {report}"

    print("PASS  [F-5] test_new_car_in_curr_state_not_flagged")


def test_crossing_endpoint_A_not_flagged():
    """
    [F-6] Endpoint A is the square start/end node — not a circle intersection.
    A car leaving the last slot of I00_A (which ends at A) must NOT be counted
    as a red-light violation.  Red-light checks only apply at circle intersections.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "E")   # E green → W-bound cars face red

    # Segment I00_A: direction W, length=2, end=A (not an intersection)
    prev = GlobalState(step=0, cars={
        1: CarState(car_id=1, segment_id="I00_A", slot=1, direction="W"),
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: CarState(car_id=1, segment_id="I00_A", slot=1,
                    direction="W", active=False),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["red_light_violations"] == 0, (
        f"Crossing endpoint A must not be flagged as red-light violation, "
        f"got {report['red_light_violations']}"
    )

    print("PASS  [F-6] test_crossing_endpoint_A_not_flagged")


# ============================================================================
# Run all tests
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("i-group test suite")
    print("=" * 60)

    print("\n--- A. Signal Generation & Topology ---")
    test_compute_signals_coverage()
    test_no_invalid_direction_signals()

    print("\n--- B. Violation Detection ---")
    test_no_violations_clean_state()
    test_collision_two_cars_same_slot()
    test_collision_three_cars_same_slot_one_event()
    test_red_light_violation()
    test_wrong_way_violation()
    test_multi_crossing_violation()
    test_get_stats_accumulates()

    print("\n--- C. Policy — Min / Max Green ---")
    test_min_green_prevents_early_switch()
    test_max_green_forces_switch()

    print("\n--- D. Policy — Adaptive Strategy ---")
    test_max_pressure_selects_busiest_direction()
    test_round_robin_tiebreaker_rotates_directions()

    print("\n--- E. Safety Guards & Validation ---")
    test_switch_to_rejects_topology_invalid_phase()
    test_invalid_initial_phase_corrected_immediately()

    print("\n--- F. Edge Cases & Boundary Conditions ---")
    test_red_light_missing_signal_entry_is_skipped()
    test_scheduler_rejects_invalid_green_bounds()
    test_car_below_threshold_not_counted()
    test_inactive_car_not_counted()
    test_new_car_in_curr_state_not_flagged()
    test_crossing_endpoint_A_not_flagged()

    print("\n" + "=" * 60)
    print("All 21 tests passed.")
    print("=" * 60)
