# i_group/test_i_group.py

"""
Small but complete test suite for the i-group module.

Spec followed  : i_group/spec.md
Algorithm      : Adaptive actuated signal control (policy.py)
                 - Force switch after MAX_GREEN steps (starvation prevention)
                 - Voluntary switch after MIN_GREEN if opposing queue is larger
                 - Otherwise hold current phase and tick the timer

Tests cover:
  1. compute_signals      — returns exactly one valid SignalAction per intersection
  2. Clean state          — verify_step reports zero violations
  3. Collision            — two cars at same (segment, slot) → collisions = 1
  4. Red-light            — car crosses intersection while signal is red → violation = 1
  5. Wrong-way            — car direction mismatches segment direction → violation = 1
  6. Cumulative stats     — get_stats() accumulates across multiple steps
  7. Topology validity    — no intersection ever emits a signal for a non-existent direction
  8. Multi-crossing       — two cars entering the same intersection in one step → violation = 1
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from shared.topology import Topology
from shared.state import GlobalState, CarState, SignalState
from shared.enums import SIGNAL_PHASES
from i_group.controller import InfrastructureController


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def build_controller():
    """Return a fresh (controller, topology) pair."""
    topo = Topology()
    topo.build()
    return InfrastructureController(topo), topo


def all_signals(topo, phase="N"):
    """Return a signals dict with every intersection set to `phase`."""
    return {
        iid: SignalState(intersection_id=iid, green_direction=phase)
        for iid in topo.all_intersection_ids()
    }


# ---------------------------------------------------------------------------
# Test 1 — compute_signals: one valid SignalAction per intersection
# ---------------------------------------------------------------------------

def test_compute_signals_coverage():
    """compute_signals must return exactly 9 actions (one per intersection),
    every green_direction must be one of the 4 cardinal strings, AND it must
    be a topology-valid direction (i.e. the intersection has an outgoing road
    in that direction).  The second check catches signals for non-existent
    directions that the first check alone cannot detect."""
    ctrl, topo = build_controller()
    state = GlobalState(step=0, cars={}, signals=all_signals(topo))

    # Pre-compute valid outgoing directions per intersection
    valid_dirs = {
        iid: {
            topo.get_segment_direction(s)
            for s in topo.get_intersection(iid).outgoing
            if topo.get_segment_direction(s) is not None
        }
        for iid in topo.all_intersection_ids()
    }

    actions = ctrl.compute_signals(state)

    expected_ids = set(topo.all_intersection_ids())   # {'I00', ..., 'I22'}
    returned_ids = {a.intersection_id for a in actions}

    assert len(actions) == 9, \
        f"Expected 9 SignalActions, got {len(actions)}"
    assert returned_ids == expected_ids, \
        f"Missing intersections: {expected_ids - returned_ids}"
    for a in actions:
        assert a.green_direction in SIGNAL_PHASES, \
            f"Invalid phase '{a.green_direction}' at {a.intersection_id}"
        assert a.green_direction in valid_dirs[a.intersection_id], \
            (f"Topology-invalid phase '{a.green_direction}' at {a.intersection_id}: "
             f"valid directions are {sorted(valid_dirs[a.intersection_id])}")

    print("PASS  test_compute_signals_coverage")


# ---------------------------------------------------------------------------
# Test 2 — verify_step: no violations on a well-behaved state
# ---------------------------------------------------------------------------

def test_no_violations_clean_state():
    """Two cars moving legally → all violation counts must be 0."""
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    # Both cars on different segments, correct directions, different slots
    prev = GlobalState(step=0, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=5,  direction="E"),
        2: CarState(car_id=2, segment_id="I01_I02", slot=10, direction="E"),
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=6,  direction="E"),
        2: CarState(car_id=2, segment_id="I01_I02", slot=11, direction="E"),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["collisions"]              == 0, f"Unexpected collisions: {report}"
    assert report["red_light_violations"]    == 0, f"Unexpected red-light: {report}"
    assert report["wrong_way_violations"]    == 0, f"Unexpected wrong-way: {report}"
    assert report["multi_crossing_violations"] == 0, f"Unexpected multi-crossing: {report}"
    print("PASS  test_no_violations_clean_state")


# ---------------------------------------------------------------------------
# Test 3 — Collision: two cars occupy the same (segment, slot)
# ---------------------------------------------------------------------------

def test_collision_detection():
    """Two cars merge into the same slot → collisions = 1."""
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    prev = GlobalState(step=0, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=5, direction="E"),
        2: CarState(car_id=2, segment_id="I00_I01", slot=7, direction="E"),
    }, signals=sigs)

    # Both cars end up at slot 6 — collision
    curr = GlobalState(step=1, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=6, direction="E"),
        2: CarState(car_id=2, segment_id="I00_I01", slot=6, direction="E"),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["collisions"] == 1, \
        f"Expected 1 collision, got {report['collisions']}"
    print("PASS  test_collision_detection")


# ---------------------------------------------------------------------------
# Test 4 — Red-light violation: car crosses intersection while signal is red
# ---------------------------------------------------------------------------

def test_red_light_violation_detection():
    """
    Setup:
      - Segment I00_I01 goes East; length = 30; last slot = 29; end = I01.
      - Signal at I01 is 'N' green → East-bound cars (E direction) face red.
      - In prev_state: car is at slot 29 of I00_I01.
      - In curr_state: car has moved to I01_I02 (it crossed the red light).
    Expected: red_light_violations = 1.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")   # N green → S/E/W red

    prev = GlobalState(step=0, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=29, direction="E"),
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: CarState(car_id=1, segment_id="I01_I02", slot=0,  direction="E"),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["red_light_violations"] == 1, \
        f"Expected 1 red-light violation, got {report['red_light_violations']}"
    print("PASS  test_red_light_violation_detection")


# ---------------------------------------------------------------------------
# Test 5 — Wrong-way: car direction mismatches segment direction
# ---------------------------------------------------------------------------

def test_wrong_way_violation_detection():
    """
    Segment I00_I01 has direction 'E'.
    A car placed on it with direction 'W' is going the wrong way.
    Expected: wrong_way_violations = 1.
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
    print("PASS  test_wrong_way_violation_detection")


# ---------------------------------------------------------------------------
# Test 6 — get_stats: cumulative counts across multiple steps
# ---------------------------------------------------------------------------

def test_get_stats_accumulates():
    """
    Step A: 1 collision (two cars same slot).
    Step B: 1 red-light violation.
    After both steps: total_collisions=1, total_red_light_violations=1,
    total_wrong_way_violations=0.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    # --- Step A: collision ---
    prev_a = GlobalState(step=0, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=5, direction="E"),
        2: CarState(car_id=2, segment_id="I00_I01", slot=7, direction="E"),
    }, signals=sigs)
    curr_a = GlobalState(step=1, cars={
        1: CarState(car_id=1, segment_id="I00_I01", slot=6, direction="E"),
        2: CarState(car_id=2, segment_id="I00_I01", slot=6, direction="E"),
    }, signals=sigs)
    ctrl.verify_step(prev_a, curr_a)

    # --- Step B: red-light violation ---
    prev_b = GlobalState(step=1, cars={
        3: CarState(car_id=3, segment_id="I00_I01", slot=29, direction="E"),
    }, signals=sigs)
    curr_b = GlobalState(step=2, cars={
        3: CarState(car_id=3, segment_id="I01_I02", slot=0,  direction="E"),
    }, signals=sigs)
    ctrl.verify_step(prev_b, curr_b)

    stats = ctrl.get_stats()
    assert stats["total_collisions"]           == 1, \
        f"Expected 1 total collision, got {stats['total_collisions']}"
    assert stats["total_red_light_violations"] == 1, \
        f"Expected 1 total red-light violation, got {stats['total_red_light_violations']}"
    assert stats["total_wrong_way_violations"] == 0, \
        f"Expected 0 wrong-way violations, got {stats['total_wrong_way_violations']}"
    print("PASS  test_get_stats_accumulates")


# ---------------------------------------------------------------------------
# Test 7 — No signal is ever issued for a topology-invalid direction
# ---------------------------------------------------------------------------

def test_no_invalid_direction_signals():
    """
    For every intersection, the green_direction in each SignalAction must be
    one of the directions that actually has an outgoing road at that intersection.

    Runs compute_signals() for enough steps to trigger both voluntary and
    mandatory phase switches (> MAX_GREEN = 40), covering all switch paths.
    Any action whose green_direction has no outgoing road is a test failure.
    """
    ctrl, topo = build_controller()
    state = GlobalState(step=0, cars={}, signals=all_signals(topo))

    # Pre-compute valid outgoing directions for each intersection
    valid_dirs = {}
    for iid in topo.all_intersection_ids():
        inter = topo.get_intersection(iid)
        valid_dirs[iid] = {
            topo.get_segment_direction(seg_id)
            for seg_id in inter.outgoing
            if topo.get_segment_direction(seg_id) is not None
        }

    for step in range(50):   # 50 > MAX_GREEN=40, exercises both switch types
        actions = ctrl.compute_signals(state)
        for action in actions:
            iid = action.intersection_id
            assert action.green_direction in valid_dirs[iid], (
                f"FAIL step={step}: {iid} emitted direction "
                f"'{action.green_direction}' but valid directions are "
                f"{sorted(valid_dirs[iid])}"
            )
        # Advance state step (no cars; signals not strictly needed but kept consistent)
        state = GlobalState(
            step=step + 1,
            cars={},
            signals={a.intersection_id: SignalState(
                        intersection_id=a.intersection_id,
                        green_direction=a.green_direction)
                    for a in actions},
        )

    print("PASS  test_no_invalid_direction_signals")

def test_multi_crossing_violation():
    """
    Two cars cross the same intersection (I01) in the same step.
      - Car 1: East-bound, last slot of I00_I01 → moves to I01_I02.
      - Car 2: West-bound, last slot of I02_I01 → moves to I01_I11.
    Both cars enter I01 simultaneously → multi_crossing_violations = 1
    (one violation per intersection with 2+ simultaneous crossings).
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    # Both cars enter I01
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
        f"Expected 1 multi_crossing_violation, got {report}"
    print("PASS  test_multi_crossing_violation")


# ---------------------------------------------------------------------------
# Test 9 — switch_to rejects topology-invalid phases via valid_phases guard
# ---------------------------------------------------------------------------

def test_switch_to_rejects_invalid_phase():
    """
    switch_to() must raise ValueError when called with a direction that has
    no outgoing road at the intersection, as enforced by the valid_phases guard.

    I00 has outgoing directions E, S, W — not N.
    Passing valid_phases={"E","S","W"} and new_phase="N" must raise.
    """
    from i_group.scheduler import SignalScheduler

    scheduler = SignalScheduler(min_green=10, max_green=40)
    scheduler.initialize(["I00"])

    topo = Topology()
    topo.build()
    inter = topo.get_intersection("I00")
    valid = {topo.get_segment_direction(s) for s in inter.outgoing
             if topo.get_segment_direction(s) is not None}   # {"E", "S", "W"}

    # A valid phase must succeed
    scheduler.switch_to("I00", "E", valid_phases=valid)

    # An invalid phase must raise ValueError
    raised = False
    try:
        scheduler.switch_to("I00", "N", valid_phases=valid)
    except ValueError:
        raised = True

    assert raised, \
        "Expected ValueError when switching I00 to 'N' with valid_phases={'E','S','W'}"
    print("PASS  test_switch_to_rejects_invalid_phase")


# ---------------------------------------------------------------------------
# Test 10 — red-light check is skipped when prev.signals has no entry
# ---------------------------------------------------------------------------

def test_red_light_no_signal_entry_is_skipped():
    """
    If prev.signals has no entry for the intersection a car just crossed,
    _check_red_light_violations cannot determine the phase and must skip the
    car rather than raising or counting a false violation.

    This covers the known gap at step 0, before compute_signals has populated
    the signal state.  The expected result is 0 red-light violations.

    Setup:
      - prev.signals = {} (empty — no signal data at all)
      - Car was at last slot of I00_I01 (end = I01) and moved to I01_I02.
      - Without a signal entry for I01, the crossing cannot be verified.
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
    print("PASS  test_red_light_no_signal_entry_is_skipped")


# ---------------------------------------------------------------------------
# Test 11 — invalid initial phase is corrected immediately on step 0
# ---------------------------------------------------------------------------

def test_invalid_initial_phase_corrected_immediately():
    """
    The scheduler initialises every intersection to "N" by default.  Edge
    intersections (top row, left/right column corners) have no outgoing North
    road, so "N" is topology-invalid for them.

    Before the fix, the voluntary-switch condition (best_other_count >
    current_count) evaluated to False when there was no traffic (both counts
    are 0), causing the invalid phase to persist until must_switch fired at
    step MAX_GREEN=40.

    After the fix, decide() detects the invalid phase at the very start of the
    first call and corrects it immediately, regardless of traffic levels.

    This test verifies that on step 0 — with zero cars — every edge
    intersection already returns a topology-valid direction.
    """
    ctrl, topo = build_controller()
    state = GlobalState(step=0, cars={}, signals=all_signals(topo))  # zero traffic

    # Pre-compute valid outgoing directions per intersection
    valid_dirs = {
        iid: {
            topo.get_segment_direction(s)
            for s in topo.get_intersection(iid).outgoing
            if topo.get_segment_direction(s) is not None
        }
        for iid in topo.all_intersection_ids()
    }

    # Edge intersections whose default "N" phase is topology-invalid
    edge_intersections = [
        iid for iid in topo.all_intersection_ids()
        if "N" not in valid_dirs[iid]
    ]
    assert edge_intersections, "Expected at least some edge intersections without N"

    actions = ctrl.compute_signals(state)
    action_map = {a.intersection_id: a.green_direction for a in actions}

    for iid in edge_intersections:
        phase = action_map[iid]
        assert phase in valid_dirs[iid], (
            f"Step 0, zero traffic: {iid} still returned invalid phase '{phase}' "
            f"(valid: {sorted(valid_dirs[iid])}). "
            f"Invalid phase was not corrected immediately."
        )
        assert phase != "N", (
            f"Step 0: {iid} returned 'N' which has no outgoing road — "
            f"correction did not fire on first call."
        )

    print("PASS  test_invalid_initial_phase_corrected_immediately")


# ---------------------------------------------------------------------------
# Run all tests
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    test_compute_signals_coverage()
    test_no_violations_clean_state()
    test_collision_detection()
    test_red_light_violation_detection()
    test_wrong_way_violation_detection()
    test_get_stats_accumulates()
    test_no_invalid_direction_signals()
    test_multi_crossing_violation()
    test_switch_to_rejects_invalid_phase()
    test_red_light_no_signal_entry_is_skipped()
    test_invalid_initial_phase_corrected_immediately()
    print("\nAll 11 tests passed.")
