# i_group/test_i_group.py

"""
Small but complete test suite for the i-group module.

Spec followed  : i_group/spec.md
Algorithm      : Adaptive actuated signal control (policy.py)
                 - Force switch after MAX_GREEN steps (starvation prevention)
                 - Voluntary switch after MIN_GREEN if opposing queue is larger
                 - Otherwise hold current phase and tick the timer

Tests cover:
  1. compute_signals  — returns exactly one valid SignalAction per intersection
  2. Clean state      — verify_step reports zero violations
  3. Collision        — two cars at same (segment, slot) → collisions = 1
  4. Red-light        — car crosses intersection while signal is red → violation = 1
  5. Wrong-way        — car direction mismatches segment direction → violation = 1
  6. Cumulative stats — get_stats() accumulates across multiple steps
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
    """compute_signals must return exactly 9 actions (one per intersection)
    and every green_direction must be 'N', 'S', 'E', or 'W'."""
    ctrl, topo = build_controller()
    state = GlobalState(step=0, cars={}, signals=all_signals(topo))

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

    assert report["collisions"]           == 0, f"Unexpected collisions: {report}"
    assert report["red_light_violations"] == 0, f"Unexpected red-light: {report}"
    assert report["wrong_way_violations"] == 0, f"Unexpected wrong-way: {report}"
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
# Run all tests
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    test_compute_signals_coverage()
    test_no_violations_clean_state()
    test_collision_detection()
    test_red_light_violation_detection()
    test_wrong_way_violation_detection()
    test_get_stats_accumulates()
    print("\nAll 6 tests passed.")
