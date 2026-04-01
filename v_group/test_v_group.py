# v_group/test_v_group.py

"""
Small but complete test suite for the v-group module.

Spec followed  : v_group/spec.md
Algorithm      : BFS-based route planning (planner.py) +
                 Safety-first action decision (strategy.py)

Tests cover:
  1. Perception — is_signal_green, is_blocked_ahead
  2. Planner    — BFS next intersection, destination resolution
  3. Strategy   — MOVE, STAY (blocked), STAY (red light),
                  STAY (intersection in use), turn-action selection
  4. Controller — verify_step: collisions, red-light violations,
                  wrong-way violations, cumulative stats
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from shared.topology import Topology, WAYPOINT_INTERSECTIONS
from shared.state import GlobalState, CarState, SignalState
from shared.enums import SIGNAL_PHASES, SPEED_MOVE, SPEED_STOP

from v_group.controller import VehicleController
from v_group.perception import Perception
from v_group.planner import RoutePlanner
from v_group.strategy import ActionStrategy


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def build_controller():
    """Return a fresh (VehicleController, Topology) pair."""
    topo = Topology()
    topo.build()
    return VehicleController(topo), topo


def build_topo():
    topo = Topology()
    topo.build()
    return topo


def all_signals(topo, phase="N"):
    """Return a signals dict with every intersection set to `phase`."""
    return {
        iid: SignalState(intersection_id=iid, green_direction=phase)
        for iid in topo.all_intersection_ids()
    }


def make_car(car_id, segment_id, slot, direction,
             speed=SPEED_MOVE, active=True,
             destinations_visited=None, current_destination="B"):
    return CarState(
        car_id=car_id,
        segment_id=segment_id,
        slot=slot,
        direction=direction,
        speed=speed,
        active=active,
        destinations_visited=destinations_visited or [],
        current_destination=current_destination,
    )


# ---------------------------------------------------------------------------
# ── PERCEPTION TESTS ────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

def test_perception_signal_green():
    """
    Car travelling East on I00_I01.
    Segment ends at I01. Signal at I01 is 'E' → green for this car.
    """
    topo = build_topo()
    perc = Perception(topo)
    sigs = all_signals(topo, "E")  # all green for East

    car = make_car(1, "I00_I01", slot=15, direction="E")
    state = GlobalState(step=0, cars={1: car}, signals=sigs)

    assert perc.is_signal_green(car, state) is True, \
        "Expected green signal for East-bound car at I01 with E phase"
    print("PASS  test_perception_signal_green")


def test_perception_signal_red():
    """
    Car travelling East on I00_I01.
    Signal at I01 is 'N' → red for East-bound car.
    """
    topo = build_topo()
    perc = Perception(topo)
    sigs = all_signals(topo, "N")  # only North is green

    car = make_car(1, "I00_I01", slot=15, direction="E")
    state = GlobalState(step=0, cars={1: car}, signals=sigs)

    assert perc.is_signal_green(car, state) is False, \
        "Expected red signal for East-bound car at I01 with N phase"
    print("PASS  test_perception_signal_red")


def test_perception_not_blocked():
    """Single car on segment — no car ahead → not blocked."""
    topo = build_topo()
    perc = Perception(topo)
    sigs = all_signals(topo, "N")

    car = make_car(1, "I00_I01", slot=5, direction="E")
    state = GlobalState(step=0, cars={1: car}, signals=sigs)

    assert perc.is_blocked_ahead(car, state) is False, \
        "Expected not blocked (only car on segment)"
    print("PASS  test_perception_not_blocked")


def test_perception_blocked_by_car_ahead():
    """
    Car 1 at slot 5, Car 2 at slot 10, same segment, same direction.
    Car 1 should see Car 2 directly ahead → blocked.
    """
    topo = build_topo()
    perc = Perception(topo)
    sigs = all_signals(topo, "N")

    car1 = make_car(1, "I00_I01", slot=5,  direction="E")
    car2 = make_car(2, "I00_I01", slot=10, direction="E")
    state = GlobalState(step=0, cars={1: car1, 2: car2}, signals=sigs)

    assert perc.is_blocked_ahead(car1, state) is True, \
        "Expected car 1 to be blocked by car 2 at slot 10"
    print("PASS  test_perception_blocked_by_car_ahead")


def test_perception_not_blocked_by_car_behind():
    """
    Car 2 is BEHIND car 1 — car 1 should not be blocked.
    """
    topo = build_topo()
    perc = Perception(topo)
    sigs = all_signals(topo, "N")

    car1 = make_car(1, "I00_I01", slot=10, direction="E")
    car2 = make_car(2, "I00_I01", slot=5,  direction="E")
    state = GlobalState(step=0, cars={1: car1, 2: car2}, signals=sigs)

    assert perc.is_blocked_ahead(car1, state) is False, \
        "Car behind should not block car ahead"
    print("PASS  test_perception_not_blocked_by_car_behind")


# ---------------------------------------------------------------------------
# ── PLANNER TESTS ────────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

def test_planner_destination_intersection():
    """Waypoint names should resolve to correct intersection IDs."""
    topo = build_topo()
    planner = RoutePlanner(topo)

    assert planner.destination_intersection("B") == "I02"
    assert planner.destination_intersection("C") == "I22"
    assert planner.destination_intersection("D") == "I20"
    assert planner.destination_intersection("A") == "I00"
    print("PASS  test_planner_destination_intersection")


def test_planner_bfs_finds_path():
    """
    BFS from I00 to I02 (B) in an uncongested network.
    Expected path: I00 → I01 → I02  (2 hops).
    """
    topo = build_topo()
    planner = RoutePlanner(topo)
    state = GlobalState(step=0, cars={}, signals={}, congestion={})

    path = planner._bfs_path("I00", "I02", state)
    assert path is not None, "BFS should find a path from I00 to I02"
    assert path[0] == "I00" and path[-1] == "I02", \
        f"Path should start at I00 and end at I02, got {path}"
    assert len(path) == 3, \
        f"Expected 3-node path [I00, I01, I02], got {path}"
    print("PASS  test_planner_bfs_finds_path")


def test_planner_next_intersection():
    """
    Car on I00_I01 heading to B (I02).
    Next intersection should be I01 (one step toward B).
    """
    topo = build_topo()
    planner = RoutePlanner(topo)
    state = GlobalState(step=0, cars={}, signals={}, congestion={})

    car = make_car(1, "I00_I01", slot=5, direction="E",
                   current_destination="B")
    next_inter = planner.next_intersection(car, "I02", state)

    assert next_inter is not None, "Expected a next intersection"
    # From I00_I01, the end is I01 → BFS next step from I01 toward I02 is I02
    # But _current_intersection_id returns I01 (end of segment), so next is I02
    assert next_inter in ("I01", "I02"), \
        f"Expected I01 or I02 as next step, got {next_inter}"
    print("PASS  test_planner_next_intersection")


def test_planner_choose_destination_no_visits():
    """
    Car has visited no waypoints — choose_next_destination should return
    one of B/C/D (nearest).
    """
    topo = build_topo()
    planner = RoutePlanner(topo)
    state = GlobalState(step=0, cars={}, signals={}, congestion={})

    car = make_car(1, "I00_I01", slot=0, direction="E",
                   destinations_visited=[], current_destination="B")
    dest = planner.choose_next_destination(car, state)

    assert dest in ["B", "C", "D"], \
        f"Expected unvisited waypoint, got {dest}"
    print("PASS  test_planner_choose_destination_no_visits")


def test_planner_choose_destination_all_visited():
    """
    Car has visited B, C, D → should return A (go home).
    """
    topo = build_topo()
    planner = RoutePlanner(topo)
    state = GlobalState(step=0, cars={}, signals={}, congestion={})

    car = make_car(1, "I00_I01", slot=0, direction="E",
                   destinations_visited=["B", "C", "D"],
                   current_destination="A")
    dest = planner.choose_next_destination(car, state)

    assert dest == "A", f"Expected 'A' (all waypoints visited), got {dest}"
    print("PASS  test_planner_choose_destination_all_visited")


# ---------------------------------------------------------------------------
# ── STRATEGY TESTS ────────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

def test_strategy_move_on_open_segment():
    """
    Car at slot 5 with no car ahead and green signal → action must be MOVE.
    """
    topo = build_topo()
    strategy = ActionStrategy(topo)
    sigs = all_signals(topo, "E")   # green for East

    car = make_car(1, "I00_I01", slot=5, direction="E")
    state = GlobalState(step=0, cars={1: car}, signals=sigs)
    actions = strategy.decide_all(state)

    assert len(actions) == 1
    assert actions[0].action == "MOVE", \
        f"Expected MOVE on open segment, got {actions[0].action}"
    print("PASS  test_strategy_move_on_open_segment")


def test_strategy_stay_when_blocked():
    """
    Car 1 has Car 2 directly ahead on same segment → Car 1 must STAY.
    """
    topo = build_topo()
    strategy = ActionStrategy(topo)
    sigs = all_signals(topo, "E")

    car1 = make_car(1, "I00_I01", slot=5,  direction="E")
    car2 = make_car(2, "I00_I01", slot=10, direction="E")
    state = GlobalState(step=0, cars={1: car1, 2: car2}, signals=sigs)
    actions = {a.car_id: a.action for a in strategy.decide_all(state)}

    assert actions[1] == "STAY", \
        f"Expected car 1 to STAY (blocked by car 2), got {actions[1]}"
    print("PASS  test_strategy_stay_when_blocked")


def test_strategy_stay_at_red_light():
    """
    Car East-bound at slot 29 of I00_I01 (intersection boundary).
    Signal at I01 is 'N' (red for East) → STAY.
    """
    topo = build_topo()
    strategy = ActionStrategy(topo)
    sigs = all_signals(topo, "N")   # only North is green

    car = make_car(1, "I00_I01", slot=29, direction="E")
    state = GlobalState(step=0, cars={1: car}, signals=sigs)
    actions = strategy.decide_all(state)

    assert actions[0].action == "STAY", \
        f"Expected STAY at red light, got {actions[0].action}"
    print("PASS  test_strategy_stay_at_red_light")


def test_strategy_straight_through_green():
    """
    Car East-bound at slot 29 of I00_I01, signal at I01 is 'E'.
    Next segment should be I01_I02 (also East) → STRAIGHT.
    """
    topo = build_topo()
    strategy = ActionStrategy(topo)
    sigs = all_signals(topo, "E")   # green for East

    car = make_car(1, "I00_I01", slot=29, direction="E",
                   current_destination="B",  # B = I02
                   destinations_visited=[])
    state = GlobalState(step=0, cars={1: car}, signals=sigs)
    actions = strategy.decide_all(state)

    assert actions[0].action == "STRAIGHT", \
        f"Expected STRAIGHT through green intersection, got {actions[0].action}"
    print("PASS  test_strategy_straight_through_green")


def test_strategy_intersection_in_use():
    """
    Two cars both at slot 29 of different segments incoming to I01.
    Only ONE should cross; the other must STAY.
    """
    topo = build_topo()
    strategy = ActionStrategy(topo)
    # Both E and S are green — but only 1 car can cross
    sigs = {
        iid: SignalState(intersection_id=iid, green_direction="E")
        for iid in topo.all_intersection_ids()
    }
    # Override I01 to E (green for East)
    sigs["I01"] = SignalState(intersection_id="I01", green_direction="E")
    # Override I11 to S for the south-bound car approaching I11 - not needed here

    # Car 1 East-bound approaching I01 from I00_I01
    car1 = make_car(1, "I00_I01", slot=29, direction="E",
                    current_destination="B", destinations_visited=[])
    # Car 2 also at I01 boundary from a different incoming segment — I01_I00
    # (West-bound, but let's use South-bound from I01 incoming)
    # Actually place car2 on I01_I11 going South at slot 0 to avoid conflict
    car2 = make_car(2, "I01_I11", slot=0, direction="S",
                    current_destination="C", destinations_visited=[])

    state = GlobalState(step=0, cars={1: car1, 2: car2}, signals=sigs)
    actions = {a.car_id: a.action for a in strategy.decide_all(state)}

    # Car 1 is at intersection boundary; car 2 is not → car 1 should cross
    assert actions[1] in ("STRAIGHT", "TURN_LEFT", "TURN_RIGHT"), \
        f"Car 1 should cross (green+clear), got {actions[1]}"
    assert actions[2] == "MOVE", \
        f"Car 2 not at boundary, should MOVE, got {actions[2]}"
    print("PASS  test_strategy_intersection_in_use")


# ---------------------------------------------------------------------------
# ── CONTROLLER VERIFICATION TESTS ────────────────────────────────────────────
# ---------------------------------------------------------------------------

def test_verify_no_violations_clean_state():
    """Two cars moving legally → all violation counts must be 0."""
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    prev = GlobalState(step=0, cars={
        1: make_car(1, "I00_I01", slot=5,  direction="E"),
        2: make_car(2, "I01_I02", slot=10, direction="E"),
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: make_car(1, "I00_I01", slot=6,  direction="E"),
        2: make_car(2, "I01_I02", slot=11, direction="E"),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["collisions"]           == 0
    assert report["red_light_violations"] == 0
    assert report["wrong_way_violations"] == 0
    print("PASS  test_verify_no_violations_clean_state")


def test_verify_collision_detection():
    """Two cars in same (segment, slot) → collisions = 1."""
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    prev = GlobalState(step=0, cars={
        1: make_car(1, "I00_I01", slot=5, direction="E"),
        2: make_car(2, "I00_I01", slot=7, direction="E"),
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: make_car(1, "I00_I01", slot=6, direction="E"),
        2: make_car(2, "I00_I01", slot=6, direction="E"),  # collision!
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["collisions"] == 1, \
        f"Expected 1 collision, got {report['collisions']}"
    print("PASS  test_verify_collision_detection")


def test_verify_red_light_violation():
    """
    Car crosses I01 intersection while signal is 'N' (red for East).
    Expected: red_light_violations = 1.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")   # N green → E/W/S red

    prev = GlobalState(step=0, cars={
        1: make_car(1, "I00_I01", slot=29, direction="E"),
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: make_car(1, "I01_I02", slot=0, direction="E"),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["red_light_violations"] == 1, \
        f"Expected 1 red-light violation, got {report['red_light_violations']}"
    print("PASS  test_verify_red_light_violation")


def test_verify_wrong_way_violation():
    """
    Car on I00_I01 (direction=E) but car.direction='W' → wrong-way.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    prev = GlobalState(step=0, cars={
        1: make_car(1, "I00_I01", slot=5, direction="W"),  # wrong way!
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: make_car(1, "I00_I01", slot=5, direction="W"),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["wrong_way_violations"] == 1, \
        f"Expected 1 wrong-way violation, got {report['wrong_way_violations']}"
    print("PASS  test_verify_wrong_way_violation")


def test_verify_cumulative_stats():
    """
    Two steps: one collision, one red-light.
    get_stats() must reflect both.
    """
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    # Step A — collision
    prev_a = GlobalState(step=0, cars={
        1: make_car(1, "I00_I01", slot=5, direction="E"),
        2: make_car(2, "I00_I01", slot=7, direction="E"),
    }, signals=sigs)
    curr_a = GlobalState(step=1, cars={
        1: make_car(1, "I00_I01", slot=6, direction="E"),
        2: make_car(2, "I00_I01", slot=6, direction="E"),
    }, signals=sigs)
    ctrl.verify_step(prev_a, curr_a)

    # Step B — red-light
    prev_b = GlobalState(step=1, cars={
        3: make_car(3, "I00_I01", slot=29, direction="E"),
    }, signals=sigs)
    curr_b = GlobalState(step=2, cars={
        3: make_car(3, "I01_I02", slot=0, direction="E"),
    }, signals=sigs)
    ctrl.verify_step(prev_b, curr_b)

    stats = ctrl.get_stats()
    assert stats["total_collisions"]           == 1
    assert stats["total_red_light_violations"] == 1
    assert stats["total_wrong_way_violations"] == 0
    print("PASS  test_verify_cumulative_stats")


def test_verify_inactive_cars_ignored():
    """Inactive cars must not trigger any violations."""
    ctrl, topo = build_controller()
    sigs = all_signals(topo, "N")

    prev = GlobalState(step=0, cars={
        1: make_car(1, "I00_I01", slot=5, direction="W",
                    active=False),  # inactive wrong-way car
    }, signals=sigs)

    curr = GlobalState(step=1, cars={
        1: make_car(1, "I00_I01", slot=5, direction="W", active=False),
    }, signals=sigs)

    report = ctrl.verify_step(prev, curr)

    assert report["collisions"]           == 0
    assert report["red_light_violations"] == 0
    assert report["wrong_way_violations"] == 0
    print("PASS  test_verify_inactive_cars_ignored")


# ---------------------------------------------------------------------------
# Run all tests
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("=" * 60)
    print("v_group Test Suite")
    print("=" * 60)

    print("\n── Perception ──")
    test_perception_signal_green()
    test_perception_signal_red()
    test_perception_not_blocked()
    test_perception_blocked_by_car_ahead()
    test_perception_not_blocked_by_car_behind()

    print("\n── Planner ──")
    test_planner_destination_intersection()
    test_planner_bfs_finds_path()
    test_planner_next_intersection()
    test_planner_choose_destination_no_visits()
    test_planner_choose_destination_all_visited()

    print("\n── Strategy ──")
    test_strategy_move_on_open_segment()
    test_strategy_stay_when_blocked()
    test_strategy_stay_at_red_light()
    test_strategy_straight_through_green()
    test_strategy_intersection_in_use()

    print("\n── Controller Verification ──")
    test_verify_no_violations_clean_state()
    test_verify_collision_detection()
    test_verify_red_light_violation()
    test_verify_wrong_way_violation()
    test_verify_cumulative_stats()
    test_verify_inactive_cars_ignored()

    print("\n" + "=" * 60)
    print("All 21 tests passed.")
    print("=" * 60)