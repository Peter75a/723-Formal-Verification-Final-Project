# main.py
"""
Quick demo: runs a mini simulation loop showing v_group and i_group
working together for a few steps. No full simulator needed.

Run from the project root:
    python main.py
"""

from shared.topology import Topology, WAYPOINT_INTERSECTIONS
from shared.state import GlobalState, CarState, SignalState
from shared.enums import SPEED_MOVE


# (incoming_direction, turn_action) -> outgoing_direction
_OUTGOING_DIRECTION = {
    ("E", "STRAIGHT"):   "E", ("E", "TURN_LEFT"):  "N", ("E", "TURN_RIGHT"): "S",
    ("W", "STRAIGHT"):   "W", ("W", "TURN_LEFT"):  "S", ("W", "TURN_RIGHT"): "N",
    ("N", "STRAIGHT"):   "N", ("N", "TURN_LEFT"):  "W", ("N", "TURN_RIGHT"): "E",
    ("S", "STRAIGHT"):   "S", ("S", "TURN_LEFT"):  "E", ("S", "TURN_RIGHT"): "W",
}

# intersection_id -> waypoint name, for B/C/D waypoints only
_WAYPOINT_AT = {v: k for k, v in WAYPOINT_INTERSECTIONS.items()}

from v_group.controller import VehicleController
from i_group.controller import InfrastructureController


def make_initial_state(topo: Topology) -> GlobalState:
    """Create a simple starting state with 2 cars."""
    signals = {
        iid: SignalState(intersection_id=iid, green_direction="E")
        for iid in topo.all_intersection_ids()
    }
    cars = {
        1: CarState(car_id=1, segment_id="A_I00", slot=0,
                    direction="E", speed=SPEED_MOVE,
                    destinations_visited=[], current_destination="B"),
        2: CarState(car_id=2, segment_id="A_I00", slot=1,
                    direction="E", speed=SPEED_MOVE,
                    destinations_visited=[], current_destination="D"),
    }
    return GlobalState(step=0, cars=cars, signals=signals, congestion={})


def apply_actions_naive(state: GlobalState, car_actions, signal_actions,
                        topo: Topology) -> GlobalState:
    """
    Apply one simulation step:
      - STAY          : no change
      - MOVE          : advance one slot; deactivate if segment ends at A
      - STRAIGHT /
        TURN_LEFT /
        TURN_RIGHT    : if not at last slot, advance one slot;
                        if at last slot, cross the intersection and enter
                        slot 0 of the outgoing segment; mark waypoint if needed
    """
    import copy
    new_cars = {}
    for car_id, car in state.cars.items():
        new_car = copy.copy(car)
        new_car.destinations_visited = list(car.destinations_visited)

        action_obj = next((a for a in car_actions if a.car_id == car_id), None)
        if action_obj is None or action_obj.action == "STAY":
            new_cars[car_id] = new_car
            continue

        seg = topo.get_segment_by_id(car.segment_id)
        if seg is None:
            new_cars[car_id] = new_car
            continue

        action = action_obj.action
        at_last = (car.slot == seg.length - 1)

        if not at_last:
            # Simple advance within the same segment
            new_car.slot = car.slot + 1
        else:
            # At the last slot — handle based on what the segment ends at
            end_node = seg.end

            if action == "MOVE":
                # Segment ends at the A endpoint → car completed the tour
                if not topo.is_intersection(end_node):
                    new_car.active = False

            elif action in ("STRAIGHT", "TURN_LEFT", "TURN_RIGHT"):
                # Crossing a circle intersection
                if topo.is_intersection(end_node):
                    out_dir = _OUTGOING_DIRECTION.get((car.direction, action))
                    if out_dir:
                        # Find the outgoing segment with the required direction
                        inter = topo.get_intersection(end_node)
                        next_seg = next(
                            (topo.get_segment_by_id(s)
                             for s in inter.outgoing
                             if topo.get_segment_by_id(s) is not None
                             and topo.get_segment_by_id(s).direction == out_dir),
                            None
                        )
                        if next_seg:
                            new_car.segment_id = next_seg.segment_id
                            new_car.slot = 0
                            new_car.direction = out_dir

                            # Mark waypoint if this intersection is B, C, or D
                            waypoint = _WAYPOINT_AT.get(end_node)
                            if waypoint and waypoint not in new_car.destinations_visited:
                                new_car.destinations_visited.append(waypoint)
                                remaining = [w for w in ["B", "C", "D"]
                                             if w not in new_car.destinations_visited]
                                new_car.current_destination = (
                                    remaining[0] if remaining else "A"
                                )

        new_cars[car_id] = new_car

    new_signals = {
        a.intersection_id: SignalState(
            intersection_id=a.intersection_id,
            green_direction=a.green_direction)
        for a in signal_actions
    }

    return GlobalState(
        step=state.step + 1,
        cars=new_cars,
        signals=new_signals,
        congestion=state.congestion,
    )


def main():
    print("=" * 55)
    print("  ECEN 723 Traffic Simulation — Phase A Demo")
    print("=" * 55)

    # ── Setup ──────────────────────────────────────────────
    topo = Topology()
    topo.build()
    print(f"\nTopology built: {len(topo.all_intersection_ids())} intersections, "
          f"{len(topo.all_segment_ids())} segments\n")

    v_ctrl = VehicleController(topo)
    i_ctrl = InfrastructureController(topo)

    state = make_initial_state(topo)
    STEPS = 60

    # ── Simulation loop ────────────────────────────────────
    for step in range(STEPS):
        print(f"── Step {step} ──────────────────────────────────")

        # i-group decides signals
        signal_actions = i_ctrl.compute_signals(state)
        sig_summary = {a.intersection_id: a.green_direction
                       for a in signal_actions}
        # Print a few key signals
        for iid in ["I00", "I01", "I11"]:
            print(f"   Signal {iid}: green={sig_summary.get(iid)}")

        # v-group decides car actions
        car_actions = v_ctrl.decide_actions(state)
        for a in car_actions:
            car = state.cars[a.car_id]
            print(f"   Car {a.car_id}: {a.action:12s}  "
                  f"(seg={car.segment_id}, slot={car.slot})")

        # Advance state
        prev_state = state
        state = apply_actions_naive(state, car_actions, signal_actions, topo)

        # Both groups verify
        v_report = v_ctrl.verify_step(prev_state, state)
        i_report = i_ctrl.verify_step(prev_state, state)

        common_keys = ("collisions", "red_light_violations", "wrong_way_violations")
        v_ok = all(v_report[k] == 0 for k in common_keys)
        i_ok = all(i_report[k] == 0 for k in common_keys)

        status = "✓ CLEAN" if (v_ok and i_ok) else "✗ VIOLATIONS"
        agree  = "✓ AGREE" if all(v_report[k] == i_report[k] for k in common_keys) \
                           else "✗ MISMATCH"
        extra = ""
        if v_report.get("illegal_turn_violations", 0):
            extra += f" illegal_turns={v_report['illegal_turn_violations']}"
        if v_report.get("multi_crossing_violations", 0):
            extra += f" multi_cross={v_report['multi_crossing_violations']}"
        print(f"   Verify: {status}   v/i-group: {agree}{extra}")

    # ── Final stats ────────────────────────────────────────
    print("\n" + "=" * 55)
    print("Final cumulative stats:")
    print(f"  v_group: {v_ctrl.get_stats()}")
    print(f"  i_group: {i_ctrl.get_stats()}")
    print("=" * 55)


if __name__ == "__main__":
    main()