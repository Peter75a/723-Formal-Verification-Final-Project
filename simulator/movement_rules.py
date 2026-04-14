# simulator/movement_rules.py

"""
Movement logic for the simulator.

apply_car_actions() advances all cars by one step based on their chosen
actions, then returns a new GlobalState.

compute_congestion() counts stopped cars near each intersection and is used
to populate GlobalState.congestion before v-group reads it.

These functions are pure (no side effects) — they consume a state and return
a new one, leaving the original untouched.
"""

import copy
from typing import List

from shared.topology import Topology, WAYPOINT_INTERSECTIONS
from shared.state import GlobalState, CarState, SignalState
from shared.actions import CarAction, SignalAction


# (incoming_direction, turn_action) -> outgoing_direction
_OUTGOING_DIRECTION = {
    ("E", "STRAIGHT"):   "E", ("E", "TURN_LEFT"):  "N", ("E", "TURN_RIGHT"): "S",
    ("W", "STRAIGHT"):   "W", ("W", "TURN_LEFT"):  "S", ("W", "TURN_RIGHT"): "N",
    ("N", "STRAIGHT"):   "N", ("N", "TURN_LEFT"):  "W", ("N", "TURN_RIGHT"): "E",
    ("S", "STRAIGHT"):   "S", ("S", "TURN_LEFT"):  "E", ("S", "TURN_RIGHT"): "W",
}

# intersection_id -> waypoint name  (B/C/D only)
_WAYPOINT_AT = {v: k for k, v in WAYPOINT_INTERSECTIONS.items()}


def apply_car_actions(
    state: GlobalState,
    car_actions: List[CarAction],
    signal_actions: List[SignalAction],
    topo: Topology,
) -> GlobalState:
    """
    Apply one simulation step: advance cars and update signals.

    Step order for each car:
      STAY           -> no change
      MOVE           -> advance one slot; if at end of A-segment, deactivate car
      STRAIGHT /
      TURN_LEFT /
      TURN_RIGHT     -> if not at last slot, advance one slot within same segment;
                       if at last slot, cross the intersection into slot 0 of the
                       outgoing segment; mark waypoint if B/C/D is reached

    Args:
        state:          Current GlobalState (step t).
        car_actions:    List[CarAction] from v-group.
        signal_actions: List[SignalAction] from i-group.
        topo:           Topology instance.

    Returns:
        New GlobalState at step t+1.
    """
    action_map = {a.car_id: a.action for a in car_actions}

    new_cars = {}
    for car_id, car in state.cars.items():
        new_car = copy.copy(car)
        new_car.destinations_visited = list(car.destinations_visited)

        action = action_map.get(car_id, "STAY")

        if action == "STAY":
            new_car.speed = 0
            new_cars[car_id] = new_car
            continue

        seg = topo.get_segment_by_id(car.segment_id)
        if seg is None:
            new_cars[car_id] = new_car
            continue

        at_last = (car.slot == seg.length - 1)

        if not at_last:
            # Advance within the same segment
            new_car.slot = car.slot + 1
            new_car.speed = 30
        else:
            end_node = seg.end

            if action == "MOVE":
                # Segment ends at a non-intersection (endpoint A) -> tour complete
                if not topo.is_intersection(end_node):
                    new_car.active = False
                    new_car.speed = 0

            elif action in ("STRAIGHT", "TURN_LEFT", "TURN_RIGHT"):
                if topo.is_intersection(end_node):
                    out_dir = _OUTGOING_DIRECTION.get((car.direction, action))
                    if out_dir:
                        inter = topo.get_intersection(end_node)
                        next_seg = next(
                            (topo.get_segment_by_id(s)
                             for s in inter.outgoing
                             if topo.get_segment_by_id(s) is not None
                             and topo.get_segment_by_id(s).direction == out_dir),
                            None,
                        )
                        if next_seg:
                            new_car.segment_id = next_seg.segment_id
                            new_car.slot = 0
                            new_car.direction = out_dir
                            new_car.speed = 30

                            # Mark waypoint B/C/D if reached
                            waypoint = _WAYPOINT_AT.get(end_node)
                            if waypoint and waypoint not in new_car.destinations_visited:
                                new_car.destinations_visited.append(waypoint)
                                remaining = [
                                    w for w in ["B", "C", "D"]
                                    if w not in new_car.destinations_visited
                                ]
                                new_car.current_destination = (
                                    remaining[0] if remaining else "A"
                                )

        new_cars[car_id] = new_car

    new_signals = {
        a.intersection_id: SignalState(
            intersection_id=a.intersection_id,
            green_direction=a.green_direction,
        )
        for a in signal_actions
    }

    return GlobalState(
        step=state.step + 1,
        cars=new_cars,
        signals=new_signals,
        congestion=state.congestion,
    )


def compute_congestion(state: GlobalState, topo: Topology) -> dict:
    """
    Count stopped cars at or near each intersection.

    A car is "stopping" (per spec) if speed == 0, which the simulator sets
    when action == STAY. We attribute each stopped car to the intersection
    that is the end-node of the car's current segment.

    Args:
        state: Current GlobalState (after movement applied).
        topo:  Topology instance.

    Returns:
        Dict[intersection_id, int] -- number of stopped cars per intersection.
    """
    congestion: dict = {iid: 0 for iid in topo.all_intersection_ids()}

    for car in state.cars.values():
        if not car.active or car.speed != 0:
            continue
        seg = topo.get_segment_by_id(car.segment_id)
        if seg is None:
            continue
        end_node = seg.end
        if end_node in congestion:
            congestion[end_node] += 1

    return congestion
