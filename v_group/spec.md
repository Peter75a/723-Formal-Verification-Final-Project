# v-group Specification

# Vehicle Control & Routing System

> **Purpose:** This document is the authoritative spec for the v-group module.
> It describes the architecture, algorithms, and interface contracts for vehicle
> simulation in the 3×3 grid traffic network.

---

## 1. Project Overview

Each vehicle departs from **A**, visits waypoints **B**, **C**, **D** (in any order),
and returns to **A**. The v-group goal is to maximize throughput (completed tours
per hour) with zero collisions, zero red-light violations, and zero wrong-way events.

The **v-group** is responsible for:

1. Perceiving the environment — signals, nearby cars, congestion (`perception.py`)
2. Planning routes through the grid — BFS-based shortest path (`planner.py`)
3. Deciding each car's action every step — safety-first decision policy (`strategy.py`)
4. Verifying safety constraints independently (`controller.py`)

---

## 2. Road Network (from shared/topology.py)

```
A(□) ── I00 ── I01 ── I02 (= B)
          |      |      |
         I10 ── I11 ── I12
          |      |      |
        I20 ── I21 ── I22
       (= D)          (= C)
```

- **Slots per segment:** 30 (inter-intersection), 2 (A ↔ I00)
- **Step duration:** 2 seconds
- **Speed:** 30 mph = 1 slot/step, or 0 (stopped)
- **Waypoints:** B = I02, C = I22, D = I20

---

## 3. Module Structure

```
v_group/
├── controller.py   # VehicleController — top-level interface, verify_step
├── perception.py   # Perception — sense signals, cars ahead, intersection state
├── planner.py      # RoutePlanner — BFS route planning with congestion weighting
└── strategy.py     # ActionStrategy — safety-first action decision per car
```

---

## 4. Algorithm: BFS Route Planning

**Planner** builds a graph of intersections from `shared/topology.py` and runs
Breadth-First Search to find the shortest (fewest steps) route from the car's
current intersection to the next waypoint.

**Congestion weighting:** When congestion count at a neighbor is ≥ 3 stopped cars,
that neighbor's edge cost is increased, causing BFS to prefer less-congested paths.

**Waypoint order:** Determined greedily — the nearest unvisited waypoint (fewest hops)
is chosen as the next destination. This avoids long detours.

---

## 5. Algorithm: Safety-First Action Decision

Priority order in `strategy.py`:

1. **Inactive car** → skip (no action)
2. **Car not at last slot** → check visibility ahead; STAY if blocked, else MOVE
3. **Car at last slot (intersection boundary):**
   a. Check signal — if red, STAY
   b. Determine next segment from route plan
   c. Check right-turn-at-red constraint
   d. Check at-most-1-car-per-intersection constraint
   e. Determine turn type (STRAIGHT / TURN_LEFT / TURN_RIGHT)
   f. Issue crossing action

---

## 6. Perception Rules

- **Forward visibility:** A car sees up to 30 slots ahead on the same segment.
  If another car is directly ahead (no third car in between), the car MUST stop.
- **Intersection visibility:** At slot 29, a car can see other cars at the same
  intersection node (i.e., also at slot 29 of any incoming segment).
- **Signal reading:** Read `state.signals[intersection_id].green_direction`.
  Green only if `car.direction == green_direction`.

---

## 7. Verification Rules (mirrors i-group)

`verify_step(prev_state, curr_state)` checks:

| Violation | Condition                                                   |
| --------- | ----------------------------------------------------------- |
| Collision | >1 active car at same (segment_id, slot) in curr_state      |
| Red-light | Car crossed intersection while signal red for its direction |
| Wrong-way | Car direction ≠ segment direction                           |

Results must match i-group's `verify_step` output.

---

## 8. Interface Contract

- **Input:** `GlobalState` from simulator each step
- **Output:** `List[CarAction]` — one per active car, action in `ACTIONS`
- **Never** modify `GlobalState` directly
- **Respect** `shared/` contracts — do not change shared files
