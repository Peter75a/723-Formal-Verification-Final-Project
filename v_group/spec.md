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
Dijkstra (via min-heap) to find the shortest (fewest hops) route from the car's
current intersection to the next waypoint.

**Congestion weighting:** When `GlobalState.congestion[neighbor] >= 3` stopped cars,
that neighbor's edge cost is increased by 2, causing the planner to prefer less-congested paths.

**Waypoint order:** Determined greedily — the nearest unvisited waypoint (fewest hops
via unweighted BFS distances) is chosen as the next destination. This avoids long detours.

---

## 5. Algorithm: Safety-First Action Decision

Priority order in `strategy.py._decide_one()`:

1. **Inactive car** → skip (no action emitted)
2. **Car not at last slot** (`slot < seg.length - 1`) → check visibility:
   - Blocked by car ahead on same segment → **STAY**
   - Otherwise → **MOVE**
3. **Car at last slot** (`slot == seg.length - 1`), approaching segment end:
   - **Endpoint A** (segment end is not an intersection) → **MOVE**
     (simulator deactivates the car when it reaches A)
   - **Real intersection ahead:**
     - a. Signal at intersection is red for car's direction → **STAY**
     - b. Intersection already committed to another car this step → **STAY**
     - c. Another car at intersection boundary already has green signal → **STAY**
     - d. No valid outgoing segment found (no legal turn toward destination) → **STAY**
     - e. Turn pair `(incoming_dir, outgoing_dir)` maps to no action (U-turn) → **STAY**
     - f. All clear → commit intersection as in-use; emit **STRAIGHT / TURN_LEFT / TURN_RIGHT**

> **Right-turn-at-red:** There is no separate check for this. Step (a) already
> requires the signal to be green for the car's incoming direction before any
> crossing action (including right turns) is allowed. A right turn at red is
> therefore blocked automatically by step (a).

---

## 6. Perception Rules

- **Forward visibility:** A car sees other active cars up to `VISIBILITY_RANGE = 30`
  slots ahead on the **same segment**. The nearest car ahead (if any) blocks movement.
  "Directly ahead" = smallest slot index greater than the observing car's slot within range.
- **Intersection visibility:** At `slot == seg.length - 1` (the last slot, adjacent to
  the intersection), a car can see other cars waiting at the same intersection node
  (i.e., also at `slot == seg.length - 1` of any incoming segment of that intersection).
- **Signal reading:** Read `state.signals[intersection_id].green_direction`.
  Green only if `car.direction == green_direction` (single-direction phase model).

---

## 7. Verification Rules

`verify_step(prev_state, curr_state)` checks **five** violation types:

| Violation | Condition | Mirrors i-group? |
| --------- | ----------------------------------------------------------- | --- |
| Collision | >1 active car at same `(segment_id, slot)` in `curr_state` | Yes |
| Red-light | Car crossed intersection while signal was red for its direction | Yes |
| Wrong-way | Active car's `direction` ≠ its current segment's `direction` | Yes |
| Multi-crossing | >1 car crossed the same intersection in one step | Yes |
| Illegal turn | Car changed segments and new direction is direct opposite of old (U-turn: E↔W, N↔S) | v-group only |

The first four categories are implemented identically in both controllers and must
produce the same counts for any given state transition. The fifth (`illegal_turn_violations`)
is an extra v-group safety check not reported by i-group.

Cumulative stats are tracked in `VehicleController` and returned by `get_stats()`:

- `total_collisions`
- `total_red_light_violations`
- `total_wrong_way_violations`
- `total_multi_crossing_violations`
- `total_illegal_turn_violations`
- `total_completed_tours`

---

## 8. Interface Contract

- **Input:** `GlobalState` from simulator each step
- **Output:** `List[CarAction]` — one per active car, action in `ACTIONS`
- **Never** modify `GlobalState` directly
- **Respect** `shared/` contracts — do not change shared files
