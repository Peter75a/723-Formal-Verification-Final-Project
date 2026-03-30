# i-group Specification
# Traffic Signal Control & Verification System

> **Purpose:** This document is the authoritative spec for the i-group module.
> It is written to be directly consumed by Claude (or any LLM) to generate,
> review, or extend code in `i_group/`. All class names, field names, and
> constants reference the actual codebase exactly as implemented.

---

## 1. Project Overview

A 3×3 grid road network simulation. Each vehicle departs from **A**, visits
waypoints **B**, **C**, **D** (in any order), and returns to **A**.

The **i-group** is responsible for:
1. Deciding traffic signal phases at every intersection each step (`compute_signals`)
2. Verifying safety violations after each step (`verify_step`)
3. Reporting cumulative violation counts (`get_stats`)

The **v-group** controls vehicle movement. The **simulator** owns the ground truth
`GlobalState` and calls both groups each step.

---

## 2. Road Network Topology

> Source of truth: `shared/topology.py`

### 2.1 Grid Layout

```
A(□) ── I00 ── I01 ── I02 (= B)
          |      |      |
         I10 ── I11 ── I12
          |      |      |
        I20 ── I21 ── I22
       (= D)          (= C)
```

- **9 intersections**: `I{row}{col}`, row 0 = top, col 0 = left
- **A**: square endpoint, located WEST of `I00`; the only non-intersection node
- **B** = `I02` (top-right), **C** = `I22` (bottom-right), **D** = `I20` (bottom-left)
  — these are regular intersections, not separate nodes

### 2.2 Segments

| Type | Slots | Distance | Travel time |
|---|---|---|---|
| Normal (inter-intersection) | 30 | 0.5 mile | 60 s = 30 steps |
| Endpoint (A ↔ I00) | 2 | 1/30 mile | 4 s = 2 steps |

- Segment ID format: `"{from_node}_{to_node}"` e.g. `"I00_I01"`, `"A_I00"`
- Every inter-intersection link has **two directed segments** (one per direction)
- Constants: `SLOTS_PER_SEGMENT = 30`, `ENDPOINT_SLOTS = 2`

### 2.3 Slot Indexing

- Slots are numbered `0` (entry end) → `29` (exit end, adjacent to intersection)
- A car at slot `29` is at the intersection boundary
- Crossing an intersection = moving from slot `29` of an incoming segment to slot `0` of an outgoing segment
- Constants: `MIN_SLOT = 0`, `MAX_SLOT = 29`

### 2.4 Turn Rules

U-turns are **always forbidden**. Right turns are allowed except at red light.

| Incoming direction | Forbidden outgoing |
|---|---|
| E | W (U-turn) |
| W | E (U-turn) |
| N | S (U-turn) |
| S | N (U-turn) |

See `VALID_TURNS` dict in `topology.py` for the full matrix.

---

## 3. Time & Speed Model

- **Step duration**: 2 seconds per control step
- **Speed options**: `SPEED_MOVE = 30` mph (advance 1 slot/step) or `SPEED_STOP = 0` (stay)
- A car moves **exactly 1 slot per step** when moving, or stays in place
- **At most 1 car** may cross any single intersection between two consecutive steps

---

## 4. Shared Data Structures

> Do NOT modify files in `shared/`. These are the team contract.

### 4.1 `CarState` (`shared/state.py`)

```python
@dataclass
class CarState:
    car_id: int                        # unique vehicle ID
    segment_id: str                    # e.g. "I00_I01"
    slot: int                          # 0–29
    direction: str                     # "N" | "S" | "E" | "W"
    speed: int                         # SPEED_MOVE (30) or SPEED_STOP (0)
    active: bool                       # False when trip is complete
    destinations_visited: List[str]    # subset of ["B", "C", "D"]
    current_destination: str           # next waypoint to visit
```

### 4.2 `SignalState` (`shared/state.py`)

```python
@dataclass
class SignalState:
    intersection_id: str     # e.g. "I11"
    green_direction: str     # "NS" or "EW"
```

### 4.3 `GlobalState` (`shared/state.py`)

```python
@dataclass
class GlobalState:
    step: int
    cars: Dict[int, CarState]
    signals: Dict[str, SignalState]
    congestion: Dict[str, int]    # stopped cars near each intersection (read-only for i-group)
```

### 4.4 `SignalAction` (`shared/actions.py`)

```python
@dataclass
class SignalAction:
    intersection_id: str     # target intersection
    green_direction: str     # "NS" or "EW"
```

This is the **only output** i-group produces. The simulator applies it to update `GlobalState.signals`.

---

## 5. Signal Phase Rules

- Each intersection has **at most 4 signal lights**, one per direction: N, S, E, W
- At any time, **at most 1 light can be green**; all others are red
- A car at **red light must stop** (speed = 0, action = `"STAY"`)
- A car cannot make a **right turn at red light** (even though right turns are otherwise allowed)


## 6. i-group Module Structure

```
i_group/
├── controller.py   # InfrastructureController — main entry point
├── policy.py       # SignalPolicy — adaptive green phase decision logic
├── scheduler.py    # SignalScheduler — per-intersection timer management
└── spec.md         # this file
```



## 7. Safety Verification Rules

`verify_step(prev_state, curr_state)` checks three violation types after each step.

### 7.1 Collision (`_check_collisions`)

**Definition:** More than 1 active car occupies the same `(segment_id, slot)` in `curr_state`.

**Detection:**
```
for each active car in curr_state:
    key = (car.segment_id, car.slot)
    if occupancy[key] > 1 → collision
```

Each over-occupied slot counts as **1 collision event**.

### 7.2 Red-Light Violation (`_check_red_light_violations`)

**Definition:** A car crossed an intersection while the signal was red for its direction.

**Detection conditions (all must be true):**
1. Car was active in both `prev_state` and `curr_state`
2. In `prev_state`, car was at slot `segment.length - 1` (last slot, intersection boundary)
3. In `curr_state`, car is on a **different segment** (it crossed)
4. The segment end node is a real circle intersection (not endpoint A)
5. The signal at that intersection in `prev_state` did **not** cover the car's direction

**Key:** Uses `prev_state.signals` (the signals in effect when the car moved).

### 7.3 Wrong-Way Violation (`_check_wrong_way`)

**Definition:** An active car's `direction` does not match its current segment's defined direction.

**Detection:**
```
for each active car in curr_state:
    if car.direction != topology.get_segment_direction(car.segment_id) → violation
```

---

## 8. Safety Constraints Summary

| Constraint | Checked by | Source |
|---|---|---|
| No two cars in the same slot | `_check_collisions` | `curr_state` |
| No crossing intersection at red light | `_check_red_light_violations` | `prev_state` signals |
| No driving against segment direction | `_check_wrong_way` | `curr_state` |
| No right turn at red light | Covered by red-light check | `prev_state` signals |
| No U-turn | `topology.is_valid_turn()` | turn validation |
| At most 1 car crosses intersection per step | Covered by collision check | `curr_state` |

All three violation counts must be **0** for a valid simulation run.

---

## 9. Visibility Rules (for reference — enforced by v-group)

- A car can see another car ahead on the same segment within **0.5 mile = 30 slots**
- If car P sees car Q directly ahead (no third car between them), P **must not move**
- At an intersection, a car can see all other cars at the same intersection
- i-group uses `congestion` in `GlobalState` (pre-computed by simulator) to observe stopped cars

---

## 10. Coding Conventions

- i-group must **never modify** `GlobalState` directly; only return `List[SignalAction]`
- All intersection IDs come from `topology.all_intersection_ids()` → sorted list e.g. `["I00", "I01", ..., "I22"]`
- Signal phase strings are always `"NS"` or `"EW"` (from `SIGNAL_PHASES` in `enums.py`)
- Direction strings are always single uppercase chars: `"N"`, `"S"`, `"E"`, `"W"`
- `active=False` cars must be skipped in all checks
- `prev_state.signals` is a `Dict[str, SignalState]` keyed by `intersection_id`
