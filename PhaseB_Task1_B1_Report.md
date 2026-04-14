# ECEN 723 Spring 2026 — Project Phase B Report

**Team ID:** 10  
**Group:** v-group  
**Task:** B.1 — Software Integration  
**Members:** Brian Hsieh, Dhruv Nandwani  
**Date:** April 14, 2026

---

## 1. Task Overview

Task B.1 requires integrating the v-group and i-group codebases into a single unified simulation loop such that:

1. **Vehicles (v-group) know the traffic light state at every intersection at every moment.**
2. **The infrastructure controller (i-group) knows all vehicle locations at every moment.**

Both groups must also independently verify the same safety properties — no collisions, no red-light violations, no wrong-way driving, no multiple crossings at one intersection per step — and their results must agree.

---

## 2. Integration Architecture

### 2.1 File Structure

The integration lives entirely in `simulator/`, which neither group previously owned:

```
simulator/
  simulator.py        ← Simulator class: unified step loop
  movement_rules.py   ← apply_car_actions(), compute_congestion()
  test_integration.py ← three test scenarios with log file output
  logs/               ← auto-generated output files (evidence)
    test1_safety_2cars_100steps.txt
    test2_extended_4cars_400steps.txt
    test3_throughput_6cars_1hour.txt
```

`shared/` (GlobalState, CarState, SignalState, Topology, actions) was not modified. `i_group/` and `v_group/` controller interfaces were not changed.

### 2.2 The Integration Contract: `GlobalState`

The shared `GlobalState` object is the integration point. It holds:
- `state.signals` — signal decisions written by i-group, read by v-group
- `state.cars`    — vehicle positions read by i-group for verification and signal scheduling
- `state.congestion` — stopped-car counts per intersection, computed by simulator, read by v-group for route planning

Neither group modifies `GlobalState` directly. Only the simulator writes to it between group calls.

### 2.3 Step-by-Step Execution Order

Each simulation tick follows this exact order:

```
Step N:
  1.  i_ctrl.compute_signals(state)         → signal_actions
  2.  state.signals ← signal_actions         [i-group output published]
  3.  v_ctrl.decide_actions(state)           → car_actions
         (v-group reads state.signals here)  [v-group sees live signals]
  4.  prev_state ← deep copy of state
  5.  new_state  ← apply_car_actions(state, car_actions, signal_actions)
  6.  new_state.congestion ← compute_congestion(new_state)
  7.  v_report   ← v_ctrl.verify_step(prev_state, new_state)
  8.  i_report   ← i_ctrl.verify_step(prev_state, new_state)
         (i-group reads new_state.cars here) [i-group sees all positions]
  9.  Assert v_report and i_report agree on all shared metrics
  10. Respawn: inject new car at A_I00 slot 0 if active count < target
```

**Step 2 is the key integration moment for the v-group:** signals are written into `state` *before* `decide_actions` is called, so every vehicle decision already reflects the current green/red phase at every intersection.

**Steps 7–8 are the key integration moments for the i-group:** both `verify_step` calls receive `new_state`, which contains all vehicle positions after movement.

### 2.4 How v-group Accesses Traffic Lights

In `v_group/perception.py`, `is_signal_green()` reads directly from `state.signals`:

```python
def is_signal_green(self, car: CarState, state: GlobalState) -> bool:
    seg      = self.topology.get_segment_by_id(car.segment_id)
    end_node = seg.end
    signal   = state.signals.get(end_node)   # ← reads i-group's decision
    return signal.green_direction == car.direction
```

Because the simulator writes `signal_actions` into `state.signals` (Step 2) before `decide_actions` is called (Step 3), every car decision is based on the live signal state for the current step.

### 2.5 How i-group Accesses All Vehicle Locations

`i_group/controller.py` uses `curr_state.cars` directly in all three checkers:

```python
def _check_collisions(self, state: GlobalState) -> int:
    occupancy = {}
    for car in state.cars.values():    # ← all vehicles visible
        if not car.active:
            continue
        key = (car.segment_id, car.slot)
        occupancy[key] = occupancy.get(key, 0) + 1
    return sum(1 for count in occupancy.values() if count > 1)
```

The signal scheduling policy (`i_group/policy.py`) also uses `state.cars` to count approaching vehicles when deciding which phase to keep green:

```python
for car in state.cars.values():
    if car.segment_id in relevant and car.slot >= seg.length - APPROACH_THRESHOLD:
        count += 1
```

### 2.6 Bug Fixes Applied During Integration

Two pre-existing v-group bugs were discovered during integration testing with multiple cars and corrected:

**Fix 1 — Intersection slot-0 visibility (`v_group/perception.py`):**  
A car crossing an intersection could enter slot 0 of the outgoing segment even when another car was already sitting there. Per spec, "at an intersection, a car can see other cars at the same intersection." A car at slot 0 of an outgoing segment is within the intersection zone. A new method `is_destination_slot_blocked()` was added and called in `strategy.py` before committing to a crossing action.

**Fix 2 — Return-to-A routing (`v_group/strategy.py`):**  
When a car had visited all waypoints (B, C, D) and arrived at intersection I00, the route planner mapped destination "A" to intersection "I00" (already reached), but then could not find an outgoing segment ending at "I00" — causing the car to loop indefinitely. A targeted fix was added: when `current_destination == "A"` and the car is at I00, take the `I00_A` endpoint segment directly.

---

## 3. Car Lifecycle and Fleet Management

Cars are managed by the simulator without any changes to the group controllers:

- **Initial spawn:** up to 2 cars placed on `A_I00` (slots 0 and 1), the only entry segment (length = 2 slots).
- **Respawn:** after each step, if the active car count is below the target and slot 0 of `A_I00` is free, a new car is injected. This maintains a steady fleet even as cars complete tours.
- **Tour completion:** when a car's `active` flag transitions from True to False (it moved off the last slot of `I00_A`), the simulator increments the completed-tour counter and triggers a respawn.
- **Destinations:** new cars cycle through starting destinations B → C → D → B → ... to distribute initial routes across the grid.

---

## 4. Test Results

Three test scenarios were run. Full output is in `simulator/logs/`. Log files are written automatically on every run.

### Test 1 — Safety Demo: 2 Cars, 100 Steps (verbose)

**Purpose:** A readable per-step log showing signal states, car positions, and violation status. This is the primary evidence artifact for the report.

**Log file:** `simulator/logs/test1_safety_2cars_100steps.txt` (818 lines)

**Excerpt (steps 10–12, the moment cars first move):**

```
-- Step 10 --------------------------------------------
   Signal I00: green=E        ← i-group switched phase after min_green=10 steps
   Signal I01: green=N
   Signal I11: green=N
   Car 1: STAY          seg=A_I00  slot= 0  visited=[-]  next=B
   Car 2: STRAIGHT      seg=I00_I01  slot= 0  visited=[-]  next=C
   Verify: CLEAN | v/i-group: AGREE

-- Step 11 --------------------------------------------
   Signal I00: green=E
   Signal I01: green=N
   Signal I11: green=N
   Car 1: MOVE          seg=A_I00  slot= 1  visited=[-]  next=B
   Car 2: MOVE          seg=I00_I01  slot= 1  visited=[-]  next=C
   Verify: CLEAN | v/i-group: AGREE

-- Step 12 --------------------------------------------
   Signal I00: green=E
   Signal I01: green=N
   Signal I11: green=N
   Car 1: STRAIGHT      seg=I00_I01  slot= 0  visited=[-]  next=B
   Car 2: MOVE          seg=I00_I01  slot= 2  visited=[-]  next=C
   Verify: CLEAN | v/i-group: AGREE
```

**Final summary:**

```
Steps simulated   : 100 (200s = 0.056 hr)
Target fleet size : 2 cars
Completed tours   : 0
Throughput        : 0.0 tours/hr
Report mismatches : 0
v_group stats : {collisions: 0, red_light_violations: 0,
                 wrong_way_violations: 0, illegal_turn_violations: 0,
                 multi_crossing_violations: 0}
i_group stats : {collisions: 0, red_light_violations: 0,
                 wrong_way_violations: 0, multi_crossing_violations: 0}
```

Note: 0 completed tours is expected at 100 steps. A full A→B→C→D→A tour requires a minimum of 244 slot-advances plus signal wait time (~320–340 steps total).

---

### Test 2 — Safety Extended: 4 Cars, 400 Steps (quiet)

**Purpose:** Verify the system handles 4 simultaneously active cars with zero violations and completes at least one full tour.

**Log file:** `simulator/logs/test2_extended_4cars_400steps.txt`

```
Steps simulated   : 400 (800s = 0.222 hr)
Target fleet size : 4 cars
Completed tours   : 4
Throughput        : 18.0 tours/hr
Report mismatches : 0
v_group stats : {collisions: 0, red_light_violations: 0,
                 wrong_way_violations: 0, illegal_turn_violations: 0,
                 multi_crossing_violations: 0}
i_group stats : {collisions: 0, red_light_violations: 0,
                 wrong_way_violations: 0, multi_crossing_violations: 0}
```

**Result:** 4 complete tours, zero violations of any kind, v-group and i-group in full agreement on every step.

---

### Test 3 — Throughput Benchmark: 6 Cars, 1800 Steps = 1 Simulated Hour

**Purpose:** Measure throughput over exactly one simulated hour (1800 steps × 2 s/step = 3600 s). This is the project's primary evaluation metric.

**Log file:** `simulator/logs/test3_throughput_6cars_1hour.txt`

```
Steps simulated   : 1800 (3600s = 1.000 hr)
Target fleet size : 6 cars
Completed tours   : 33
Throughput        : 33.0 tours/hr
Report mismatches : 0
v_group stats : {collisions: 0, red_light_violations: 0,
                 wrong_way_violations: 0, illegal_turn_violations: 0,
                 multi_crossing_violations: 0}
i_group stats : {collisions: 0, red_light_violations: 0,
                 wrong_way_violations: 0, multi_crossing_violations: 0}
```

**Result:** **33 tours/hr** with zero violations across all 1800 steps and full v/i-group agreement.

---

## 5. Summary of Evidence

| Requirement | Evidence |
|---|---|
| v-group sees traffic light at every intersection | `Simulator._step()` writes `signal_actions` into `state.signals` (Step 2) before calling `v_ctrl.decide_actions()` (Step 3). Per-step log shows cars STAY at red signals and move on green. |
| i-group sees all vehicle locations | `i_ctrl.verify_step()` and `i_ctrl.compute_signals()` both iterate over `new_state.cars`, which contains every car's current segment and slot. |
| Both groups agree on violations | `Report mismatches: 0` across all 1800 steps in Test 3. All shared counters (collisions, red-light, wrong-way, multi-crossing) are identical between v-group and i-group stats. |
| No collisions | 0 across all tests |
| No red-light violations | 0 across all tests |
| No wrong-way driving | 0 across all tests |
| No illegal turns (U-turns) | 0 across all tests |
| No multi-crossing violations | 0 across all tests |
| Throughput > 0 | **33.0 tours/hr** (Test 3, 6 cars, 1 simulated hour) |

---

## 6. How to Reproduce

From the project root:

```bash
python -m simulator.test_integration
```

This runs all three tests and writes log files to `simulator/logs/`. All tests pass with the above results.
