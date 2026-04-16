# simulator/test_integration.py

"""
B1 Integration tests for the Simulator.

Three test scenarios:

  Test 1 -- Safety (small):
    2 cars, 100 steps.  verbose=True.
    Per-step log + summary written to logs/test1_safety_2cars_100steps.txt
    Use this file as the "readable demo" appendix in your Phase B report.

  Test 2 -- Safety (extended):
    4 cars, 400 steps (~13 min simulated).  verbose=False.
    Only the final summary is written to logs/test2_extended_4cars_400steps.txt

  Test 3 -- Throughput (one simulated hour):
    6 cars, 1800 steps (= 1 hr at 2 s/step).  verbose=False.
    Only the final summary is written to logs/test3_throughput_6cars_1hour.txt
    The throughput figure here is what you report for Phase B/C.

How to run (from project root):
    python -m simulator.test_integration

Log files are written to:
    723-Formal-Verification-Final-Project/simulator/logs/
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from shared.topology import Topology
from i_group.controller import InfrastructureController
from v_group.controller import VehicleController
from simulator.simulator import Simulator

# All log files go into simulator/logs/
_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_sim() -> tuple:
    """Return a fresh (topo, v_ctrl, i_ctrl, sim) for each test."""
    topo   = Topology()
    topo.build()
    v_ctrl = VehicleController(topo)
    i_ctrl = InfrastructureController(topo)
    sim    = Simulator(topo, v_ctrl, i_ctrl)
    return topo, v_ctrl, i_ctrl, sim


def _log_path(filename: str) -> str:
    """Return absolute path for a log file inside simulator/logs/."""
    return os.path.join(_LOG_DIR, filename)


def _assert_zero_violations(summary: dict, label: str) -> None:
    """Assert that both groups report zero violations of every kind."""
    v = summary["v_stats"]
    i = summary["i_stats"]

    checks = [
        ("total_collisions",                v, i),
        ("total_red_light_violations",      v, i),
        ("total_wrong_way_violations",      v, i),
        ("total_multi_crossing_violations", v, i),
    ]
    for key, v_stats, i_stats in checks:
        assert v_stats[key] == 0, f"[{label}] v-group {key} = {v_stats[key]}"
        assert i_stats[key] == 0, f"[{label}] i-group {key} = {i_stats[key]}"

    assert v["total_illegal_turn_violations"] == 0, \
        f"[{label}] v-group illegal_turn_violations = {v['total_illegal_turn_violations']}"

    assert summary["mismatches"] == 0, \
        f"[{label}] {summary['mismatches']} step(s) with v/i-group disagreement"


def _assert_groups_agree(summary: dict, label: str) -> None:
    """Assert v-group and i-group counters match on all shared keys."""
    v = summary["v_stats"]
    i = summary["i_stats"]
    shared_keys = [
        "total_collisions",
        "total_red_light_violations",
        "total_wrong_way_violations",
        "total_multi_crossing_violations",
    ]
    for key in shared_keys:
        assert v[key] == i[key], (
            f"[{label}] Mismatch on {key}: v={v[key]}, i={i[key]}"
        )


# ---------------------------------------------------------------------------
# Test 1: Safety demo -- 2 cars, 100 steps, full verbose log
# ---------------------------------------------------------------------------

def test_safety_2cars_100steps():
    """
    Small safety smoke test with full per-step output.

    verbose=True  → every step is printed to console AND written to the log file.
    Use the log file as the readable demo appendix in your Phase B report.

    Log file: simulator/logs/test1_safety_2cars_100steps.txt
    """
    log_file = _log_path("test1_safety_2cars_100steps.txt")

    print("\n" + "=" * 60)
    print("TEST 1: Safety -- 2 cars, 100 steps (verbose)")
    print(f"  Log -> {log_file}")
    print("=" * 60)

    _, _, _, sim = _make_sim()
    summary = sim.run(steps=100, num_cars=2, verbose=True, log_file=log_file)

    _assert_zero_violations(summary, "Test1")
    _assert_groups_agree(summary, "Test1")

    print("\n[PASS] Test 1 passed.")
    print(f"  Log written to: {log_file}")
    return summary


# ---------------------------------------------------------------------------
# Test 2: Safety extended -- 4 cars, 400 steps, summary-only log
# ---------------------------------------------------------------------------

def test_safety_extended_4cars_400steps():
    """
    Extended safety test: 4 cars sustained over 400 steps (~13 min simulated).

    verbose=False → no per-step output on console or in file.
    Final summary is written to the log file.

    Asserts:
      - Zero violations of all kinds
      - v/i-group agree on all counters
      - At least 1 completed tour (proves end-to-end routing works)

    Log file: simulator/logs/test2_extended_4cars_400steps.txt
    """
    log_file = _log_path("test2_extended_4cars_400steps.txt")

    print("\n" + "=" * 60)
    print("TEST 2: Safety extended -- 4 cars, 400 steps (quiet)")
    print(f"  Log -> {log_file}")
    print("=" * 60)

    _, _, _, sim = _make_sim()
    summary = sim.run(steps=400, num_cars=4, verbose=False, log_file=log_file)

    _assert_zero_violations(summary, "Test2")
    _assert_groups_agree(summary, "Test2")

    assert summary["completed_tours"] >= 1, (
        f"[Test2] Expected at least 1 completed tour in 400 steps with 4 cars, "
        f"got {summary['completed_tours']}."
    )

    print(f"\n[PASS] Test 2 passed.")
    print(f"  Completed tours : {summary['completed_tours']}")
    print(f"  Log written to  : {log_file}")
    return summary


# ---------------------------------------------------------------------------
# Test 3: Throughput -- 6 cars, 1800 steps = 1 simulated hour, summary-only log
# ---------------------------------------------------------------------------

def test_throughput_6cars_1hour():
    """
    Throughput benchmark: 6 cars over 1800 steps = exactly 1 simulated hour.

    verbose=False → no per-step output on console or in file.
    Final summary (including throughput figure) is written to the log file.

    Per the project spec, the system is evaluated on:
      - Throughput (tours/hr)         <- primary metric
      - Collisions
      - Illegal directions / U-turns
      - Red-light violations

    Asserts:
      - Zero violations of all kinds
      - v/i-group agree on all counters
      - Throughput > 0 tours/hr

    Log file: simulator/logs/test3_throughput_6cars_1hour.txt
    """
    log_file = _log_path("test3_throughput_6cars_1hour.txt")

    print("\n" + "=" * 60)
    print("TEST 3: Throughput -- 6 cars, 1800 steps = 1 hr (quiet)")
    print(f"  Log -> {log_file}")
    print("=" * 60)

    _, _, _, sim = _make_sim()
    summary = sim.run(steps=1800, num_cars=6, verbose=False, log_file=log_file)

    _assert_zero_violations(summary, "Test3")
    _assert_groups_agree(summary, "Test3")

    assert summary["throughput_per_hr"] > 0, (
        f"[Test3] Throughput is 0 tours/hr after 1800 steps with 6 cars. "
        f"Completed tours: {summary['completed_tours']}."
    )

    print(f"\n[PASS] Test 3 passed.")
    print(f"  Throughput      : {summary['throughput_per_hr']} tours/hr")
    print(f"  Completed tours : {summary['completed_tours']}")
    print(f"  Log written to  : {log_file}")
    return summary


# ---------------------------------------------------------------------------
# Test 4: Bottleneck search -- increase cars until throughput drops or violations appear
# ---------------------------------------------------------------------------

def test_bottleneck_search(start_cars: int = 6, max_cars: int = 60, step_size: int = 2):
    """
    Sweep num_cars from start_cars up to max_cars (step = step_size).
    Each run is 1800 steps (1 simulated hour).

    When a stop condition is triggered (violation or throughput drop), continues
    for 5 more data points before stopping, then always runs max_cars as the
    extreme validation point.

    Prints a summary table to console and saves full log to:
        simulator/logs/test4_bottleneck_search.txt
    """
    import datetime

    log_file = _log_path("test4_bottleneck_search.txt")

    header_lines = [
        "=" * 60,
        "TEST 4: Bottleneck search",
        f"  Cars range : {start_cars} → {max_cars}, step={step_size}",
        f"  Each run   : 1800 steps = 1 simulated hour",
        f"  On trigger : runs 5 more points, then jumps to {max_cars} cars",
        f"  Run time   : {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        "=" * 60,
    ]

    def emit(line: str = "") -> None:
        """Print to console and append to log file."""
        print(line)
        with open(log_file, "a", encoding="utf-8") as f:
            f.write(line + "\n")

    # Clear / create log file
    with open(log_file, "w", encoding="utf-8") as f:
        f.write("")

    for line in header_lines:
        emit(line)

    best_cars        = start_cars
    best_throughput  = 0.0
    prev_throughput  = 0.0
    results          = []
    triggered_at     = None

    car_list = list(range(start_cars, max_cars + 1, step_size))
    if car_list[-1] != max_cars:
        car_list.append(max_cars)

    i_car = 0
    while i_car < len(car_list):
        num_cars = car_list[i_car]
        i_car += 1

        _, _, _, sim = _make_sim()
        summary = sim.run(steps=1800, num_cars=num_cars, verbose=False)

        throughput = summary["throughput_per_hr"]
        v  = summary["v_stats"]
        i_s = summary["i_stats"]

        violation_keys = [
            "total_collisions",
            "total_red_light_violations",
            "total_wrong_way_violations",
            "total_multi_crossing_violations",
        ]
        has_violation = (
            any(v[k] > 0 or i_s[k] > 0 for k in violation_keys)
            or v.get("total_illegal_turn_violations", 0) > 0
            or summary["mismatches"] > 0
        )

        drop   = (len(results) > 0 and throughput < prev_throughput)
        status = "VIOLATION" if has_violation else ("DROP" if drop else "ok")
        results.append((num_cars, throughput, status, summary["v_stats"], summary["i_stats"]))

        emit(f"  {num_cars:3d} cars -> {throughput:6.1f} tours/hr  [{status}]")

        if triggered_at is None and (has_violation or drop):
            triggered_at = num_cars
            reason = "violation" if has_violation else "throughput drop"
            emit(f"\n  *** {reason} at {num_cars} cars — running 5 more points ***\n")

            next5    = list(range(num_cars + step_size,
                                  num_cars + step_size * 6,
                                  step_size))
            remaining = [c for c in next5 if c < max_cars]
            car_list  = remaining + [max_cars]
            i_car     = 0
            prev_throughput = throughput
            continue

        if throughput > best_throughput and status == "ok":
            best_throughput = throughput
            best_cars       = num_cars

        prev_throughput = throughput

    # Summary table
    emit("")
    emit("-" * 60)
    emit(f"  Best result : {best_cars} cars -> {best_throughput} tours/hr")
    if triggered_at:
        emit(f"  Bottleneck  : first triggered at {triggered_at} cars")
    emit("-" * 60)
    emit("")
    emit(f"  {'Cars':>5}  {'Throughput':>12}  {'Collisions':>10}  {'RedLight':>9}  {'IllegalDir':>10}  {'U-turns':>8}  Status")
    emit(f"  {'-'*5}  {'-'*12}  {'-'*10}  {'-'*9}  {'-'*10}  {'-'*8}  {'-'*10}")
    for cars, tput, status, v_s, i_s in results:
        marker = " <-- bottleneck" if cars == triggered_at else ""
        emit(
            f"  {cars:>5}  {tput:>10.1f}/hr"
            f"  {v_s['total_collisions']:>10}"
            f"  {v_s['total_red_light_violations']:>9}"
            f"  {v_s['total_wrong_way_violations']:>10}"
            f"  {v_s.get('total_illegal_turn_violations', 0):>8}"
            f"  {status}{marker}"
        )

    emit("")
    emit(f"  Log saved to: {log_file}")
    emit("=" * 60)

    return best_cars, best_throughput, results


# ---------------------------------------------------------------------------
# Entry point: run all tests
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys
    os.makedirs(_LOG_DIR, exist_ok=True)

    # If called with --bottleneck, only run the bottleneck search
    if "--bottleneck" in sys.argv:
        test_bottleneck_search(start_cars=6, max_cars=60, step_size=2)
    else:
        results = {}
        results["test1"] = test_safety_2cars_100steps()
        results["test2"] = test_safety_extended_4cars_400steps()
        results["test3"] = test_throughput_6cars_1hour()

        print("\n" + "=" * 60)
        print("ALL TESTS PASSED")
        print(f"  Test 1 throughput : {results['test1']['throughput_per_hr']} tours/hr  (100 steps)")
        print(f"  Test 2 throughput : {results['test2']['throughput_per_hr']} tours/hr  (400 steps)")
        print(f"  Test 3 throughput : {results['test3']['throughput_per_hr']} tours/hr  (1800 steps)")
        print(f"\nLog files written to: {_LOG_DIR}/")
        print(f"  test1_safety_2cars_100steps.txt    <- use this in your report")
        print(f"  test2_extended_4cars_400steps.txt")
        print(f"  test3_throughput_6cars_1hour.txt   <- throughput figure for report")
        print("=" * 60)
