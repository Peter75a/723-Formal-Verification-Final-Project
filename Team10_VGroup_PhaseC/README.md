# Team 10 - V-group - Phase C

ECEN 723, Spring 2026
Authors: Dhruv Nandwani, Brian Hsieh

This folder contains the v-group's Phase C verification of the i-group's
signal system. We use the SPIN model checker (Promela) to verify four
LTL properties of the controller in `i_group/scheduler.py` and
`i_group/policy.py`.

## What's in this folder

| File | Purpose |
|------|---------|
| `Team10_VGroup_PhaseC_Report.docx` | The full report (read this first). |
| `signal_check.pml` | Promela model of the controller at the worst-case interior intersection I11. Contains LTL claims for P1 (every direction eventually green), P2 (every direction green infinitely often), and P3 (exactly one direction green at any time). |
| `signal_check_corner.pml` | Same controller at the corner intersection I00 (valid set = {S, E}), with `phase` initialised to N. Demonstrates the topology-validity bug (P4) in the original `scheduler.initialize`. |
| `signal_check_corner_fixed.pml` | Same as above but with the proposed fix applied (`phase` initialised to E). Re-verifies P4. |
| `run_all.sh` | Driver script. Runs every property in sequence and writes per-property logs to `logs/`. |
| `commands.md` | Manual command reference, in case you want to reproduce one property at a time or take fresh screenshots. |
| `logs/` | Verifier output for each property (populated by `run_all.sh`). |

## How to reproduce the verification

Prerequisites (macOS, both already required by the i-group's Phase B):

```
brew install spin
xcode-select --install      # provides gcc / clang
```

Then, from inside this folder:

```
bash run_all.sh
```

This takes well under a minute. At the end it prints a summary of
`errors:` lines across all logs. Expected output:

```
p1_eventually_N.txt        errors: 0
p1_eventually_S.txt        errors: 0
p1_eventually_E.txt        errors: 0
p1_eventually_W.txt        errors: 0
p2_inf_often_N.txt         errors: 0
p2_inf_often_S.txt         errors: 0
p2_inf_often_E.txt         errors: 0
p2_inf_often_W.txt         errors: 0
p3_one_green.txt           errors: 0
p4_topology_valid_original.txt    errors: 1   <-- deliberate counterexample
p4_topology_valid_fixed.txt       errors: 0
```

The single `errors: 1` is the deliberate counterexample at I00 with the
unfixed scheduler; it is replayed at depth 0 (the initial state) and
written to `p4_counterexample_trail.txt`.

If you only want to reproduce one property at a time (e.g. to take a
fresh screenshot), see `commands.md` for the individual three-step
`spin -a` / `gcc` / `./pan` invocations.

## Properties verified

| ID | Property | Type | File | LTL claim |
|----|----------|------|------|-----------|
| P1 | Every direction eventually green | Liveness | `signal_check.pml` | `<>(phase == d)` for d in {N,S,E,W} |
| P2 | Every direction infinitely often green | Liveness | `signal_check.pml` | `[]<>(phase == d)` for d in {N,S,E,W} |
| P3 | Exactly one direction green at any time | Safety | `signal_check.pml` | `[](exactly_one_of(green_N, green_S, green_E, green_W))` |
| P4 | Topology validity at corner I00 (self-chosen) | Safety | `signal_check_corner*.pml` | `[](phase == S \|\| phase == E)` |

Liveness properties are verified with weak fairness (`./pan -a -f`).

## Note on the LTL-claim selection mechanism

`signal_check.pml` wraps each LTL claim in `#ifdef SEL_<name>` blocks
and is verified one property at a time by passing `-DSEL_<name>` on
the `spin -a` command line. This avoids the `-ltl <name>` flag, which
is parsed inconsistently across SPIN builds (the brew-installed SPIN
on macOS, in particular, treats the property name as the input
filename and silently fails). The `-DSEL_<name>` form is portable
across all SPIN versions. See Section 4 of the report for details.
