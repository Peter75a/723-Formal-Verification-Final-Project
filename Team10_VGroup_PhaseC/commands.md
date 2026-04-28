# Phase C Verification - Commands & Screenshot Plan (rev 3)

Team 10 V-group. Each verification is split into three explicit steps
(spin generate -> gcc compile -> pan run). The LTL property is selected
with a preprocessor flag (-DSEL_<name>) rather than `-ltl <name>`,
because the brew-installed Spin on macOS parses `-ltl` inconsistently
and treats the claim name as the input file.

To run everything at once and dump logs under `./logs/`:

```
bash run_all.sh
```

The sections below list the individual commands so the run can be
reproduced property-by-property and so the right output is on screen at
screenshot time. **Between commands, always clean up the previous build:**

```
rm -f pan pan.* _spin_nvr.tmp
```

`run_all.sh` does this automatically.

---

## 0. Environment

```
spin -V
gcc --version | head -1
```

**Screenshot S0** - Spin banner.

---

## 1. P3 - Safety: exactly one direction green (signal_check.pml)

```
rm -f pan pan.* _spin_nvr.tmp
spin -a -DSEL_p3_one_green signal_check.pml
gcc -O2 -o pan pan.c
./pan
```

Expected:

```
State-vector NN byte, depth reached MMM, errors: 0
```

**Screenshot S1** - the `errors: 0` summary block.

---

## 2. P1 - Liveness: every direction eventually green (signal_check.pml)

`-a` to pan tells it to look for acceptance cycles (required for
liveness). `-f` enables weak fairness so the controller is not allowed
to stutter forever.

```
rm -f pan pan.* _spin_nvr.tmp
spin -a -DSEL_p1_N signal_check.pml
gcc -O2 -o pan pan.c
./pan -a -f

rm -f pan pan.* _spin_nvr.tmp
spin -a -DSEL_p1_S signal_check.pml
gcc -O2 -o pan pan.c
./pan -a -f

rm -f pan pan.* _spin_nvr.tmp
spin -a -DSEL_p1_E signal_check.pml
gcc -O2 -o pan pan.c
./pan -a -f

rm -f pan pan.* _spin_nvr.tmp
spin -a -DSEL_p1_W signal_check.pml
gcc -O2 -o pan pan.c
./pan -a -f
```

Each must print `errors: 0`.

**Screenshot S2** - one direction (W is the round-robin worst case)
showing `errors: 0`.

---

## 3. P2 - Liveness: every direction green infinitely often (signal_check.pml)

```
rm -f pan pan.* _spin_nvr.tmp
spin -a -DSEL_p2_N signal_check.pml
gcc -O2 -o pan pan.c
./pan -a -f

rm -f pan pan.* _spin_nvr.tmp
spin -a -DSEL_p2_S signal_check.pml
gcc -O2 -o pan pan.c
./pan -a -f

rm -f pan pan.* _spin_nvr.tmp
spin -a -DSEL_p2_E signal_check.pml
gcc -O2 -o pan pan.c
./pan -a -f

rm -f pan pan.* _spin_nvr.tmp
spin -a -DSEL_p2_W signal_check.pml
gcc -O2 -o pan pan.c
./pan -a -f
```

Each must print `errors: 0`.

**Screenshot S3** - `errors: 0` for `p2_inf_often_W`.

---

## 4. P4 - Topology validity at corner I00, ORIGINAL scheduler

The corner files contain a single ltl claim, so no `-D` flag is needed
- spin auto-selects the only claim.

```
rm -f pan pan.* _spin_nvr.tmp
spin -a signal_check_corner.pml
gcc -O2 -o pan pan.c
./pan
```

Expected:

```
pan: ltl formula p4_topology_valid violated
...
errors: 1
```

**Screenshot S4** - `errors: 1` and the violation message.

Replay the counterexample trail:

```
spin -t signal_check_corner.pml
```

Expected: a 1-state trail showing `phase = 0` (i.e. N).

**Screenshot S5** - the trail replay output.

---

## 5. P4 - Topology validity at corner I00, FIXED scheduler

```
rm -f pan pan.* _spin_nvr.tmp
spin -a signal_check_corner_fixed.pml
gcc -O2 -o pan pan.c
./pan
```

Expected:

```
errors: 0
```

**Screenshot S6** - `errors: 0`.

---

## Screenshot inventory (for the report appendix)

| # | What | Notes |
|---|------|-------|
| S0 | Spin / gcc versions | `spin -V` |
| S1 | P3 errors:0 (safety) | one screenshot |
| S2 | P1 errors:0, e.g. W | representative |
| S3 | P2 errors:0, e.g. W | representative |
| S4 | P4 errors:1 (counterexample) | original scheduler |
| S5 | P4 trail replay (`spin -t`) | shows phase = 0 / N |
| S6 | P4 errors:0 after fix | fixed scheduler |

7 screenshots total.

---

## Why the `-DSEL_<name>` form

Each LTL claim in `signal_check.pml` is wrapped in `#ifdef SEL_<name>`.
Passing `-DSEL_<name>` on the spin command line activates exactly one
claim during preprocessing; the rest are stripped. Spin then sees a
file containing a single ltl claim and uses it as the never claim, no
`-ltl` flag required. This is portable across all Spin versions and
sidesteps the macOS-Homebrew Spin's broken `-ltl <name>` parsing.
