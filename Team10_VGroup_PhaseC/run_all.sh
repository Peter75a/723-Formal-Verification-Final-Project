#!/usr/bin/env bash
#
# run_all.sh  --  Team 10 V-group Phase C verification driver  (rev 3)
#
# Each verification is split into the three explicit Spin steps:
#     1.  spin -a [-DSEL_<name>] <file>     (generate pan.c)
#     2.  gcc -O2 -o pan pan.c              (compile the verifier)
#     3.  ./pan [pan options]               (run it)
#
# In signal_check.pml the LTL claims are wrapped in #ifdef SEL_<name>
# blocks; passing -DSEL_<name> on the spin command line activates one
# claim and strips the rest at preprocessing time.  This avoids the
# `-ltl <name>` flag, which is parsed inconsistently across Spin
# versions on macOS and causes spin to mistake the property name for
# the input file.
#
# Usage:  bash run_all.sh
# Prereqs: Spin installed (brew install spin), gcc, .pml files in CWD.
#

mkdir -p logs

cleanup () {
    rm -f pan pan.* _spin_nvr.tmp
}

# args: <file> <define-or-empty> <outname>
run_safety () {
    local file="$1"
    local def="$2"
    local outname="$3"
    local outfile="logs/${outname}.txt"

    echo "============================================================" | tee    "$outfile"
    echo "  $outname  ($file, $def)  -- SAFETY"                          | tee -a "$outfile"
    echo "============================================================" | tee -a "$outfile"

    cleanup
    if [ -z "$def" ]; then
        spin -a "$file"                  2>&1 | tee -a "$outfile"
    else
        spin -a "$def" "$file"           2>&1 | tee -a "$outfile"
    fi
    gcc  -O2 -o pan pan.c                2>&1 | tee -a "$outfile"
    ./pan                                2>&1 | tee -a "$outfile"
    echo                                      | tee -a "$outfile"
}

# args: <file> <define> <outname>
run_liveness () {
    local file="$1"
    local def="$2"
    local outname="$3"
    local outfile="logs/${outname}.txt"

    echo "============================================================" | tee    "$outfile"
    echo "  $outname  ($file, $def)  -- LIVENESS, weak fairness"         | tee -a "$outfile"
    echo "============================================================" | tee -a "$outfile"

    cleanup
    spin -a "$def" "$file"               2>&1 | tee -a "$outfile"
    gcc  -O2 -o pan pan.c                2>&1 | tee -a "$outfile"
    ./pan -a -f                          2>&1 | tee -a "$outfile"
    echo                                      | tee -a "$outfile"
}

echo "Team 10 V-group Phase C  --  Verification Run  (rev 3)"
echo "Date: $(date)"
echo

# ---- 0. Environment ----------------------------------------
{
    echo "=== Spin version ==="
    spin -V
    echo
    echo "=== gcc version ==="
    gcc --version | head -1
} | tee logs/00_spin_version.txt
echo

# ---- P3: safety, exactly-one-green ------------------------
run_safety   signal_check.pml         -DSEL_p3_one_green   p3_one_green

# ---- P1: liveness, eventually green (4 directions) --------
run_liveness signal_check.pml         -DSEL_p1_N           p1_eventually_N
run_liveness signal_check.pml         -DSEL_p1_S           p1_eventually_S
run_liveness signal_check.pml         -DSEL_p1_E           p1_eventually_E
run_liveness signal_check.pml         -DSEL_p1_W           p1_eventually_W

# ---- P2: liveness, infinitely often green (4 directions) ---
run_liveness signal_check.pml         -DSEL_p2_N           p2_inf_often_N
run_liveness signal_check.pml         -DSEL_p2_S           p2_inf_often_S
run_liveness signal_check.pml         -DSEL_p2_E           p2_inf_often_E
run_liveness signal_check.pml         -DSEL_p2_W           p2_inf_often_W

# ---- P4 (original)  -- expected counterexample (errors: 1) ----
# Only one ltl claim in this file; no -D needed, spin auto-selects it.
run_safety   signal_check_corner.pml  ""                   p4_topology_valid_original

# Replay the counterexample trail
echo "=== P4 counterexample trail replay ==="            | tee    logs/p4_counterexample_trail.txt
spin -t signal_check_corner.pml 2>&1                     | tee -a logs/p4_counterexample_trail.txt
echo                                                     | tee -a logs/p4_counterexample_trail.txt

# ---- P4 (fixed)  -- expected pass (errors: 0) ----------------
run_safety   signal_check_corner_fixed.pml  ""             p4_topology_valid_fixed

cleanup

# ---- Summary ------------------------------------------------
echo
echo "============================================================"
echo "  SUMMARY (grep 'errors:' across all logs)"
echo "============================================================"
for f in logs/p*.txt; do
    line=$(grep -m1 -E "errors:" "$f" 2>/dev/null)
    printf "%-45s  %s\n" "$(basename "$f")" "$line"
done

echo
echo "Done. Logs saved under ./logs/."
echo "Expected: errors: 0 everywhere except p4_topology_valid_original.txt,"
echo "which should show errors: 1 (the deliberate counterexample)."
