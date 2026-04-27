/*
 * ============================================================
 * signal_check.pml  --  Team 10  Phase C  v-group   (rev 3)
 *
 * PURPOSE:
 *   Model the i-group signal system in Promela so SPIN can
 *   verify the 3 mandatory v-group properties (P1, P2, P3) at
 *   the worst-case 4-direction intersection (I11).
 *
 * REV 3 NOTES:
 *
 *   (1) Globals are given explicit initial values inline so that
 *       the very first state already satisfies P3.  Spin evaluates
 *       LTL formulas at the INITIAL state, before any process has
 *       stepped, so initialising in the controller's first atomic
 *       block is too late.
 *
 *   (2) The LTL claims are wrapped in #ifdef SEL_<name> blocks.
 *       Pass exactly one such define on the spin command line
 *       (e.g. spin -a -DSEL_p3_one_green ...).  Only the matching
 *       claim survives preprocessing, and Spin uses it as the
 *       single never claim.  This avoids the `-ltl <name>` flag,
 *       which is parsed inconsistently across Spin versions and
 *       causes spin to mistake the claim name for the input file.
 *
 * HOW TO RUN:
 *   Safety  (P3):
 *     spin -a -DSEL_p3_one_green signal_check.pml
 *     gcc -O2 -o pan pan.c
 *     ./pan
 *
 *   Liveness (P1, P2) -- pass -f to ./pan for weak fairness:
 *     spin -a -DSEL_p1_N signal_check.pml
 *     gcc -O2 -o pan pan.c
 *     ./pan -a -f
 *     ... and similarly for SEL_p1_S, SEL_p1_E, SEL_p1_W,
 *         SEL_p2_N, SEL_p2_S, SEL_p2_E, SEL_p2_W.
 *
 *   On failure, replay counterexample:  spin -t signal_check.pml
 * ============================================================
 */


/* ============================================================
 * CONSTANTS
 * ============================================================ */
#define N 0
#define S 1
#define E 2
#define W 3

#define MIN_GREEN 10
#define MAX_GREEN 40


/* ============================================================
 * GLOBAL STATE (with explicit initial values)
 * ============================================================ */
byte phase       = N;
byte timer       = 0;
byte cycle_index = 0;

bool green_N = 1;   /* matches phase = N at depth 0 */
bool green_S = 0;
bool green_E = 0;
bool green_W = 0;


/* ============================================================
 * INLINE: switch_phase_to(d)
 * ============================================================ */
inline switch_phase_to(d)
{
    phase       = d;
    timer       = 0;
    cycle_index = (d + 1) % 4;
    green_N     = (d == N);
    green_S     = (d == S);
    green_E     = (d == E);
    green_W     = (d == W)
}


/* ============================================================
 * CONTROLLER PROCESS  (I11 -- all 4 directions valid)
 *
 * Faithful to policy.SignalPolicy.decide() under uniform demand
 * cnt[d] = 1.  The voluntary-switch branch is suppressed because
 * best_other_count == current_count fails the strict-greater
 * test in policy.py, so phase changes are driven by MAX_GREEN
 * with cycle_priority breaking ties.
 * ============================================================ */
active proctype Controller()
{
    byte d;
    byte dist;
    byte best;
    byte best_dist;
    bool found;

    do
    :: atomic {
        /* Step A: scan {0,1,2,3} \ {phase}, keep the candidate
         * with smallest distance from cycle_index. */
        found     = 0;
        best      = 0;
        best_dist = 0;
        d         = 0;
        do
        :: (d == 4) -> break
        :: (d <  4) ->
            if
            :: (d != phase) ->
                dist = (d + 4 - cycle_index) % 4;
                if
                :: (!found) ->
                    found = 1;
                    best  = d;
                    best_dist = dist
                :: (found && dist < best_dist) ->
                    best  = d;
                    best_dist = dist
                :: else -> skip
                fi
            :: else -> skip
            fi;
            d = d + 1
        od;

        /* Step B: apply the policy decision. */
        if
        :: (timer >= MAX_GREEN) -> switch_phase_to(best)
        :: (timer <  MAX_GREEN) -> timer = timer + 1
        fi
    }
    od
}


init { skip }


/* ============================================================
 * LTL PROPERTIES  --  exactly one is selected at compile time
 * via the preprocessor define SEL_<name>.
 * ============================================================ */

#ifdef SEL_p1_N
ltl p1_eventually_N { <> (phase == N) }
#endif
#ifdef SEL_p1_S
ltl p1_eventually_S { <> (phase == S) }
#endif
#ifdef SEL_p1_E
ltl p1_eventually_E { <> (phase == E) }
#endif
#ifdef SEL_p1_W
ltl p1_eventually_W { <> (phase == W) }
#endif

#ifdef SEL_p2_N
ltl p2_inf_often_N { [] <> (phase == N) }
#endif
#ifdef SEL_p2_S
ltl p2_inf_often_S { [] <> (phase == S) }
#endif
#ifdef SEL_p2_E
ltl p2_inf_often_E { [] <> (phase == E) }
#endif
#ifdef SEL_p2_W
ltl p2_inf_often_W { [] <> (phase == W) }
#endif

#ifdef SEL_p3_one_green
ltl p3_one_green {
    [] ( ( green_N && !green_S && !green_E && !green_W) ||
         (!green_N &&  green_S && !green_E && !green_W) ||
         (!green_N && !green_S &&  green_E && !green_W) ||
         (!green_N && !green_S && !green_E &&  green_W) )
}
#endif
