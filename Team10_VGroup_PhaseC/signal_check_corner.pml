/*
 * ============================================================
 * signal_check_corner.pml  --  Team 10  Phase C  v-group  (rev 2)
 *
 * PURPOSE:
 *   Verify v-group spec property #4 (self-chosen) -- topology
 *   validity at I00, where only S and E are valid.  This file
 *   encodes the ORIGINAL i_group/scheduler.py, which initialises
 *   every intersection with default_phase="N".  At I00, that
 *   "N" is invalid -- and Spin will report it.
 *
 * REV 2 NOTE:
 *   The bug being demonstrated is precisely that the initial
 *   value of phase is "N" (= 0), an invalid direction at I00.
 *   We declare phase = N inline so the very first state of the
 *   model carries that invalid value, exactly as the simulator
 *   does at start-up.
 *
 * HOW TO RUN:
 *     spin -a -ltl p4_topology_valid signal_check_corner.pml
 *     gcc -O2 -o pan pan.c
 *     ./pan
 *
 *   Expected: errors: 1 (counterexample at depth 0).
 *   Replay:   spin -t signal_check_corner.pml
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

/* I00 valid directions = {S, E}. */
#define IS_VALID(d) ((d == S) || (d == E))


/* ============================================================
 * GLOBAL STATE  (matches scheduler.initialize(default_phase="N"))
 * ============================================================ */
byte phase       = N;     /* THE BUG: invalid at I00 */
byte timer       = 0;
byte cycle_index = 0;

bool green_N = true;      /* matches phase = N */
bool green_S = false;
bool green_E = false;
bool green_W = false;


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
 * CONTROLLER PROCESS  (I00 -- valid = {S, E})
 *
 * Includes the topology-invalid correction branch from policy.py,
 * which would correct the invalid phase on the first decide.
 * Spin still flags the literal property violation at depth 0.
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
        found     = false;
        best      = 0;
        best_dist = 0;
        d         = 0;
        do
        :: (d == 4) -> break
        :: (d <  4) ->
            if
            :: (IS_VALID(d) && (!IS_VALID(phase) || d != phase)) ->
                dist = (d + 4 - cycle_index) % 4;
                if
                :: (!found) ->
                    found = true;
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

        if
        :: (!IS_VALID(phase)) ->
            switch_phase_to(best)
        :: (IS_VALID(phase) && timer >= MAX_GREEN) ->
            switch_phase_to(best)
        :: (IS_VALID(phase) && timer <  MAX_GREEN) ->
            timer = timer + 1
        fi
    }
    od
}


init { skip }


/* ============================================================
 * LTL PROPERTY
 * ============================================================ */

/* ---- P4: topology validity at I00 ----
 *  With phase initialised to N (this file): errors: 1, depth 0.
 *  signal_check_corner_fixed.pml initialises phase to E:  errors: 0.
 */
ltl p4_topology_valid {
    [] (phase == S || phase == E)
}
