/*
 * ============================================================
 * signal_check_corner_fixed.pml -- Team 10 Phase C v-group  (rev 2)
 *
 * PURPOSE:
 *   Same model as signal_check_corner.pml but with the proposed
 *   one-line fix to scheduler.initialize() applied: corner
 *   intersection I00 starts in a topology-valid phase (E)
 *   rather than the global default "N".
 *
 *   Spin reports errors: 0.
 *
 *   Proposed Python patch (one-line change in
 *   i_group/scheduler.py:initialize):
 *
 *     - def initialize(self, intersection_ids, default_phase="N"):
 *     -     for iid in intersection_ids:
 *     -         self._phase[iid] = default_phase
 *     + def initialize(self, intersection_ids, valid_phases_for):
 *     +     for iid in intersection_ids:
 *     +         self._phase[iid] = next(iter(valid_phases_for(iid)))
 *
 * REV 2 NOTE:
 *   Globals are initialised inline so the very first state has
 *   phase = E and green_E = true.  Without inline initialisation
 *   Spin would see phase = 0 (N) at depth 0 and falsely report
 *   the property as violated -- the language default for `byte`
 *   is 0, regardless of any later atomic block.
 *
 * HOW TO RUN:
 *     spin -a -ltl p4_topology_valid signal_check_corner_fixed.pml
 *     gcc -O2 -o pan pan.c
 *     ./pan
 *
 *   Expected: errors: 0.
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

#define IS_VALID(d) ((d == S) || (d == E))


/* ============================================================
 * GLOBAL STATE  (proposed FIX: start I00 at a topology-valid phase)
 * ============================================================ */
byte phase       = E;     /* THE FIX: valid at I00 from depth 0 */
byte timer       = 0;
byte cycle_index = 0;

bool green_N = false;
bool green_S = false;
bool green_E = true;      /* matches phase = E */
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
 * CONTROLLER PROCESS  (I00 -- valid = {S, E}, FIXED init)
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
ltl p4_topology_valid {
    [] (phase == S || phase == E)
}
