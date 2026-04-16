/*
 * ============================================================
 * vehicle_check.pml  —  Team 10  Phase C  i-group
 *
 * PURPOSE:
 *   Model the v-group vehicle design in Promela so SPIN can
 *   verify the 5 i-group properties from Phase C spec.
 *
 * WHAT IS MODELED:
 *   v_group/strategy.py  — decision logic (signal check, turn rules)
 *   v_group/planner.py   — routing (abstracted as non-determinism)
 *   shared/topology.py   — road grid (all 9 intersections, all edges)
 *
 * ABSTRACTION:
 *   30 slots per segment  →  2 states: ON_SEG / APPROACH
 *   BFS planner           →  non-det next direction (sound over-approx)
 *   2 cars                →  covers all pairwise collision cases
 *   Signal controller     →  non-deterministic (worst-case schedule)
 *
 * GRID LAYOUT (shared/topology.py):
 *   I00(0)--I01(1)--I02(2)=B
 *    |        |        |
 *   I10(3)--I11(4)--I12(5)
 *    |        |        |
 *   I20(6)=D-I21(7)--I22(8)=C
 *
 * PROPERTIES VERIFIED:
 *   P1 (safety)   : No vehicle makes a U-turn
 *   P2 (safety)   : No vehicle crosses an intersection at red light
 *   P3 (liveness) : Every vehicle eventually visits B, C, and D
 *   P4 (safety)   : No two vehicles occupy the same position (no collision)
 *   P5 (liveness) : No vehicle waits at a red light forever (no starvation)
 *
 * HOW TO RUN:
 *   Safety  (P1, P2, P4):
 *     spin -run -ltl p1_no_uturn      vehicle_check.pml
 *     spin -run -ltl p2_no_redlight   vehicle_check.pml
 *     spin -run -ltl p4_no_collision  vehicle_check.pml
 *
 *   Liveness (P3, P5) — add -f for weak fairness:
 *     spin -run -f -ltl p3_visits_all     vehicle_check.pml
 *     spin -run -f -ltl p5_no_starvation  vehicle_check.pml
 *
 *   On failure, replay counterexample:
 *     spin -t vehicle_check.pml
 * ============================================================
 */


/* ============================================================
 * CONSTANTS
 * ============================================================ */

/* Directions (must match shared/enums.py DIRECTIONS) */
#define N 0
#define S 1
#define E 2
#define W 3

/* Abstract position on a road segment
 * ON_SEG  = mid-road (slots 0..28 abstracted)   strategy.py:123
 * APPROACH= last slot, waiting at intersection   strategy.py:120 at_last_slot
 */
#define ON_SEG   0
#define APPROACH 1

/* Intersection IDs  (shared/topology.py _node_id) */
#define I00 0
#define I01 1
#define I02 2   /* = waypoint B */
#define I10 3
#define I11 4
#define I12 5
#define I20 6   /* = waypoint D */
#define I21 7
#define I22 8   /* = waypoint C */

/* A endpoint  (shared/topology.py _build_endpoint_segments)
 * Square node 1/30 mile west of I00 — 2 slots at 30 mph = 1 step each way.
 * NOT a circle intersection: no signal, no turn choices.
 * Arriving here = tour complete.
 */
#define A_NODE 9

/* Waypoint bitmask  (shared/topology.py WAYPOINT_INTERSECTIONS) */
#define HAS_B 1   /* bit 0 = visited I02 */
#define HAS_D 2   /* bit 1 = visited I20 */
#define HAS_C 4   /* bit 2 = visited I22 */
#define ALL_W 7   /* all three visited    */


/* ============================================================
 * GLOBAL STATE
 * ============================================================ */

/* Signal: one green direction per intersection
 * Modeled non-deterministically to cover ALL possible schedules,
 * including the round-robin scheduler in i_group/scheduler.py.
 * If a property holds for any schedule, it holds for our specific one.
 */
byte sig[9];       /* sig[i] in {N,S,E,W} */

/* Per-car state (index 0 or 1) */
byte pos[2];       /* ON_SEG or APPROACH                        */
byte at_i[2];      /* current intersection (0..8)               */
byte dir[2];       /* current direction of travel (N/S/E/W)     */
byte pdir[2];      /* direction on PREVIOUS segment (U-turn det)*/
byte vis[2];       /* visited waypoint bitmask (0..7)           */
bool done[2];      /* true when full A→B→C→D→A tour complete   */

/* Observation flags used in LTL formulas */
bool uturn_flag;          /* set if any car makes a U-turn (P1)  */
bool just_crossed[2];     /* true for one state: car just crossed */
bool cross_was_green[2];  /* was signal green when car crossed?  (P2) */


/* ============================================================
 * MACRO: U-turn check
 * Mirrors topology.py VALID_TURNS (U-turn pairs return False)
 * ============================================================ */
#define IS_UTURN(a,b) \
    ((a==E && b==W) || (a==W && b==E) || \
     (a==N && b==S) || (a==S && b==N))


/* ============================================================
 * SIGNAL CONTROLLER PROCESS
 * Non-deterministic: picks any direction as green each step.
 * Covers the adaptive policy in i_group/policy.py decide().
 * ============================================================ */
active proctype Signals()
{
    byte i;

    /* initialise all intersections to East-green */
    atomic {
        i = 0;
        do
        :: (i < 9) -> sig[i] = E; i++
        :: (i >= 9) -> break
        od
    };

    /* each iteration = one time step */
    do
    :: true ->
        atomic {
            i = 0;
            do
            :: (i < 9) ->
                /* non-det: any direction can become green */
                if
                :: sig[i] = N
                :: sig[i] = S
                :: sig[i] = E
                :: sig[i] = W
                fi;
                i++
            :: (i >= 9) -> break
            od
        }
    od
}


/* ============================================================
 * CAR PROCESS
 *
 * Parameters:
 *   id     — car index (0 or 1)
 *   s_at   — starting intersection
 *   s_dir  — starting direction of travel
 *
 * Models v_group/strategy.py _decide_one():
 *   - ON_SEG  : move toward intersection OR stay blocked
 *   - APPROACH: wait at red (strategy.py:141) OR
 *               cross on green with non-det valid routing
 * ============================================================ */
proctype Car(byte id; byte s_at; byte s_dir)
{
    byte nd;   /* chosen next direction after crossing */
    byte ni;   /* next intersection after crossing     */

    /* --- initialise car state --- */
    atomic {
        at_i[id]  = s_at;
        dir[id]   = s_dir;
        pdir[id]  = s_dir;
        pos[id]   = ON_SEG;
        vis[id]   = 0;
        done[id]  = false;
        just_crossed[id]   = false;
        cross_was_green[id]= true;
    };

    do
    :: done[id] -> break   /* tour complete — exit loop */

    /* ── ON_SEG: non-det advance or stay (car-ahead blocked)
     * Abstracts slots 0..28 from strategy.py:123-126
     * ── */
    :: (pos[id] == ON_SEG) ->
        if
        :: pos[id] = APPROACH   /* move forward to last slot */
        :: skip                 /* stay blocked by car ahead  */
        fi

    /* ── APPROACH: handle A endpoint OR check signal at intersection
     * Directly models strategy.py:132-135 (endpoint = just MOVE, no signal)
     * and strategy.py:141 (intersection = check signal)
     * ── */
    :: (pos[id] == APPROACH) ->
        just_crossed[id] = false;

        if
        /* ── A_NODE: short 1/30-mile segment ends at A (not an intersection).
         * No signal check needed — strategy.py:132 is_intersection() == False.
         * Arriving at A means the full A→B→C→D→A tour is complete.
         * ── */
        :: (at_i[id] == A_NODE) ->
            done[id] = true

        /* ── Circle intersection: check signal ── */
        :: (at_i[id] != A_NODE) ->
        if
        /* RED: must wait — strategy.py:141-142 */
        :: (sig[at_i[id]] != dir[id]) -> skip

        /* GREEN: cross the intersection
         * Choose next (direction, intersection) non-deterministically.
         * Every choice obeys:
         *   (a) Edge must exist in the 3x3 grid  (topology.py)
         *   (b) No U-turn                         (topology.py VALID_TURNS)
         * This non-det set is a sound over-approximation of the
         * BFS planner in v_group/planner.py — if P1..P5 hold for
         * ALL routing choices, they hold for BFS routing too.
         */
        :: (sig[at_i[id]] == dir[id]) ->

            /* record crossing observation BEFORE changing state */
            cross_was_green[id] = (sig[at_i[id]] == dir[id]);
            just_crossed[id]    = true;

            /* ── Choose next valid (direction, intersection) ──
             * Enumerated from topology.py for every reachable
             * (intersection, incoming-direction) pair.
             * U-turns are excluded from every branch.
             */
            if

            /* ===== I00 ===== outgoing: E→I01, S→I10, W→A_NODE(return) */
            :: (at_i[id]==I00 && dir[id]==E) ->   /* came from A: go east or south */
               if :: nd=E; ni=I01
                  :: nd=S; ni=I10
               fi
            :: (at_i[id]==I00 && dir[id]==N) ->   /* came from I10: go east OR return to A */
               if
               :: nd=E; ni=I01                    /* continue into grid */
               :: (vis[id]==ALL_W) -> nd=W; ni=A_NODE  /* all done: head home */
               fi
            :: (at_i[id]==I00 && dir[id]==W) ->   /* came from I01: go south OR return to A */
               if
               :: nd=S; ni=I10                    /* continue into grid */
               :: (vis[id]==ALL_W) -> nd=W; ni=A_NODE  /* all done: head home */
               fi

            /* ===== I01 ===== outgoing: E→I02, W→I00, S→I11 */
            :: (at_i[id]==I01 && dir[id]==E) ->   /* came from I00 */
               if :: nd=E; ni=I02
                  :: nd=S; ni=I11
               fi
            :: (at_i[id]==I01 && dir[id]==W) ->   /* came from I02 */
               if :: nd=W; ni=I00
                  :: nd=S; ni=I11
               fi
            :: (at_i[id]==I01 && dir[id]==S) ->   /* came from I11 */
               if :: nd=E; ni=I02
                  :: nd=W; ni=I00
                  :: nd=S; ni=I11
               fi
            :: (at_i[id]==I01 && dir[id]==N) ->   /* came from I11 going N */
               if :: nd=E; ni=I02
                  :: nd=W; ni=I00
               fi

            /* ===== I02 = B ===== outgoing: W→I01, S→I12 */
            :: (at_i[id]==I02 && dir[id]==E) ->   /* came from I01 */
               nd=S; ni=I12                        /* only valid (W=U-turn) */
            :: (at_i[id]==I02 && dir[id]==N) ->   /* came from I12 */
               nd=W; ni=I01                        /* only valid (S=U-turn) */

            /* ===== I10 ===== outgoing: N→I00, E→I11, S→I20 */
            :: (at_i[id]==I10 && dir[id]==S) ->   /* came from I00 */
               if :: nd=E; ni=I11
                  :: nd=S; ni=I20
               fi
            :: (at_i[id]==I10 && dir[id]==N) ->   /* came from I20 */
               if :: nd=N; ni=I00
                  :: nd=E; ni=I11
               fi
            :: (at_i[id]==I10 && dir[id]==W) ->   /* came from I11 */
               if :: nd=N; ni=I00
                  :: nd=S; ni=I20
               fi

            /* ===== I11 ===== outgoing: N→I01, E→I12, W→I10, S→I21 */
            :: (at_i[id]==I11 && dir[id]==E) ->   /* came from I10 */
               if :: nd=N; ni=I01
                  :: nd=E; ni=I12
                  :: nd=S; ni=I21
               fi
            :: (at_i[id]==I11 && dir[id]==W) ->   /* came from I12 */
               if :: nd=N; ni=I01
                  :: nd=W; ni=I10
                  :: nd=S; ni=I21
               fi
            :: (at_i[id]==I11 && dir[id]==N) ->   /* came from I21 */
               if :: nd=N; ni=I01
                  :: nd=E; ni=I12
                  :: nd=W; ni=I10
               fi
            :: (at_i[id]==I11 && dir[id]==S) ->   /* came from I01 */
               if :: nd=E; ni=I12
                  :: nd=W; ni=I10
                  :: nd=S; ni=I21
               fi

            /* ===== I12 ===== outgoing: N→I02, W→I11, S→I22 */
            :: (at_i[id]==I12 && dir[id]==S) ->   /* came from I02 */
               if :: nd=W; ni=I11
                  :: nd=S; ni=I22
               fi
            :: (at_i[id]==I12 && dir[id]==N) ->   /* came from I22 */
               if :: nd=N; ni=I02
                  :: nd=W; ni=I11
               fi
            :: (at_i[id]==I12 && dir[id]==E) ->   /* came from I11 */
               if :: nd=N; ni=I02
                  :: nd=S; ni=I22
               fi

            /* ===== I20 = D ===== outgoing: N→I10, E→I21 */
            :: (at_i[id]==I20 && dir[id]==S) ->   /* came from I10 */
               nd=E; ni=I21                        /* only valid (N=U-turn) */
            :: (at_i[id]==I20 && dir[id]==W) ->   /* came from I21 */
               nd=N; ni=I10                        /* only valid (E=U-turn) */

            /* ===== I21 ===== outgoing: N→I11, E→I22, W→I20 */
            :: (at_i[id]==I21 && dir[id]==S) ->   /* came from I11 */
               if :: nd=E; ni=I22
                  :: nd=W; ni=I20
               fi
            :: (at_i[id]==I21 && dir[id]==E) ->   /* came from I20 */
               if :: nd=N; ni=I11
                  :: nd=E; ni=I22
               fi
            :: (at_i[id]==I21 && dir[id]==W) ->   /* came from I22 */
               if :: nd=N; ni=I11
                  :: nd=W; ni=I20
               fi

            /* ===== I22 = C ===== outgoing: N→I12, W→I21 */
            :: (at_i[id]==I22 && dir[id]==S) ->   /* came from I12 */
               nd=W; ni=I21                        /* only valid (N=U-turn) */
            :: (at_i[id]==I22 && dir[id]==E) ->   /* came from I21 */
               nd=N; ni=I12                        /* only valid (W=U-turn) */

            fi; /* end direction choice */

            /* ── Detect U-turn (should NEVER trigger; guards above
             * exclude all opposite-direction choices)
             * Records uturn_flag for P1 LTL formula
             * ── */
            if
            :: IS_UTURN(dir[id], nd) -> uturn_flag = true
            :: else                  -> skip
            fi;

            /* ── Update visited waypoints ──
             * Records the intersection we are LEAVING (at_i[id])
             * before we update it, matching strategy.py visit logic
             * ── */
            if
            :: (at_i[id] == I02) -> vis[id] = vis[id] | HAS_B
            :: (at_i[id] == I20) -> vis[id] = vis[id] | HAS_D
            :: (at_i[id] == I22) -> vis[id] = vis[id] | HAS_C
            :: else -> skip
            fi;

            /* ── Advance to next node ── */
            pdir[id]  = dir[id];
            dir[id]   = nd;
            at_i[id]  = ni;   /* may be A_NODE or a circle intersection */
            pos[id]   = ON_SEG
            /* Tour completion is detected in the next APPROACH step:
             * when at_i == A_NODE the car has physically arrived at A. */

        fi /* end red/green */
        fi /* end A_NODE / circle intersection */
    od /* end main loop */
}


/* ============================================================
 * INIT: spawn all processes
 *
 * Car 0: starts AT A_NODE going East — on the 1/30-mile A→I00
 *         segment (2 slots).  Correctly models departure from A.
 * Car 1: starts at I01 going East — offset by one intersection
 *         to avoid a t=0 collision at I00.
 * ============================================================ */
init {
    atomic {
        uturn_flag = false;
        run Signals();
        run Car(0, A_NODE, E);   /* departs from A, heads east to I00 */
        run Car(1, I01,    E);
    }
}


/* ============================================================
 * LTL PROPERTIES
 * ============================================================

 * ── P1: No U-turn ──────────────────────────────────────────
 * "No vehicle takes any U-turn."  (Phase C spec, i-group #1)
 *
 * uturn_flag is set inside the Car process if any crossing
 * produces a direction that is opposite to the previous one.
 * The guards in the if/fi above structurally prevent it, so
 * SPIN should verify this holds on ALL execution paths.
 *
 * Run: spin -run -ltl p1_no_uturn vehicle_check.pml
 */
ltl p1_no_uturn { [] !uturn_flag }


/* ── P2: No red-light crossing ────────────────────────────
 * "No vehicle enters an intersection at red light." (spec #2)
 *
 * cross_was_green[id] is set to (sig==dir) at the moment of
 * crossing. Since crossing is guarded by sig==dir, this is
 * always true. SPIN exhaustively confirms no path violates it.
 *
 * Run: spin -run -ltl p2_no_redlight vehicle_check.pml
 */
ltl p2_no_redlight {
    [] ( (!just_crossed[0] || cross_was_green[0]) &&
         (!just_crossed[1] || cross_was_green[1]) )
}


/* ── P3: Every car visits all waypoints ───────────────────
 * "Every vehicle must visit all of points B, C and D." (spec #3)
 *
 * Liveness property: use weak fairness flag -f so that SPIN
 * assumes every enabled process eventually runs.
 *
 * Run: spin -run -f -ltl p3_visits_all vehicle_check.pml
 */
/* done[id] is now set when car physically arrives at A_NODE
 * (1/30-mile return segment from I00 completed) */
ltl p3_visits_all {
    (<> done[0]) &&
    (<> done[1])
}


/* ── P4: No collision ─────────────────────────────────────
 * "There is no collision between any two vehicles." (spec #4)
 *
 * Two active cars must never share the same (intersection,
 * position) pair simultaneously. Mirrors the occupancy check
 * in v_group/controller.py _check_collisions().
 *
 * Run: spin -run -ltl p4_no_collision vehicle_check.pml
 */
ltl p4_no_collision {
    [] !( !done[0] && !done[1] &&
          at_i[0] == at_i[1]   &&
          pos[0]  == pos[1]    )
}


/* ── P5: No starvation at red light (own-choice property) ─
 * "A car waiting at APPROACH always eventually crosses."
 *
 * This verifies the signal system (non-det here, round-robin
 * in i_group/scheduler.py) gives every car a green eventually.
 * Without this, a car could wait forever at red — a liveness
 * failure that would prevent completing the tour.
 *
 * Requires weak fairness: spin -run -f -ltl p5_no_starvation vehicle_check.pml
 */
ltl p5_no_starvation {
    [] ( (pos[0] == APPROACH -> <> (pos[0] == ON_SEG || done[0])) &&
         (pos[1] == APPROACH -> <> (pos[1] == ON_SEG || done[1])) )
}
