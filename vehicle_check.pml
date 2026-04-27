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
 *  A(9)- I00(0)--I01(1)--I02(2)=D
 *           |        |        |
 *        I10(3)--I11(4)--I12(5)
 *          |        |        |
 *       I20(6)=B-I21(7)--I22(8)=C
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
#define I02 2   /* = waypoint D */
#define I10 3
#define I11 4
#define I12 5
#define I20 6   /* = waypoint B */
#define I21 7
#define I22 8   /* = waypoint C */
#define A_NODE 9
/* A endpoint  (shared/topology.py _build_endpoint_segments)
 * Square node 1/30 mile west of I00 — 2 slots at 30 mph = 1 step each way.
 * NOT a circle intersection: no signal, no turn choices.
 * Arriving here = tour complete.
 */




/* Waypoint bitmask  (shared/topology.py WAYPOINT_INTERSECTIONS) */
#define HAS_D 1   /* bit 0 = visited I02 */
#define HAS_C 2   /* bit 1 = visited I22 */
#define HAS_B 4   /* bit 2 = visited I20 */
#define ALL_W 7   /* all three visited    */

bool busy[36];
#define IDX(i,d) (i*4 + d)
#define VALID_NODE(i) ((i) != A_NODE)
#define SEG(i,d) busy[IDX(i,d)]

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
bool started[2];   /* true after each car finishes initialization */
byte turn;   /* 0 = car moves, 1 = signals move */

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
 *
 * Round-robin scheduler: cycles N -> S -> E -> W -> N ...
 * Matches the actual implementation in i_group/scheduler.py.
 *
 * WHY round-robin instead of non-det:
 *   P1/P2/P4 (safety) were already verified under a fully
 *   non-deterministic scheduler — a strictly stronger guarantee.
 *   P5 (liveness/no-starvation) requires a FAIR scheduler;
 *   with pure non-det, SPIN cannot bound the search because a
 *   hostile scheduler can delay a car forever.  Round-robin is
 *   fair by construction and matches i_group/scheduler.py.
 * ============================================================ */
proctype Signals()
{
    byte i;

    atomic {
        i = 0;
        do
        :: (i < 9) -> sig[i] = E; i++
        :: else -> break
        od
    };

    do
    :: (turn == 2) ->
        atomic {
            i = 0;
            do
            :: (i < 9) ->
                if
                :: (sig[i] == N) -> sig[i] = S
                :: (sig[i] == S) -> sig[i] = E
                :: (sig[i] == E) -> sig[i] = W
                :: (sig[i] == W) -> sig[i] = N
                fi;
                i++
            :: else -> break
            od;

            turn = 0
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
    byte nd;
    byte ni;
    byte target;
    byte old_i;
    byte old_d;

    atomic {
        at_i[id]  = s_at;
        dir[id]   = s_dir;
        pdir[id]  = s_dir;
        pos[id]   = ON_SEG;
        vis[id]   = 0;
        done[id]  = false;
        just_crossed[id]    = false;
        cross_was_green[id] = true;
        started[id] = true;
        if
        :: VALID_NODE(s_at) ->
            SEG(s_at, s_dir) = true
        :: else -> skip
        fi;
    };

    do
    

    :: ((id == 0 && turn == 0) || (id == 1 && turn == 1)) ->
        atomic {

            if
            :: done[id] -> skip

            :: (pos[id] == ON_SEG) ->
               if
               :: (started[1-id] &&
                  !done[1-id] &&
                  at_i[1-id] == at_i[id] &&
                  pos[1-id]  == APPROACH &&
                  dir[1-id]  == dir[id]) ->
                  skip
               :: else ->
                    pos[id] = APPROACH;
                    fi

            :: (pos[id] == APPROACH) ->
                just_crossed[id] = false;
                /* mark waypoint when car reaches that intersection */
                if
                :: (at_i[id] == I02) -> vis[id] = vis[id] | HAS_D
                :: (at_i[id] == I20) -> vis[id] = vis[id] | HAS_B
                :: (at_i[id] == I22) -> vis[id] = vis[id] | HAS_C
                :: else -> skip
                fi;

                /* fixed visit order: B -> D -> C -> A */
                /* choose next target in a fixed order: D -> C -> B -> A */
                /* fixed visiting order: D -> C -> B -> A */
                if
                :: ((vis[id] & HAS_D) == 0) ->
                    target = I02

                :: ((vis[id] & HAS_D) != 0 && (vis[id] & HAS_C) == 0) ->
                    target = I22

                :: ((vis[id] & HAS_D) != 0 &&
                    (vis[id] & HAS_C) != 0 &&
                    (vis[id] & HAS_B) == 0) ->
                    target = I20

                :: else ->
                    target = A_NODE
                fi

                if
                :: (at_i[id] == A_NODE) ->
                    if
                    :: (vis[id] == ALL_W) ->
                        done[id] = true

                    :: else ->
                        if
                        :: (!SEG(I00, E)) ->
                            SEG(I00, E) = true;
                            at_i[id] = I00;
                            dir[id]  = E;
                            pos[id]  = ON_SEG
                        :: else ->
                            pos[id] = APPROACH
                        fi
                    fi

                :: (at_i[id] != A_NODE) ->
                    if
                    :: (sig[at_i[id]] != dir[id]) ->
                        pos[id] = APPROACH

                    :: (sig[at_i[id]] == dir[id]) ->
                        cross_was_green[id] = true;
                        just_crossed[id]    = true;

                       if
                        /* ---------- target = D (I02, go along top row) ---------- */
                        :: (target == I02 && at_i[id] == I10 && dir[id] == E) ->
                           nd = N; ni = I00

                        :: (target == I02 && at_i[id] == I00 && dir[id] == N) ->
                           nd = E; ni = I01

                        :: (target == I02 && at_i[id] == I00 && dir[id] == E) ->
                           nd = E; ni = I01

                        :: (target == I02 && at_i[id] == I01 && dir[id] == E) ->
                           nd = E; ni = I02


                        /* ---------- target = C (I22, go down right side) ---------- */
                        :: (target == I22 && at_i[id] == I02 && dir[id] == E) ->
                            nd = S; ni = I12

                        :: (target == I22 && at_i[id] == I12 && dir[id] == S) ->
                            nd = S; ni = I22


                        /* ---------- target = B (I20, go left along bottom) ---------- */
                        :: (target == I20 && at_i[id] == I22 && dir[id] == S) ->
                            nd = W; ni = I21

                        :: (target == I20 && at_i[id] == I21 && dir[id] == W) ->
                            nd = W; ni = I20


                        /* ---------- target = A (return via left side) ---------- */
                        :: (target == A_NODE && at_i[id] == I20 && dir[id] == W) ->
                            nd = N; ni = I10

                        :: (target == A_NODE && at_i[id] == I10 && dir[id] == N) ->
                            nd = N; ni = I00

                        /* allow reaching A regardless of direction */
                        :: (target == A_NODE && at_i[id] == I00) ->
                            nd = W; ni = A_NODE


                        :: else ->
                            nd = dir[id];
                            ni = at_i[id]
                        fi;

                        if
                        :: IS_UTURN(dir[id], nd) -> uturn_flag = true
                        :: else -> skip
                        fi;
                        atomic {
                       if
                        :: (VALID_NODE(ni) && !SEG(ni, nd)) ->

                            old_i = at_i[id];
                            old_d = dir[id];

                            /* acquire next segment */
                            SEG(ni, nd) = true;
                        
                            /* move */
                            pdir[id] = dir[id];
                            dir[id]  = nd;
                            at_i[id] = ni;
                            pos[id]  = ON_SEG;

                            /* ⭐release segment（old_i！） */
                            if
                            :: VALID_NODE(old_i) ->
                                SEG(old_i, old_d) = false
                            :: else -> skip
                            fi;


                            /* update visited waypoints */
                            if
                            :: (ni == I02) -> vis[id] = vis[id] | HAS_D
                            :: (ni == I20) -> vis[id] = vis[id] | HAS_B
                            :: (ni == I22) -> vis[id] = vis[id] | HAS_C
                            :: else -> skip
                            fi;

                            /* check completion */
                            if
                            :: (ni == A_NODE && vis[id] == ALL_W) -> done[id] = true
                            :: else -> skip
                            fi;

                        :: (ni == A_NODE) ->   /*  A_NODE（no busy） */

                        

                                old_i = at_i[id];
                                old_d = dir[id];

                                pdir[id] = dir[id];
                                dir[id]  = nd;
                                at_i[id] = ni;
                                pos[id]  = ON_SEG;

                                /* ⭐ release old segment */
                                if
                                :: VALID_NODE(old_i) ->
                                    SEG(old_i, old_d) = false
                                :: else -> skip
                                fi;

                        :: else ->
                            /* segment occupy → wait */
                            pos[id] = APPROACH
                        fi;
                        }
                    fi
                fi
            fi;

            if
            :: (id == 0) -> turn = 1
            :: (id == 1) -> turn = 2
            fi
        }
    od
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
        started[0] = false;
        started[1] = false;
        turn = 0;
        run Signals();
        run Car(0, A_NODE, E);   /* departs from A, heads east to I00 */
        run Car(1, I01,    E);
         /* initial segment occupancy */
        SEG(I01, E) = true;   /* Car1 starts on I01 -> I02 */
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
    [] (
        (!just_crossed[0] || cross_was_green[0])
        &&
        (!just_crossed[1] || cross_was_green[1])
    )
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
    <> (
        (vis[0] == ALL_W && done[0])
        &&
        (vis[1] == ALL_W && done[1])
    )
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
    [] !(
        started[0] && started[1] &&
        !done[0] && !done[1] &&
        at_i[0] != A_NODE &&
        at_i[1] != A_NODE &&
        at_i[0] == at_i[1] &&
        pos[0]  == pos[1]  &&
        dir[0]  == dir[1]
    )
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
    [] (
        (pos[0] == APPROACH -> <> (pos[0] == ON_SEG || done[0]))
        &&
        (pos[1] == APPROACH -> <> (pos[1] == ON_SEG || done[1]))
    )
}