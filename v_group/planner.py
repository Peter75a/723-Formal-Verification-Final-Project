# v_group/planner.py

"""
Route planning module for the v-group vehicle controller.

Implements BFS-based shortest-path planning on the intersection graph.
Supports congestion-aware re-routing: intersections with many stopped cars
receive a higher traversal cost, nudging vehicles toward less-congested paths.

Key classes:
    RoutePlanner   — stateless planner; call plan_route() each step for fresh routes
    RouteCache     — optional per-car route cache to avoid replanning every step
"""

from collections import deque
from typing import Dict, List, Optional, Tuple

from shared.topology import Topology, WAYPOINT_INTERSECTIONS
from shared.state import GlobalState, CarState


# An intersection whose stopped-car count meets this threshold is "congested".
CONGESTION_THRESHOLD = 3
# Extra BFS hops to add for a congested intersection (simulates higher cost).
CONGESTION_PENALTY = 2


class RoutePlanner:
    """
    BFS route planner over the intersection graph.

    The planner answers two questions:
      1. What is the next waypoint this car should head to?
      2. What is the next intersection the car should move toward?

    Usage:
        planner = RoutePlanner(topology)
        next_inter = planner.next_intersection(car, state)
    """

    def __init__(self, topology: Topology):
        self.topology = topology
        # Reverse map: intersection_id -> waypoint name (B/C/D), if applicable
        self._inter_to_waypoint: Dict[str, str] = {
            v: k for k, v in WAYPOINT_INTERSECTIONS.items()
        }

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def choose_next_destination(self, car: CarState,
                                state: GlobalState) -> Optional[str]:
        """
        Choose the best next waypoint for the car to visit.

        Strategy:
          - From the car's current intersection, BFS to find the closest
            unvisited waypoint (B/C/D) or A (if all visited).
          - "Closest" = fewest hops in the unweighted graph.

        Args:
            car:   The car whose destination is being chosen.
            state: Current global state (used for congestion info).

        Returns:
            Waypoint name ("A", "B", "C", or "D"), or None if already done.
        """
        remaining = [w for w in ["B", "C", "D"]
                     if w not in car.destinations_visited]

        if not remaining:
            # All waypoints visited — return to A
            return "A"

        # Find the current intersection (or nearest one ahead)
        current_inter = self._current_intersection_id(car)
        if current_inter is None:
            return car.current_destination  # keep existing destination

        # BFS distances to all reachable intersections
        distances = self._bfs_distances(current_inter, state)

        # Pick the closest unvisited waypoint
        best_dest = None
        best_dist = float("inf")
        for waypoint in remaining:
            inter_id = WAYPOINT_INTERSECTIONS[waypoint]
            dist = distances.get(inter_id, float("inf"))
            if dist < best_dist:
                best_dist = dist
                best_dest = waypoint

        return best_dest

    def next_intersection(self, car: CarState,
                          destination_intersection: str,
                          state: GlobalState) -> Optional[str]:
        """
        Return the next intersection the car should move toward on its way
        to `destination_intersection`.

        Uses BFS on the intersection graph with congestion penalties.

        Args:
            car:                      The moving car.
            destination_intersection: Target intersection ID (e.g. "I02" for B).
            state:                    Current global state.

        Returns:
            Next intersection ID to head toward, or None if already there /
            no path found.
        """
        current_inter = self._current_intersection_id(car)
        if current_inter is None:
            return None
        if current_inter == destination_intersection:
            return destination_intersection  # already at destination

        path = self._bfs_path(current_inter, destination_intersection, state)
        if path is None or len(path) < 2:
            return None

        return path[1]  # first step toward destination

    def plan_full_route(self, car: CarState,
                        state: GlobalState) -> List[str]:
        """
        Plan the complete remaining route for the car as a list of intersection IDs.

        Visits remaining waypoints in nearest-first order, ending at A.

        Args:
            car:   The car to plan for.
            state: Current global state.

        Returns:
            Ordered list of intersection IDs to visit (including final "I00"
            for the A-return leg).
        """
        remaining = [w for w in ["B", "C", "D"]
                     if w not in car.destinations_visited]

        route: List[str] = []
        current_inter = self._current_intersection_id(car)
        if current_inter is None:
            return route

        # Greedy nearest-neighbor ordering of remaining waypoints
        visited_order: List[str] = []
        unvisited = list(remaining)
        pos = current_inter

        while unvisited:
            distances = self._bfs_distances(pos, state)
            best = min(
                unvisited,
                key=lambda w: distances.get(WAYPOINT_INTERSECTIONS[w], float("inf"))
            )
            visited_order.append(best)
            unvisited.remove(best)
            pos = WAYPOINT_INTERSECTIONS[best]

        # Build segment-by-segment path
        pos = current_inter
        for waypoint in visited_order:
            dest_inter = WAYPOINT_INTERSECTIONS[waypoint]
            path = self._bfs_path(pos, dest_inter, state)
            if path:
                route.extend(path[1:])  # skip current position (already included)
            pos = dest_inter

        # Return to A via I00
        path_home = self._bfs_path(pos, "I00", state)
        if path_home:
            route.extend(path_home[1:])

        return route

    def destination_intersection(self, destination: str) -> Optional[str]:
        """
        Convert a waypoint name or node name to an intersection ID.

        Args:
            destination: "A", "B", "C", or "D"

        Returns:
            Intersection ID string, or None for endpoint A (handled separately).
        """
        if destination == "A":
            return "I00"   # car must reach I00 then take A_I00 segment backward
        return WAYPOINT_INTERSECTIONS.get(destination)

    # ------------------------------------------------------------------
    # BFS helpers
    # ------------------------------------------------------------------

    def _bfs_distances(self, start: str,
                       state: GlobalState) -> Dict[str, int]:
        """
        BFS from `start` intersection, returning distance (hops) to every
        reachable intersection. Congested intersections count as extra hops.

        Args:
            start: Starting intersection ID.
            state: Current global state for congestion data.

        Returns:
            Dict mapping intersection_id -> hop count.
        """
        distances: Dict[str, int] = {start: 0}
        queue: deque = deque([start])

        while queue:
            node = queue.popleft()
            inter = self.topology.get_intersection(node)
            if inter is None:
                continue

            base_cost = distances[node]
            for seg_id in inter.outgoing:
                seg = self.topology.get_segment_by_id(seg_id)
                if seg is None:
                    continue
                neighbor = seg.end
                if not self.topology.is_intersection(neighbor):
                    continue   # skip endpoint A

                # Apply congestion penalty
                congestion = state.congestion.get(neighbor, 0)
                extra = CONGESTION_PENALTY if congestion >= CONGESTION_THRESHOLD else 0
                cost = base_cost + 1 + extra

                if neighbor not in distances or distances[neighbor] > cost:
                    distances[neighbor] = cost
                    queue.append(neighbor)

        return distances

    def _bfs_path(self, start: str, goal: str,
                  state: GlobalState) -> Optional[List[str]]:
        """
        Dijkstra-style shortest path from `start` to `goal` with the same
        congestion penalties used by _bfs_distances, so routing and
        destination selection use a consistent cost model.

        Args:
            start: Source intersection ID.
            goal:  Target intersection ID.
            state: Current global state.

        Returns:
            List of intersection IDs from start to goal (inclusive), or None.
        """
        import heapq

        if start == goal:
            return [start]

        # (cost, node)
        heap = [(0, start)]
        dist: Dict[str, int] = {start: 0}
        parent: Dict[str, Optional[str]] = {start: None}

        while heap:
            cost, node = heapq.heappop(heap)
            if node == goal:
                return self._reconstruct_path(parent, goal)
            if cost > dist.get(node, float("inf")):
                continue

            inter = self.topology.get_intersection(node)
            if inter is None:
                continue

            for seg_id in inter.outgoing:
                seg = self.topology.get_segment_by_id(seg_id)
                if seg is None:
                    continue
                neighbor = seg.end
                if not self.topology.is_intersection(neighbor):
                    continue

                congestion = state.congestion.get(neighbor, 0)
                extra = CONGESTION_PENALTY if congestion >= CONGESTION_THRESHOLD else 0
                new_cost = cost + 1 + extra

                if new_cost < dist.get(neighbor, float("inf")):
                    dist[neighbor] = new_cost
                    parent[neighbor] = node
                    heapq.heappush(heap, (new_cost, neighbor))

        return None  # no path found

    def _reconstruct_path(self, parent: Dict[str, Optional[str]],
                          goal: str) -> List[str]:
        """Reconstruct the path from BFS parent pointers."""
        path: List[str] = []
        node: Optional[str] = goal
        while node is not None:
            path.append(node)
            node = parent[node]
        path.reverse()
        return path

    def _current_intersection_id(self, car: CarState) -> Optional[str]:
        """
        Return the intersection the car is moving TOWARD (end of current segment),
        if it is a real circle intersection. Otherwise return the start node
        of the segment if it is an intersection.

        This is used as the "current position" in BFS.
        """
        seg = self.topology.get_segment_by_id(car.segment_id)
        if seg is None:
            return None

        # If segment ends at a circle intersection, that's where we're heading
        if self.topology.is_intersection(seg.end):
            return seg.end

        # Segment ends at A (e.g. A_I00 reversed) — treat start as current
        if self.topology.is_intersection(seg.start):
            return seg.start

        return None


# ------------------------------------------------------------------
# Per-car route cache
# ------------------------------------------------------------------

class RouteCache:
    """
    Lightweight per-car route cache to avoid replanning every step.

    The cache is invalidated when the car reaches a new intersection
    (i.e., the head of the cached route is consumed).

    Usage:
        cache = RouteCache()
        route = cache.get_or_plan(car, planner, state)
        cache.advance(car.car_id)   # call after car crosses an intersection
    """

    def __init__(self):
        # car_id -> list of remaining intersection IDs in plan
        self._routes: Dict[int, List[str]] = {}

    def get_or_plan(self, car: CarState,
                    planner: RoutePlanner,
                    state: GlobalState) -> List[str]:
        """
        Return the cached route for the car, or generate a fresh one.
        """
        if car.car_id not in self._routes or not self._routes[car.car_id]:
            self._routes[car.car_id] = planner.plan_full_route(car, state)
        return self._routes[car.car_id]

    def advance(self, car_id: int) -> None:
        """Remove the first step of the route (car just crossed an intersection)."""
        if car_id in self._routes and self._routes[car_id]:
            self._routes[car_id].pop(0)

    def invalidate(self, car_id: int) -> None:
        """Force replanning for a specific car (e.g., after congestion reroute)."""
        self._routes.pop(car_id, None)

    def invalidate_all(self) -> None:
        """Clear all cached routes."""
        self._routes.clear()