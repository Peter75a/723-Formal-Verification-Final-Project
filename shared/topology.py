# shared/topology.py

"""
This module defines the road network topology for a 3x3 grid traffic system.

Grid layout:
              B(=I02)
    A ──●──●──●
       |  |  |
       ●──●──●
       |  |  |
    D(=I20)──●──●──● C(=I22)

Intersections are labeled I{row}{col}, row 0=top, col 0=left:
    I00 ── I01 ── I02 (= B)
     |      |      |
    I10 ── I11 ── I12
     |      |      |
    I20 ── I21 ── I22
  (= D)          (= C)

Endpoints / Waypoints:
    A  = square node to the WEST of I00; connected by a short 2-slot segment.
    B  = waypoint AT intersection I02  (top-right;  no extra segment)
    C  = waypoint AT intersection I22  (bottom-right; no extra segment)
    D  = waypoint AT intersection I20  (bottom-left;  no extra segment)

Each vehicle departs from A, visits B / C / D in any order, and returns to A.

Segment naming convention:
    "{from_node}_{to_node}"
    e.g. "I00_I01" = segment going East from I00 to I01
         "I01_I00" = segment going West from I01 to I00

Normal segments (inter-intersection): 30 slots (0.5 mile @ 30 mph = 60 s = 30 × 2 s).
A ↔ I00 endpoint segment  :  2 slots (1/30 mile @ 30 mph = 4 s = 2 × 2 s).
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SLOTS_PER_SEGMENT = 30       # slots for a normal inter-intersection segment
ENDPOINT_SLOTS    = 2        # slots for the A ↔ I00 short segment (1/30 mile)

# B, C, D are co-located with intersections — no separate segments needed.
# Use this mapping to translate waypoint names to intersection IDs.
WAYPOINT_INTERSECTIONS: Dict[str, str] = {
    "B": "I02",   # top-right intersection
    "C": "I22",   # bottom-right intersection
    "D": "I20",   # bottom-left intersection
}

# Valid turn rules: (incoming_direction, outgoing_direction) -> allowed?
# Only U-turns are forbidden. Right turns are allowed (but NOT at red light).
# Incoming direction = direction the car was travelling TO reach intersection.
# Outgoing direction = direction the car will travel FROM the intersection.
#
# If a car travels EAST to reach an intersection:
#   - EAST (straight)   ✅
#   - NORTH (turn left) ✅
#   - SOUTH (turn right)✅  allowed (but forbidden at red light)
#   - WEST (U-turn)     ❌  forbidden by spec at all times
#
# Note: red-light enforcement is handled by the signal controller, not here.
VALID_TURNS: Dict[Tuple[str, str], bool] = {
    # Incoming EAST
    ("E", "E"): True,   # straight
    ("E", "N"): True,   # left turn
    ("E", "S"): True,   # right turn — allowed (blocked at red light)
    ("E", "W"): False,  # U-turn — forbidden
    # Incoming WEST
    ("W", "W"): True,   # straight
    ("W", "S"): True,   # left turn
    ("W", "N"): True,   # right turn — allowed (blocked at red light)
    ("W", "E"): False,  # U-turn — forbidden
    # Incoming NORTH
    ("N", "N"): True,   # straight
    ("N", "W"): True,   # left turn
    ("N", "E"): True,   # right turn — allowed (blocked at red light)
    ("N", "S"): False,  # U-turn — forbidden
    # Incoming SOUTH
    ("S", "S"): True,   # straight
    ("S", "E"): True,   # left turn
    ("S", "W"): True,   # right turn — allowed (blocked at red light)
    ("S", "N"): False,  # U-turn — forbidden
}


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class Segment:
    """
    Represents a directed road segment between two nodes.

    Attributes:
        segment_id (str): Unique identifier, e.g. "I00_I01"
        start (str): Start node ID
        end (str): End node ID
        direction (str): Cardinal direction of travel (N/S/E/W)
        length (int): Number of slots (always 30)
    """
    segment_id: str
    start: str
    end: str
    direction: str          # direction a car travels along this segment
    length: int = SLOTS_PER_SEGMENT


@dataclass
class Intersection:
    """
    Represents a traffic intersection (circle node).

    Attributes:
        intersection_id (str): Unique identifier, e.g. "I00"
        row (int): Grid row (0 = top)
        col (int): Grid column (0 = left)
        incoming (List[str]): Segment IDs whose end is this intersection
        outgoing (List[str]): Segment IDs whose start is this intersection
    """
    intersection_id: str
    row: int
    col: int
    incoming: List[str] = field(default_factory=list)
    outgoing: List[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Topology class
# ---------------------------------------------------------------------------

class Topology:
    """
    Encapsulates the complete 3x3 road network.

    Usage:
        topo = Topology()
        topo.build()
        seg = topo.get_segment("I00", "I01")
        valid = topo.is_valid_turn("E", "N")
    """

    def __init__(self):
        self.segments: Dict[str, Segment] = {}
        self.intersections: Dict[str, Intersection] = {}
        # Maps (from_node, to_node) -> segment_id for quick lookup
        self._segment_lookup: Dict[Tuple[str, str], str] = {}

    # -----------------------------------------------------------------------
    # Build
    # -----------------------------------------------------------------------

    def build(self):
        """Build the complete 3x3 grid topology."""
        self._build_intersections()
        self._build_horizontal_segments()
        self._build_vertical_segments()
        self._build_endpoint_segments()

    def _node_id(self, row: int, col: int) -> str:
        return f"I{row}{col}"

    def _add_segment(self, from_node: str, to_node: str, direction: str,
                     length: int = SLOTS_PER_SEGMENT):
        seg_id = f"{from_node}_{to_node}"
        seg = Segment(
            segment_id=seg_id,
            start=from_node,
            end=to_node,
            direction=direction,
            length=length,
        )
        self.segments[seg_id] = seg
        self._segment_lookup[(from_node, to_node)] = seg_id

        # Register with intersections (only circle nodes, not square endpoints)
        if from_node in self.intersections:
            self.intersections[from_node].outgoing.append(seg_id)
        if to_node in self.intersections:
            self.intersections[to_node].incoming.append(seg_id)

    def _build_intersections(self):
        """Create the 9 circle intersections."""
        for row in range(3):
            for col in range(3):
                node_id = self._node_id(row, col)
                self.intersections[node_id] = Intersection(
                    intersection_id=node_id,
                    row=row,
                    col=col,
                )

    def _build_horizontal_segments(self):
        """
        Create East/West segments along each row.
        Each adjacent pair of intersections gets two directed segments.
        """
        for row in range(3):
            for col in range(2):
                left = self._node_id(row, col)
                right = self._node_id(row, col + 1)
                self._add_segment(left, right, "E")   # going East
                self._add_segment(right, left, "W")   # going West

    def _build_vertical_segments(self):
        """
        Create North/South segments along each column.
        Note: row 0 is the TOP, so going from row 0 → row 1 is SOUTH.
        """
        for col in range(3):
            for row in range(2):
                top = self._node_id(row, col)
                bottom = self._node_id(row + 1, col)
                self._add_segment(top, bottom, "S")   # going South
                self._add_segment(bottom, top, "N")   # going North

    def _build_endpoint_segments(self):
        """
        Create the short 2-slot segments connecting A to its adjacent intersection I00.

        A is to the WEST of I00.  Distance = 1/30 mile → 2 slots (4 s at 30 mph).

        B, C, D are located directly AT intersections I02, I22, I20 respectively,
        so no additional segments are needed for them.
        """
        self._add_segment("A", "I00", "E", length=ENDPOINT_SLOTS)
        self._add_segment("I00", "A", "W", length=ENDPOINT_SLOTS)

    # -----------------------------------------------------------------------
    # Query helpers
    # -----------------------------------------------------------------------

    def get_segment(self, from_node: str, to_node: str) -> Optional[Segment]:
        """
        Return the segment going from from_node to to_node, or None.

        Args:
            from_node: Start node ID (e.g. "I00" or "A")
            to_node:   End node ID   (e.g. "I01" or "D")

        Returns:
            Segment or None if no direct segment exists.
        """
        seg_id = self._segment_lookup.get((from_node, to_node))
        return self.segments.get(seg_id) if seg_id else None

    def get_segment_by_id(self, segment_id: str) -> Optional[Segment]:
        """Return a segment by its ID string."""
        return self.segments.get(segment_id)

    def get_intersection(self, intersection_id: str) -> Optional[Intersection]:
        """Return an intersection by its ID, or None."""
        return self.intersections.get(intersection_id)

    def get_neighbors(self, intersection_id: str) -> List[str]:
        """
        Return all intersection IDs directly reachable from the given one.

        Args:
            intersection_id: e.g. "I11"

        Returns:
            List of reachable node IDs (may include endpoint squares).
        """
        inter = self.intersections.get(intersection_id)
        if not inter:
            return []
        return [self.segments[seg_id].end for seg_id in inter.outgoing]

    def get_outgoing_segments(self, intersection_id: str) -> List[Segment]:
        """Return all outgoing Segment objects from a given intersection."""
        inter = self.intersections.get(intersection_id)
        if not inter:
            return []
        return [self.segments[seg_id] for seg_id in inter.outgoing]

    def get_incoming_segments(self, intersection_id: str) -> List[Segment]:
        """Return all incoming Segment objects to a given intersection."""
        inter = self.intersections.get(intersection_id)
        if not inter:
            return []
        return [self.segments[seg_id] for seg_id in inter.incoming]

    def is_valid_turn(self, incoming_direction: str, outgoing_direction: str) -> bool:
        """
        Check whether a turn from incoming_direction to outgoing_direction
        is allowed by the spec (no U-turns; right turns allowed except at red light).

        Args:
            incoming_direction: Direction car was travelling to reach intersection
                                (N/S/E/W)
            outgoing_direction: Direction car wants to travel leaving intersection
                                (N/S/E/W)

        Returns:
            True if the turn is allowed, False otherwise.
        """
        return VALID_TURNS.get((incoming_direction, outgoing_direction), False)

    def get_segment_direction(self, segment_id: str) -> Optional[str]:
        """Return the travel direction of a segment (N/S/E/W)."""
        seg = self.segments.get(segment_id)
        return seg.direction if seg else None

    def is_endpoint(self, node_id: str) -> bool:
        """Return True if node_id is the A square endpoint (the only off-grid node)."""
        return node_id == "A"

    def is_waypoint_intersection(self, intersection_id: str) -> bool:
        """Return True if this intersection is a B/C/D waypoint."""
        return intersection_id in WAYPOINT_INTERSECTIONS.values()

    def waypoint_name(self, intersection_id: str) -> Optional[str]:
        """
        Return the waypoint name (B/C/D) for an intersection, or None.

        Example: waypoint_name("I02") -> "B"
        """
        for name, iid in WAYPOINT_INTERSECTIONS.items():
            if iid == intersection_id:
                return name
        return None

    def is_intersection(self, node_id: str) -> bool:
        """Return True if node_id is a circle intersection."""
        return node_id in self.intersections

    def all_intersection_ids(self) -> List[str]:
        """Return a sorted list of all intersection IDs."""
        return sorted(self.intersections.keys())

    def all_segment_ids(self) -> List[str]:
        """Return a sorted list of all segment IDs."""
        return sorted(self.segments.keys())

    # -----------------------------------------------------------------------
    # Debug / display
    # -----------------------------------------------------------------------

    def summary(self) -> str:
        """Return a human-readable summary of the topology."""
        lines = [
            f"Topology Summary",
            f"  Intersections : {len(self.intersections)}",
            f"  Segments      : {len(self.segments)}",
            "",
            "Intersections:",
        ]
        for iid in self.all_intersection_ids():
            inter = self.intersections[iid]
            lines.append(
                f"  {iid} (row={inter.row}, col={inter.col}) | "
                f"in={inter.incoming} | out={inter.outgoing}"
            )
        lines.append("")
        lines.append("Segments:")
        for sid in self.all_segment_ids():
            seg = self.segments[sid]
            lines.append(
                f"  {sid:20s} {seg.start} → {seg.end}  dir={seg.direction}  slots={seg.length}"
            )
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Quick self-test (run: python topology.py)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    topo = Topology()
    topo.build()
    print(topo.summary())

    print("\n--- Spot checks ---")
    # Segment lookup
    seg = topo.get_segment("I00", "I01")
    print(f"I00 → I01 : {seg}")

    seg = topo.get_segment("I01", "I00")
    print(f"I01 → I00 : {seg}")

    seg = topo.get_segment("A", "I00")
    print(f"A → I00   : {seg}")

    # Neighbors
    print(f"\nNeighbors of I11 : {topo.get_neighbors('I11')}")
    print(f"Neighbors of I00 : {topo.get_neighbors('I00')}")

    # Turn validity
    print(f"\nTurn checks (incoming → outgoing):")
    print(f"  E → E (straight) : {topo.is_valid_turn('E', 'E')}")   # True
    print(f"  E → N (left)     : {topo.is_valid_turn('E', 'N')}")   # True
    print(f"  E → S (right)    : {topo.is_valid_turn('E', 'S')}")   # True
    print(f"  E → W (U-turn)   : {topo.is_valid_turn('E', 'W')}")   # False