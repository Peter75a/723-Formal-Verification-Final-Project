# shared/topology.py

"""
This module defines the road network topology.

It includes:
- Road segments (with slots)
- Intersections (connection points)

IMPORTANT:
Topology must be shared and identical across all groups.
"""


from dataclasses import dataclass
from typing import Dict


@dataclass
class Segment:
    """
    Represents a directed road segment.

    Attributes:
        segment_id (str): Unique identifier
        start (str): Start node (intersection or endpoint)
        end (str): End node (intersection or endpoint)
        length (int): Number of slots (default = 30)
    """
    segment_id: str
    start: str
    end: str
    length: int = 30


@dataclass
class Intersection:
    """
    Represents a traffic intersection.

    Attributes:
        intersection_id (str): Unique identifier
        incoming (list): List of incoming segment IDs
        outgoing (list): List of outgoing segment IDs
    """
    intersection_id: str
    incoming: list
    outgoing: list


class Topology:
    """
    Encapsulates the entire road network.
    """

    def __init__(self):
        self.segments: Dict[str, Segment] = {}
        self.intersections: Dict[str, Intersection] = {}

    def build_simple_topology(self):
        """
        Builds a minimal example topology.

        Example:
            A → I1 → B
        """

        # Define segments
        self.segments["A_to_I1"] = Segment("A_to_I1", "A", "I1")
        self.segments["I1_to_B"] = Segment("I1_to_B", "I1", "B")

        # Define intersection
        self.intersections["I1"] = Intersection(
            "I1",
            incoming=["A_to_I1"],
            outgoing=["I1_to_B"]
        )

    def get_intersection_from_segment(self, segment_id: str) -> str:
        """
        Returns the intersection at the end of a segment.

        Args:
            segment_id (str): Segment identifier

        Returns:
            str: Intersection ID
        """
        return self.segments[segment_id].end