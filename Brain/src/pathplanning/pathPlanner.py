# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.
#
# BSD 3-Clause License — see LICENSE file for full text.

"""
PathPlanner — graph-based path planning on the BFMC competition track.

The BFMC organizers provide the track as a directed graph in GraphML format.
Every node has 'x' and 'y' attributes (metres, track-frame coordinates).
Every edge has an optional 'weight' attribute (default = euclidean distance
between endpoints).
"""

import math
import heapq
from typing import Optional

import networkx as nx


class PathPlanner:
    """
    Loads a BFMC GraphML map and exposes shortest-path utilities.

    The graph is a *directed* weighted graph (DiGraph).  Edge weights are
    treated as distances in metres.  If an edge has no 'weight' attribute,
    the euclidean distance between its endpoints is used as a fallback.

    Parameters
    ----------
    graphml_path : str
        Path to the .graphml file provided by BFMC organizers.
    """

    def __init__(self, graphml_path: str):
        self._graph = nx.read_graphml(graphml_path)

        # Pre-cache node (x, y) positions for speed.
        # GraphML stores all attributes as strings — cast to float here once.
        self._pos: dict[str, tuple[float, float]] = {}
        for node_id, data in self._graph.nodes(data=True):
            try:
                x = float(data.get("x", data.get("X", 0.0)))
                y = float(data.get("y", data.get("Y", 0.0)))
                self._pos[node_id] = (x, y)
            except (ValueError, TypeError):
                self._pos[node_id] = (0.0, 0.0)

        # Ensure every edge has a numeric 'weight'.
        for u, v, data in self._graph.edges(data=True):
            if "weight" not in data or data["weight"] is None:
                dist = self._euclidean(self._pos[u], self._pos[v])
                self._graph[u][v]["weight"] = dist
            else:
                try:
                    self._graph[u][v]["weight"] = float(data["weight"])
                except (ValueError, TypeError):
                    self._graph[u][v]["weight"] = self._euclidean(
                        self._pos[u], self._pos[v]
                    )

        node_count = self._graph.number_of_nodes()
        edge_count = self._graph.number_of_edges()
        print(
            "\033[1;97m[ PathPlanner ] :\033[0m \033[1;92mINFO\033[0m"
            f" - Loaded '{graphml_path}' — {node_count} nodes, {edge_count} edges"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Internal helpers
    # ─────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _euclidean(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1])

    # ─────────────────────────────────────────────────────────────────────────
    # Public API
    # ─────────────────────────────────────────────────────────────────────────

    def find_nearest_node(self, x: float, y: float) -> Optional[str]:
        """
        Return the ID of the graph node closest (Euclidean) to (x, y).

        Parameters
        ----------
        x, y : float
            Current car position in metres.

        Returns
        -------
        str or None
            Node ID, or None if the graph is empty.
        """
        if not self._pos:
            return None

        car = (x, y)
        best_id   = None
        best_dist = float("inf")

        for node_id, pos in self._pos.items():
            d = self._euclidean(car, pos)
            if d < best_dist:
                best_dist = d
                best_id   = node_id

        return best_id

    def dijkstra(self, start_id: str, target_id: str) -> list:
        """
        Compute the shortest path from start_id to target_id using Dijkstra.

        Uses networkx's built-in implementation (C-accelerated when available).

        Parameters
        ----------
        start_id  : str  Source node ID.
        target_id : str  Destination node ID.

        Returns
        -------
        list[str]
            Ordered list of node IDs from start to target (inclusive).
            Empty list if no path exists or nodes not in graph.
        """
        if start_id not in self._graph or target_id not in self._graph:
            print(
                f"\033[1;97m[ PathPlanner ] :\033[0m \033[1;93mWARN\033[0m"
                f" - Node not in graph: start={start_id}, target={target_id}"
            )
            return []

        try:
            path = nx.dijkstra_path(
                self._graph, start_id, target_id, weight="weight"
            )
            return path
        except nx.NetworkXNoPath:
            print(
                f"\033[1;97m[ PathPlanner ] :\033[0m \033[1;93mWARN\033[0m"
                f" - No path from {start_id} to {target_id}"
            )
            return []
        except nx.NodeNotFound as e:
            print(
                f"\033[1;97m[ PathPlanner ] :\033[0m \033[1;93mWARN\033[0m"
                f" - NodeNotFound: {e}"
            )
            return []

    def distance_to_node(self, x: float, y: float, node_id: str) -> float:
        """
        Euclidean distance (metres) from position (x, y) to a graph node.

        Parameters
        ----------
        x, y    : float  Car position.
        node_id : str    Target node ID.

        Returns
        -------
        float
            Distance in metres.  Returns inf if node_id is unknown.
        """
        pos = self._pos.get(node_id)
        if pos is None:
            return float("inf")
        return self._euclidean((x, y), pos)

    def heading_to_next(self, path: list, current_idx: int) -> Optional[float]:
        """
        Compute the heading (degrees) from the node at current_idx to the
        next node in the path.

        Convention: 0° = East (+X), angles increase counter-clockwise.
        This matches atan2(dy, dx) in standard mathematical orientation.

        The threadcontroller uses this to bias the Stanley steering when the
        lane-keeping camera has low confidence (e.g. no lane markings visible).

        Parameters
        ----------
        path        : list[str]   Full path returned by dijkstra().
        current_idx : int         Index of the node the car is currently at
                                  (or heading towards).

        Returns
        -------
        float or None
            Heading in degrees, or None if path is too short / index out of range.
        """
        if not path or current_idx + 1 >= len(path):
            return None

        current_id = path[current_idx]
        next_id    = path[current_idx + 1]

        cur_pos  = self._pos.get(current_id)
        next_pos = self._pos.get(next_id)

        if cur_pos is None or next_pos is None:
            return None

        dx = next_pos[0] - cur_pos[0]
        dy = next_pos[1] - cur_pos[1]

        heading_rad = math.atan2(dy, dx)
        heading_deg = math.degrees(heading_rad)
        return heading_deg

    def path_info(self, path: list) -> str:
        """
        Human-readable summary of a path (for debug prints).

        Example output:
            "12 nodes, total 4.73 m: 192 → 248 → 257 → … → 290"
        """
        if not path:
            return "empty path"

        total_dist = 0.0
        for i in range(len(path) - 1):
            a = self._pos.get(path[i])
            b = self._pos.get(path[i + 1])
            if a and b:
                total_dist += self._euclidean(a, b)

        if len(path) > 6:
            nodes_str = " → ".join(path[:3]) + " → … → " + " → ".join(path[-2:])
        else:
            nodes_str = " → ".join(path)

        return f"{len(path)} nodes, total {total_dist:.2f} m: {nodes_str}"
