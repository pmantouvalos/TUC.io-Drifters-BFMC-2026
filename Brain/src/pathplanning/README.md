# Path Planning

The `src/pathplanning` package contains the graph-based route planning and
mission navigation code for the BFMC competition track.

## Contents

```text
src/pathplanning/
|-- pathPlanner.py
|-- Navigator/
|   |-- processNavigator.py
|   `-- threads/
|       `-- threadNavigator.py
`-- README.md
```

### `pathPlanner.py`

Defines `PathPlanner`, a reusable planner around the competition track's
directed GraphML graph. It loads node coordinates, normalizes edge weights,
and provides:

- `find_nearest_node(x, y)`: find the graph node closest to a vehicle pose.
- `dijkstra(start_id, target_id)`: calculate a weighted shortest path.
- `distance_to_node(x, y, node_id)`: calculate Euclidean distance to a node.
- `heading_to_next(path, current_idx)`: calculate the next path heading in
  degrees, where 0 degrees points east and positive angles turn
  counter-clockwise.
- `path_info(path)`: produce a human-readable path summary for diagnostics.

GraphML node attributes `x` and `y` are interpreted as metre coordinates.
Existing edge `weight` attributes are converted to floats. Missing or invalid
weights fall back to the Euclidean distance between the edge endpoints.

### `Navigator/processNavigator.py`

Defines `processNavigator`, a `WorkerProcess` wrapper that creates one
`threadNavigator` instance. Its process-level hooks are currently empty; the
navigation work is performed by the thread.

### `Navigator/threads/threadNavigator.py`

Defines `threadNavigator`, the mission-level navigator. It:

1. Loads the track graph.
2. Receives `LocalizerPose` messages containing `pose: [x, y, theta]` and a
	localization `mode`.
3. Waits while localization is `GLOBAL` or `RECOVER`.
4. When localization is `LOCAL`, finds the closest graph node and routes
	through the configured waypoint sequence.
5. Converts graph-node triples at intersections into `TURN_LEFT`,
	`TURN_RIGHT`, or `GO_STRAIGHT` commands.
6. Sends the maneuver sequence, lane preferences, and the final
	`STOP_AT_DESTINATION` command as `NavCommand` messages.

The default waypoint sequence is:

```python
["97", "198", "6", "91", "75", "54", "372"]
```

## Graph Map

The shared map is located at:

```text
src/utils/Competition_track_graph.graphml
```

`PathPlanner` receives the graph path from its caller. The controller uses
the repository-relative path above, so commands that start the system should
normally run from the repository root.

`threadNavigator` currently uses the Raspberry Pi path below directly:

```text
/home/pi/Documents/Brain/src/utils/Competition_track_graph.graphml
```

When running the navigator on another machine, update `_init_graph()` or
provide a matching deployment path before starting it.

## Routing Behavior

`threadNavigator` uses NetworkX `shortest_path` for each leg of the mission.
It first removes nodes listed in `NODES_TO_AVOID` and falls back to the full
graph if no safe path exists. The reusable `PathPlanner` instead uses weighted
Dijkstra routing through its normalized `weight` attributes.

Turn selection follows this order:

1. Apply a matching entry from `SPECIAL_TURNS`.
2. Return `GO_STRAIGHT` when the middle node is not in
	`INTERSECTION_NODES`.
3. Use the change in heading between the incoming and outgoing graph edges:
	changes above 30 degrees are left turns, changes below -30 degrees are
	right turns, and all other changes are straight.

The navigator tracks the current leg using a 0.50 metre lookahead threshold.
Waypoints are considered reached within 0.8 metres. The master maneuver
sequence is sent once per mission; subsequent paths are used for tracking the
next waypoint.

## Configuration Points

The following values are defined in `threadNavigator` and can be adjusted for
the track or mission:

- `target_nodes`: ordered mission waypoints.
- `INTERSECTION_NODES`: nodes where geometric turn detection is enabled.
- `SPECIAL_TURNS`: explicit exceptions for known intersection routes.
- `NODES_TO_AVOID`: node blacklist used for safe-path planning.
- `FORCE_LEFT_LANE_NODES` and `FORCE_RIGHT_LANE_NODES`: lane overrides.
- `lookahead_distance`: distance used to advance along the current path.

## Message Contract

The navigator subscribes to `LocalizerPose` and publishes `NavCommand`.
Typical outgoing payloads are:

```python
{"maneuver_sequence": ["GO_STRAIGHT", "TURN_LEFT"]}
{"lane_preference": "left"}
{"command": "STOP_AT_DESTINATION"}
```

The controller consumes these commands and applies maneuver, lane, and stop
behavior. Dashboard `NavigateTo` requests are handled by the controller's
`PathPlanner` integration and identify destinations by graph node ID.

## Startup and Integration

The active application startup path creates `threadNavigator` from the
localization process. `processNavigator` is an available worker-process
wrapper, but it is not the path used by the current localization startup.

The package depends on NetworkX and NumPy. Install the project dependencies
from the repository root before running the system:

```bash
./setup.sh
```

For a direct `PathPlanner` use:

```python
from src.pathplanning.pathPlanner import PathPlanner

planner = PathPlanner("src/utils/Competition_track_graph.graphml")
start = planner.find_nearest_node(x, y)
path = planner.dijkstra(start, "372")
print(planner.path_info(path))
```

An empty path means that a node is unknown or no route exists. The planner
returns `None` for an unavailable nearest node or heading and infinity for an
unknown node's distance.
