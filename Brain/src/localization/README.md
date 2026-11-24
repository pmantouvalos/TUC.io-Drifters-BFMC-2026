# Localization

The `src/localization` package estimates the vehicle pose on the BFMC
competition track. A pose is represented as `(x, y, theta)`, where `x` and `y`
are map coordinates in metres and `theta` is the vehicle heading in radians.

The runtime combines:

- a graph-constrained particle filter for global localization and recovery;
- an extended Kalman filter (EKF) for precise local tracking;
- camera landmark detections containing type, bearing, and distance;
- vehicle speed and IMU yaw-rate information;
- the competition road graph and landmark map.

## Directory contents

```text
localization/
├── README.md
└── Localizer/
		├── processLocalizer.py
		├── localizer.py
		├── particle_filter.py
		├── ekf.py
		├── map_loader.py
		├── landmarks_all.py
		├── pathfinding.py
		└── threads/
				└── threadLocalizer.py
```

## Runtime architecture

### `Localizer/processLocalizer.py`

`processLocalizer` is a `WorkerProcess` wrapper. Its process hooks are empty
because the work is performed by child threads. `_init_threads()` creates:

- `threadLocalizer`, which estimates and publishes the vehicle pose;
- `threadNavigator`, imported from `src.pathplanning.Navigator`, which consumes
	the published pose and produces navigation commands.

The constructor is:

```python
processLocalizer(queueList, logging, ready_event=None, debugging=False)
```

The process must be started by the normal Brain process manager so that the
shared queue dictionary and all dependent services already exist.

### `Localizer/threads/threadLocalizer.py`

`threadLocalizer` inherits from `ThreadWithStop` and repeatedly performs the
following cycle:

1. Read the newest speed message and convert speed from mm/s to m/s.
2. Read the newest IMU message, parse its yaw in degrees, unwrap angle
	 crossings, and calculate yaw rate in radians per second.
3. Read object detections and keep only entries with both `distance_m` and
	 `bearing_rad`.
4. Ignore `EnterIntersection` detections for localization.
5. Convert valid detections to `Detection` objects.
6. Run `Localizer.predict()` and `Localizer.update()`.
7. Publish the estimated pose and particle data.

The thread subscribes to `CurrentSpeed`, `ImuData`, and `ObjectDetect`. It
publishes:

- `LocalizerPose`: `{"pose": [x, y, theta], "mode": "GLOBAL|LOCAL|RECOVER"}`;
- `LiveAnimation`: particle positions and weights, estimated pose, and mode
	for dashboard visualization.

If map initialization fails, `init_success` remains false and the thread waits
instead of executing filter math. The current implementation loads map files
from these absolute Raspberry Pi paths:

```text
/home/pi/Documents/Brain/src/utils/Competition_track_graph.graphml
/home/pi/Documents/Brain/src/utils/landmarks.json
```

Those files must exist at deployment time. The worker uses up to the first 500
point landmarks and initializes the localizer with 50 particles, 0.2 m/rad
convergence thresholds, and a 0.30 m graph-snap distance.

## Hybrid estimator

### `Localizer/localizer.py`

`Localizer` orchestrates the particle filter and EKF through three modes:

- `GLOBAL`: the particle filter is the primary estimator while the vehicle
	hypothesis is established across the graph;
- `LOCAL`: the EKF is the primary estimator, with a lightweight particle-filter
	consistency check;
- `RECOVER`: the particle filter is reseeded around the last EKF pose, with
	additional global particles, after a likely localization failure.

The public interface is:

```python
localizer.predict(v, omega, dt)
localizer.update(detections)
localizer.pose
localizer.mode
localizer.covariance
```

In `GLOBAL` and `RECOVER`, particle convergence seeds the EKF and switches to
`LOCAL`. In `LOCAL`, failed landmark updates and disagreement with the particle
cloud increment a bad-update counter. Five consecutive bad steps trigger
`RECOVER`.

The EKF mode also projects the pose onto the nearest road edge when the
projection is close enough. The correction includes the edge heading when it is
available.

## Filter implementations

### `Localizer/particle_filter.py`

This module implements adaptive Monte Carlo localization.

#### Data types

- `Particle`: one pose hypothesis and its weight;
- `Detection`: one camera observation with landmark type, bearing in radians,
	and distance in metres.

#### Main behavior

`ParticleFilter` uses a noisy unicycle motion model, likelihood-field landmark
matching, visibility and field-of-view scoring, graph-constrained motion, and
systematic resampling. It also supports:

- KLD-based adaptive particle counts;
- directed-edge and heading checks to avoid wrong-way road hypotheses;
- graph snapping after motion and resampling;
- rarity weighting for landmark types;
- first-sight teleportation for configured meaningful landmarks;
- exact snapping for a detected start line;
- singular-landmark convergence when a type has only one map instance;
- contradiction detection through fast/slow weight tracking;
- recovery particles around a known pose plus globally injected particles.

Useful helpers include `snap_to_graph()`, `sample_motion()`,
`likelihood_field()`, and `kld_particle_count()`.

The particle filter currently starts at a configured graph edge near nodes
`416` and `417`, with a random fallback list if those nodes are unavailable.
When a start detection is received, it snaps particles near node `263` using
the `264 -> 263` edge. These node IDs and the initialization behavior are track
specific.

### `Localizer/ekf.py`

`EKF` is the local three-degree-of-freedom tracker with state `[x, y, theta]`.
It uses a unicycle prediction model:

```text
x'     = x + v cos(theta) dt
y'     = y + v sin(theta) dt
theta' = theta + omega dt
```

Measurement updates use landmark bearing and range. The implementation applies
angle normalization, a 95% chi-squared innovation gate, and a Joseph-form
covariance update for numerical stability. `update_graph()` provides a soft
position and optional heading correction toward the road graph.

`seed_from_particle_filter()` transfers the particle estimate and covariance
into the EKF when global localization converges. Diagnostic properties include
`sigma_xy`, `sigma_theta`, and a readable `repr`.

### `Localizer/map_loader.py`

This module provides small data-loading helpers:

- `load_landmarks(json_path)` reads the top-level `landmarks` array from JSON;
- `load_point_landmarks(data)` keeps entries with `x` and `y`, preserving `id`,
	`type`, coordinates, and optional `facing` data;
- `filter_by_type(landmarks, lm_type)` selects landmarks by type.

The runtime uses `load_landmarks()` and `load_point_landmarks()` to turn the
map JSON into the point landmarks consumed by the filters. Polygon points are
not used directly by the runtime after the JSON has been generated.

## Map and landmark sources

### `Localizer/landmarks_all.py`

This file contains the track landmark source data and conversion script. It
defines raw coordinates, landmark IDs, detector-compatible types, and facing
directions for stop lines, crosswalks, highway signs, roundabouts, parking,
traffic lights, one-way signs, and no-entry signs.

`transform_point()` converts raw track coordinates to metres and flips the
vertical axis using the map height constant. When the file is executed, it
writes a `landmarks.json` file beside the script. It is a generation tool and
is not imported by `threadLocalizer` during normal runtime.

The deployed worker currently reads `src/utils/landmarks.json`, not the output
path beside `landmarks_all.py`. Keep the generated/deployed file synchronized
when changing landmark definitions.

### Graph data

The localization graph is the GraphML track at
`src/utils/Competition_track_graph.graphml`. Nodes contain `x` and `y`
coordinates, and edges describe the drivable track. Localization uses the
graph for motion snapping, heading selection, and EKF correction.

## `Localizer/pathfinding.py`

This is a standalone GraphML pathfinding and visualization utility. It is not
called by `threadLocalizer`, but it is useful for inspecting the competition
track and testing routes.

It provides:

- `load_graph()`: loads the graph and adds Euclidean edge weights;
- `find_path()`: A*, Dijkstra, or unweighted BFS path search;
- `path_length()` and `print_path()`: route reporting;
- `visualize()`: optional Pygame graph and path viewer;
- interactive node selection and a rectangle-based node listing helper.

Run it from the repository root or adjust the map path as needed:

```bash
python3 src/localization/Localizer/pathfinding.py \
	--map src/utils/Competition_track_graph.graphml \
	--source 1 --target 42 --algo astar
```

Available options include `--no-dotted` to exclude dotted edges and
`--visualize` to open the Pygame viewer. The CLI's current default algorithm is
BFS, even though the module description presents A* as the general default.
Use `--algo astar` explicitly when weighted shortest paths are required.

## Dependencies and development notes

The localization package depends on NumPy, NetworkX, and the project's message
and thread infrastructure. Pygame is only required for the optional
`pathfinding.py --visualize` mode.

When testing localization changes:

- provide the expected GraphML and landmark JSON files before constructing
	`threadLocalizer`;
- use detector labels and payload keys consistent with `ObjectDetect`;
- keep bearings in radians and distances in metres at the filter boundary;
- preserve the mm/s to m/s conversion at the message boundary;
- check graph node IDs used by start-line and initial-pose logic;
- verify both `LocalizerPose` and `LiveAnimation` consumers after changing
	published payloads.
