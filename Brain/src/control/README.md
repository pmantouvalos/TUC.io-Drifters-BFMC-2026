# Control subsystem

The `src/control` package is the vehicle-control layer of the Brain project. It
converts joystick input, camera detections, line classifications, ToF readings,
IMU data, V2X information, and navigation commands into steering and speed
commands for the motor subsystem.

The control process is designed to run alongside the other Brain processes and
communicates with them through the project's message queues. The main control
loop runs in a dedicated thread at approximately 30 ms intervals.

## Directory contents

```text
control/
├── README.md
├── controller/
│   ├── processcontroller.py
│   └── threads/
│       ├── threadcontroller.py
│       └── .giosaveYXUaps
└── utils/
		└── controlMode.py
```

### `controller/processcontroller.py`

`processcontroller` is a `WorkerProcess` wrapper around the controller thread.
It is responsible for:

- receiving the shared-memory object, queue dictionary, logger, and optional
	ready event;
- creating one `threadcontroller` instance in `_init_threads()`;
- allowing the inherited `WorkerProcess` lifecycle to start, pause, resume, and
	stop that thread.

`state_change_handler()` and `process_work()` are currently empty because the
actual work is performed by `threadcontroller`. The process still inherits the
standard `WorkerProcess.run()` loop, so the empty hooks are intentional for the
current architecture.

The constructor is:

```python
processcontroller(sharedMemory, queueList, logging, ready_event=None,
								 debugging=False)
```

### `controller/threads/threadcontroller.py`

`threadcontroller` is the implementation of the control system. It inherits
from `src.templates.threadwithstop.ThreadWithStop` and is configured with a
30 ms pause between work cycles.

#### Initialization

During construction it:

- stores the queue dictionary, logger, debug flag, and shared-memory object;
- creates subscribers for joystick, KLEM, state changes, vision, ToF, object
	detection, speed, IMU, lane classification, navigation, semaphore, location,
	and intersection-mask messages;
- creates senders for steering, speed, parking scan gate, and current-node
	messages;
- initializes the current `ControlMode` to `INIT`;
- initializes parking, overtaking, roundabout, intersection, localization,
	semaphore, and telemetry state;
- loads the competition track graph through `PathPlanner`;
- opens a non-blocking UDP telemetry socket targeting `192.168.50.100:9002`.

If path-planner initialization fails, the thread logs a warning and continues
with `planner = None`. Other initialization failures are logged after the
thread attempts to unsubscribe its subscribers.

#### Runtime lifecycle

`ThreadWithStop.run()` repeatedly calls:

1. `state_change_handler()` to consume dashboard mode changes;
2. `thread_work()` to read the latest inputs and execute the current mode;
3. a short wait before the next cycle.

The thread can be paused and resumed through the inherited thread API. `stop()`
unsubscribes from all message queues, commands both motors to stop, requests the
dashboard stop mode, and terminates the thread. `stopTOF()` provides a separate
ToF-stop path and also sends zero motor commands.

#### Inputs

The controller subscribes to these message types:

- `Klem`: actuator-enable state;
- `JoyAxes` and `JoyButton`: manual steering/throttle and dashboard controls;
- `StateChange`: requested `DEFAULT`, `STOP`, `LEGACY`, `MANUAL`, or `AUTO` mode;
- `VisionManeuver`: lane-keeping steering values;
- `TOFRear`, `TOFLeft`, `TOFRight`, and `TOFForward`: obstacle and parking
	distances;
- `ObjectDetect`: detected signs, lights, pedestrians, cars, and road objects;
- `CurrentSpeed` and `ImuData`: vehicle motion feedback;
- `LeftLineClassifier` and `RightLineClassifier`: solid/dashed lane boundaries;
- `NavCommand`: the next route instruction;
- `Semaphores` and `Location`: V2X traffic-light and localization data;
- `NavigateTo`: dashboard destination requests;
- `IntersectionMask`: nearest-intersection information.

Subscribers use the `lastOnly` policy, so each cycle works with the newest
available value rather than processing an unbounded backlog.

#### Outputs

The controller publishes:

- `SteerMotor`: steering commands constrained to the configured steering range;
- `SpeedMotor`: throttle/braking commands constrained to the configured speed
	range;
- `ParkingScanGate`: enables or disables parking-side scanning;
- `CurrentNode`: the current path-planning node for dashboard consumers;
- UDP telemetry containing controller state, speed, steering, detections,
	localization, and related runtime information.

Motor commands are sent as strings because that is the format expected by the
existing message handlers and serial motor process.

## Control behavior

`thread_work()` reads the latest messages, updates cached sensor state, handles
button and KLEM input, runs the state machine, and sends telemetry. The main
automatic-control path is:

1. apply lane keeping using the vision steering value and a configured speed;
2. prioritize safety conditions such as a red semaphore, pedestrian, stop sign,
	 traffic light, or obstacle;
3. select a maneuver state when a sign, road feature, or navigation command is
	 detected;
4. execute the maneuver until its timer, sensor condition, or trajectory is
	 complete;
5. return to `AUTO` and continue lane keeping.

### Lane keeping and trajectories

`_apply_lane_keeping()` maps camera steering and joystick throttle values to the
motor ranges, applies deadbands, and can override speed or lane side. The
`TrajectoryFollower` helper generates cubic Bezier paths for forward, left, and
right intersection maneuvers. It follows those paths using a bicycle model and
Stanley-style steering correction, with steering limited to 25 degrees.

### Safety and traffic behavior

The state machine supports:

- manual control and automatic lane following;
- stop-line and stop-sign handling with a timed stop;
- traffic-light handling with visual green-light detection and V2X semaphore
	confirmation;
- pedestrian and crosswalk yielding;
- highway entry and exit;
- car avoidance and timed overtaking with dashed-line and ToF checks;
- priority-controlled intersections;
- roundabout yield, cruise, and exit maneuvers.

### Parking behavior

Parking is a multi-stage sequence. The controller slows at a parking sign,
disables scanning during the blind approach, searches for a free bay using
filtered left/right ToF readings, then performs positioning, reverse entry,
correction, forward placement, waiting, and unparking. Parking has configured
timeouts, occupancy/free-space confirmation frames, a stop duration, and a
cooldown to prevent immediately re-entering the same parking zone.

## `utils/controlMode.py`

`ControlMode` is the `Enum` used by `threadcontroller.control_mode`. Its values
are grouped by behavior:

- lifecycle: `ERROR`, `INIT`, `IDLE`, `START`, `EXIT_START`;
- driving: `MANUAL`, `AUTO`, `MOVE`;
- stops and pedestrians: `FIND_STOP_LINE`, `STOP_SIGN`, `SIMPLE_PEDESTRIAN`,
	`CROSSWALK`, `PEDESTRIAN_AT_CROSSWALK`,
	`MOVE_FOR_PEDESTRIAN_IN_CROSSWALK`, `TRAFFIC_LIGHT`;
- highways and overtaking: `ENTER_HIGHWAY`, `ENTER_HIGHWAY_2`, `EXIT_HIGHWAY`,
	`OVERTAKE`, `OVERTAKE_LEFT`, `OVERTAKE_RIGHT`, `CONTINUE_AHEAD`,
	`VEER_LEFT`, `VEER_RIGHT`, `AVOID_CAR`;
- parking: `ENTER_PARKING`, `FIND_PARKING_SPOT`, `PARKING_POSITION`,
	`PARKING_ALIGN`, `PARKING_REVERSE`, `PARKING_CORRECT`, `PARKING_FORWARD`,
	`PARKED`, `UNPARKING`, `EXIT_PARKING`;
- intersections: `PRIORITY`, `ENTER_INTERSECTION`, `INTERSECTION_FWD`,
	`INTERSECTION_LEFT`, `INTERSECTION_RIGHT`, `DIRECT_INTERSECTION`;
- roundabouts: `ROUNDABOUT_APPROACH`, `ROUNDABOUT_YIELD`,
	`ROUNDABOUT_CRUISE`, `ROUNDABOUT_EXIT`.

Some enum values are retained for compatibility with older state-machine or
dashboard paths even when the current thread does not contain a distinct branch
for every value.

## Configuration and dependencies

Most control tuning is defined as module-level constants in
`threadcontroller.py`, including motor ranges, deadbands, nominal speeds,
parking timings, ToF thresholds, graph path, waypoint threshold, and rolling
checkpoints. Change these values carefully because they directly affect vehicle
motion.

The control package depends on the shared project infrastructure rather than
being standalone. In particular, it requires:

- the message sender/subscriber implementations and message definitions under
	`src/utils/messages`;
- `WorkerProcess` and `ThreadWithStop` under `src/templates`;
- `StateMachine` under `src/statemachine`;
- `PathPlanner` and the graph at `src/utils/Competition_track_graph.graphml`;
- NumPy and the project's runtime dependencies from `requirements.txt`.

Start the controller through the application's normal process manager so that
the shared queue dictionary and shared memory are initialized first. Running
`processcontroller.py` directly only adjusts the import path; it does not create
the queues or start a complete Brain runtime by itself.

## Development notes

- Keep motor outputs at zero when adding or testing a new safety state.
- Preserve the message names and payload formats used by the other processes.
- Prefer updating `ControlMode` and the `threadcontroller` transition logic
	together when adding a new behavior.
- The hidden `.giosaveYXUaps` file is an editor save artifact containing an older
	copy of the controller. It is not imported by the runtime and should not be
	treated as the source of truth; use `threadcontroller.py` instead.
