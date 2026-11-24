# BFMC Brain | TUC.io Drifters 2026

The Brain is the vehicle-side software for the TUC.io Drifters' 2026 entry in
the Bosch Future Mobility Challenge (BFMC). It runs on a Raspberry Pi 5 and
coordinates the vehicle's hardware, control, perception, localization,
planning, dashboard, and communication processes.

## What is included

- Communication with the STM32 Nucleo motor controller, including speed
	control with constant current or constant speed, braking, movement, and
	steering.
- Sensor data acquisition from the IMU, cameras, OAK-D, and ToF sensors.
- APIs for communicating with the environmental servers at the Bosch site.
- Simulated servers for the APIs.
- An extended state machine and system mode configuration.
- Road-lane and road-marking detection.
- A Stanley controller for vehicle control.
- Object recognition.
- Global and local vehicle localization.
- Track-aware path planning and navigation.
- An Angular dashboard for monitoring and calibration.

## Quick start

### Target installation

The supplied setup script installs the system packages, Python dependencies,
Node.js, Angular CLI, and dashboard dependencies. It is intended for a Debian-
based Raspberry Pi deployment and uses global `pip3` and `npm` installation.

```bash
chmod +x setup.sh
./setup.sh
```

The script may require `sudo` access and an active network connection. Review
it before running it on a development machine because it upgrades apt packages
and installs tools system-wide.

### Start the Brain

From the repository root:

```bash
python3 main.py
```

`main.py` starts the process manager and coordinates the gateway, dashboard,
serial handler, joystick input, OAK-D, ToF sensors, controller, and localizer.
The process waits for its readiness events before entering the main runtime
loop. Stop it with `Ctrl+C` so the child processes can shut down cleanly.

### Start the dashboard

Install the project first, then run the Angular development server:

```bash
cd src/dashboard/frontend
npm start
```

The first run may ask for confirmation in the terminal. The dashboard is
available at the local address printed by Angular.

## Architecture

The Brain is organized as cooperating worker processes. Processes exchange
typed messages through the shared queues in `src/utils/messages`, while each
process owns the threads that interact with its device or subsystem.

The typical runtime flow is:

1. Hardware threads read sensors, actuators, or network input.
2. Raw values are validated and converted into project messages.
3. Control, localization, planning, dashboard, and monitoring components
	 subscribe to the messages they need.
4. The controller and serial handler send commands and feedback to the vehicle.
5. The state machine enables or disables components according to the current
	 system mode.

## Repository layout

```text
.
├── main.py                    # Main process manager and runtime entry point
├── setup.sh                   # Raspberry Pi and dashboard installation
├── requirements.txt           # Python runtime dependencies
├── calibration/               # Motor calibration templates and tools
├── monitoring/                # Monitoring project notes and materials
├── services/                  # systemd/autostart and Wi-Fi fallback scripts
├── src/
│   ├── control/               # Controller and control modes
│   ├── dashboard/             # Dashboard backend and Angular frontend
│   ├── gateway/               # External/environmental communication
│   ├── hardware/              # Camera, OAK-D, serial, joystick, and ToF I/O
│   ├── localization/          # Particle filter, EKF, and map handling
│   ├── pathplanning/           # Navigator and track-aware planning
│   ├── statemachine/           # State transitions and system modes
│   ├── templates/              # Reusable process and thread templates
│   └── utils/                  # Messages, maps, output, and shared helpers
└── Project status/             # Progress reports and qualification material
```

## Main subsystems

### Control and actuation

The control stack receives vehicle state, joystick input, sensor data, and
navigation commands, then produces steering and movement commands. The serial
handler provides the communication path between the Raspberry Pi and the
Nucleo motor controller.

### Perception and hardware

The hardware package connects the Brain to the camera, OAK-D, IMU, serial
controller, joystick UDP input, tag reader, and ToF sensors. Hardware modules
are designed to run as worker processes and publish their data through the
shared message system.

### Localization

Localization combines a graph-constrained particle filter with an extended
Kalman filter (EKF). It uses camera landmark detections, vehicle speed, IMU
yaw rate, the competition track graph, and the landmark map to estimate the
vehicle pose `(x, y, theta)`. See [the localization README](src/localization/README.md)
for the estimator modes, map requirements, and diagnostic utilities.

### Path planning

The path-planning stack uses the competition track graph to generate navigation
commands. The standalone pathfinding utility can inspect the graph and test
routes without starting the complete Brain process manager.

### Dashboard and monitoring

The dashboard provides runtime visualization, calibration interfaces, and
process monitoring. Dashboard autostart files and service scripts are kept in
`services/angular-autostart/`; Brain monitoring and startup scripts are in
`services/brain-autostart/`.

## Hardware and deployment notes

This project is primarily intended for the target Raspberry Pi and vehicle
hardware. Before starting the full runtime, verify that:

- the STM32 controller is available under `/dev/serial/by-id/` and the user has
	permission to access it;
- the OAK-D, camera, IMU, and ToF devices are connected and configured;
- required model and map files are present on the deployment machine;
- the expected UDP joystick and environmental-server settings are configured;
- the dashboard and Brain services use the correct paths and user account.

Localization currently loads its deployed map files from:

```text
/home/pi/Documents/Brain/src/utils/Competition_track_graph.graphml
/home/pi/Documents/Brain/src/utils/landmarks.json
```

For safe development, enable debugging where supported and verify published
messages before enabling motor output on a real vehicle. Hardware-dependent
components cannot be fully exercised without their physical devices.

## Useful commands

Run the standalone localization pathfinding utility from the repository root:

```bash
python3 src/localization/Localizer/pathfinding.py \
	--map src/utils/Competition_track_graph.graphml \
	--source 1 --target 42 --algo astar
```

Use `--no-dotted` to exclude dotted edges or `--visualize` to open the Pygame
viewer. The command-line default is BFS; use `--algo astar` explicitly for a
weighted shortest-path search.

## Documentation

- [BFMC project documentation](https://documentation.boschfuturemobility.com/index.html)
- [Localization subsystem](src/localization/README.md)
- [Hardware subsystem](src/hardware/README.md)
- [Control subsystem](src/control/README.md)
- [Path-planning subsystem](src/pathplanning/README.md)
- [Dashboard frontend](src/dashboard/frontend/README.md)

## License

See [LICENSE](LICENSE) for the project license and redistribution terms.
