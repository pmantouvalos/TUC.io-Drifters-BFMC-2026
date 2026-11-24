# Hardware subsystem

The `src/hardware` package connects the Brain processes to the vehicle's
physical sensors, actuators, and external input devices. Each hardware module
is wrapped in a `WorkerProcess` and communicates with the rest of the system
through the shared message queues in `src/utils/messages`.

## Directory contents

```text
hardware/
├── README.md
├── camera/
│   ├── processCamera.py
│   └── threads/
├── joy_udp/
│   ├── processjoy_udp.py
│   └── threads/
├── oakd/
│   ├── processOakD.py
│   ├── threads/
│   └── utils/
├── serialhandler/
│   ├── processSerialHandler.py
│   └── threads/
├── tag_reader/
│   ├── processtag_reader.py
│   └── threads/
└── tof_sensor/
		├── processtof_sensor.py
		└── threads/
```

## Hardware processes

### Camera

`camera/processCamera.py` starts `threadCamera`, which captures camera frames
and publishes them for dashboard and vision consumers. The process listens to
`StateChange` messages and pauses or resumes its thread according to the
camera setting in `SystemMode`.

### Joystick UDP

`joy_udp/processjoy_udp.py` receives remote joystick input over UDP and
publishes steering, throttle, and button messages. It is used for manual
control and dashboard interaction when the vehicle is not being driven by
the automatic controller.

### OAK-D

`oakd/processOakD.py` manages the OAK-D camera pipeline. Its worker threads
provide visual detections and related camera data, such as detected road
objects or landmarks, to the message system. The OAK-D device and its model
configuration must be available on the deployment machine.

### Serial handler

`serialhandler/processSerialHandler.py` manages the serial connection between
the Raspberry Pi and the STM32 motor controller. It creates separate reader
and writer threads and records serial history in `temp/serial_history.log`.

The handler connects at `115200` baud using the configured device path under
`/dev/serial/by-id/`. It publishes `SerialConnectionState`, pauses its worker
threads after a disconnection, and retries the connection automatically.
Motor commands and sensor feedback travel through this serial link.

### Tag reader

`tag_reader/processtag_reader.py` starts the tag-reader worker responsible for
reading the vehicle's tag or RFID input and publishing the corresponding
messages. The reader requires its physical interface and device configuration
to be present at runtime.

### ToF sensors

`tof_sensor/processtof_sensor.py` starts `threadtof_sensor`, which reads the
time-of-flight distance sensors and publishes forward, left, and right/rear
distance measurements. The control process uses these readings for obstacle
avoidance, parking, and safety stops.

## Runtime contract

Hardware processes are normally started by the Brain process manager rather
than directly. Their constructors receive the shared queue dictionary, a
logger, an optional readiness event, and a debugging flag. The inherited
`WorkerProcess` lifecycle starts, pauses, resumes, and stops the worker
threads.

The usual data flow is:

1. A hardware thread reads a physical device or network input.
2. The thread validates and converts the raw value into a project message.
3. A message sender publishes the value to the shared queues.
4. Control, localization, dashboard, and monitoring processes subscribe to
	 the message type they need.

Most subscribers use the `lastOnly` policy so consumers receive the newest
sensor value instead of processing stale measurements. Hardware threads must
stop their device handles and unsubscribe from messages when their process is
stopped.

## Deployment requirements

- Run on the target Raspberry Pi or another machine with the required devices
	attached; hardware modules are not fully testable without those devices.
- Ensure the serial controller is visible under `/dev/serial/by-id/` and that
	the running user has permission to open it.
- Install the Python dependencies from the repository's `requirements.txt`.
- Provide the OAK-D device, camera permissions, and model/configuration files
	required by the OAK-D pipeline.
- Confirm that the UDP joystick sender and any tag-reader interface use the
	expected network address and device settings.

For safe development, start with `debugging=True` where supported and verify
the published messages before enabling motor output on a real vehicle.
