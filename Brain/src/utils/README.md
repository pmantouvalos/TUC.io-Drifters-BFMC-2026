# Utilities

This folder contains shared support files used across the Brain project. These are general-purpose assets that do not belong to a single subsystem, but are consumed by multiple processes for logging, message definitions, map data, and runtime configuration.

## Purpose

The utilities package centralizes the files that are reused by several modules, especially:

- message definitions for inter-process communication;
- logging and console output helpers;
- competition map and landmark data used by localization and path planning;
- shared static configuration used by the vehicle software.

## Directory contents

```text
utils/
├── README.md
├── bigPrintMessages.py
├── Competition_track_graph.graphml
├── landmarks.json
├── outputWriters.py
├── table_state.json
└── messages/
    ├── allMessages.py
    ├── messageHandlerSender.py
    └── messageHandlerSubscriber.py
```

## Key files

### `bigPrintMessages.py`

Contains large ASCII/terminal banner definitions used for status messages and startup output. These are mainly for readability during development and debugging.

### `outputWriters.py`

Provides queue-backed writers for line-oriented output. The `QueueWriter` helper appends text to a local buffer and pushes complete lines to a queue, which makes it useful for logging from threads and subprocesses.

### `messages/`

This folder defines the message contracts used by the distributed Brain processes. The central `allMessages.py` file contains message enums such as queue name, owner, message ID, and payload type, and is referenced by multiple subsystems across the project.

### `Competition_track_graph.graphml`

GraphML map of the competition track. It is used for localization, route planning, and graph-based navigation tasks.

### `landmarks.json`

Contains landmark positions and metadata used by the localization pipeline. The data is loaded by localization code when estimating the vehicle pose.

### `table_state.json`

Stores state or table-based configuration values for the project. This file is used as structured configuration data and may be extended when more shared state is needed.

## Usage notes

- Keep shared data files here when multiple processes depend on the same definition.
- Prefer updating message definitions in the shared message package instead of duplicating enum classes in subsystem code.
- Treat the map and landmark files as deployment-relevant artifacts; they must remain synchronized with the localization logic.

## Related modules

This folder supports the following parts of the project:

- `src/localization` for landmark and graph data;
- `src/pathplanning` for graph and route information;
- runtime processes using the shared message catalogue;
- dashboard and debugging components that receive queue-based logs and status output.
