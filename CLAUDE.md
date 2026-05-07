# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment

All commands must run inside the Pixi environment. Enter it once per shell session:

```bash
pixi shell
```

CoppeliaSim must be installed at `/Applications/coppeliaSim.app` (macOS) or at the path set by `COPPELIASIM_ROOT_DIR` in `pixi.toml` (Linux). It is always launched via `pixi run coppelia` to inherit the correct environment variables.

## Running the demo

**All-in-one** (recommended):
```bash
pixi shell
./run.sh                                         # uses easy.json by default
./run.sh src/escape_room/scenarios/easy.json     # explicit scenario
```

**Manual flow** (four terminals, all inside `pixi shell`):
```bash
# T1 — launch CoppeliaSim
pixi run coppelia

# T2 — build the scene (CoppeliaSim must be open, simulation stopped)
python src/escape_room/scripts/build_scene.py src/escape_room/scenarios/easy.json

# T3 — build the ROS package (only needed after code changes)
colcon build --packages-select escape_room
source install/setup.sh

# T3 — run the door controller (press Play in CoppeliaSim first)
ros2 run escape_room door_controller
```

## Build & test

```bash
colcon build --packages-select escape_room          # incremental build
colcon test  --packages-select escape_room          # run ament lint checks
colcon test-result --verbose                        # show test output
```

Tests are style-only (`ament_flake8`, `ament_pep257`, `ament_copyright`). There are no functional unit tests yet.

## Architecture

The project follows a two-phase pipeline (see `escape_room_pipeline.md` for the full design):

**Phase 1 — Discovery**: the robot explores reactively, builds an occupancy-grid map from ToF sensors, and detects three coloured landmarks (cube, pressure plate, door) via HSV masking on the camera image.

**Phase 2 — Execution**: once all landmarks are known the robot plans paths with A* on the grid (with morphological inflation), follows them with pure pursuit, grasps the cube, places it on the pressure plate, and exits through the door.

### ROS nodes (planned, inside `src/escape_room/escape_room/nodes/`)

| Node | Role |
|---|---|
| `color_detector` | Camera → HSV → blob → `PoseStamped` on latched topics `targets/cube`, `targets/plate`, `targets/door` |
| `mapper` | ToF ranges + odometry → occupancy grid → `/map` (`OccupancyGrid`, 10 cm/cell) |
| `planner` | A* on `/map` + pure-pursuit follower → `cmd_vel` |
| `mission` | Global state machine that orchestrates all other nodes, drives gripper/LEDs |

Currently only `door_controller` is implemented. It polls CoppeliaSim via ZMQ every 100 ms, detects when the cube is positioned on the pressure plate within a configurable XY margin, and slides the door open by dropping it below the floor.

### CoppeliaSim bridge

Python scripts connect to CoppeliaSim via `coppeliasim_zmqremoteapi_client` (ZMQ on localhost). `build_scene.py` uses this API to construct the room programmatically at startup; `door_controller.py` uses it at runtime to read object positions and reposition the door.

### Scenario JSON

Scenarios live in `src/escape_room/scenarios/`. Each file fully describes the room:
- `room` — dimensions and wall thickness
- `robot` — model path, spawn position, initial yaw
- `obstacles` — list of `box`/`cylinder` primitives
- `target_cube`, `pressure_plate`, `door` — landmark positions, sizes, and RGB colours

Object colours must match the HSV ranges in `color_detector_node.py` (cube: magenta ~280–340°; plate: green 80–160°; door: blue 200–260°). The key avoids red (Velodyne beams render red in the camera frame) and yellow (the CoppeliaSim default floor pattern is yellow); magenta is the safe distinct hue.

### Key source locations

- `src/escape_room/escape_room/nodes/` — ROS2 node implementations
- `src/escape_room/scripts/build_scene.py` — scene builder (run standalone, not via colcon)
- `src/escape_room/scenarios/` — scenario JSON files
- `src/escape_room/models/` — CoppeliaSim `.ttm` robot model files
- `src/robomaster_ros/` — upstream RoboMaster ROS2 driver (do not modify)
- `src/robomaster_sim/` — upstream CoppeliaSim simulation plugin (do not modify)
