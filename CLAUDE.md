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

The project uses a Nav2 + slam_toolbox navigation stack:

**Discovery**: A Lua script (`lidar_sensor.lua`) injected into the robot casts ground-truth rays and publishes `sensor_msgs/LaserScan` on `/scan`. `robomaster_ros` publishes wheel-encoder odometry on `/odom` and the `odom→base_link` TF. `slam_toolbox` builds a 2D occupancy map and provides `map→odom` localisation. `color_detector_node` finds coloured landmarks via HSV masking on the camera image, transforms each detection to the map frame via TF, and publishes latched `PoseStamped` on `/targets/{cube,plate,door}`.

**Execution**: `explorer_node` sends `NavigateToPose` action goals to Nav2 for all navigation (exploration waypoints, go-to-key, go-to-plate, go-to-door). Short-range gripper manoeuvres (cube pickup alignment, plate drop alignment) still use direct `cmd_vel`. Gripper and cube attach/detach are driven via CoppeliaSim ZMQ.

### ROS nodes (`src/escape_room/escape_room/nodes/`)

| Node | Role |
|---|---|
| `lidar_sensor.lua` | Lua script injected into robot: ZMQ ray-caster → `/scan` (LaserScan, 10 Hz) |
| `color_detector_node` | Camera → HSV → blob → sim-truth → TF → `PoseStamped` (map frame) on `/targets/{cube,plate,door}` |
| `explorer_node` | Mission FSM: sends Nav2 `NavigateToPose` goals + gripper control via ZMQ |
| `door_controller` | Polls CoppeliaSim via ZMQ, opens door when cube is on pressure plate |

**slam_toolbox** and **Nav2** run as external packages launched via `discovery.launch.py`.

Currently the door_controller and color_detector_node are fully implemented. The explorer_node is the mission FSM that delegates navigation to Nav2.

### CoppeliaSim bridge

Python scripts connect to CoppeliaSim via `coppeliasim_zmqremoteapi_client` (ZMQ on localhost). `build_scene.py` uses this API to construct the room programmatically at startup; `door_controller.py` uses it at runtime to read object positions and reposition the door.

### Scenario JSON

Scenarios live in `src/escape_room/scenarios/`. Each file fully describes the room:
- `room` — dimensions and wall thickness
- `robot` — model path, spawn position, initial yaw
- `obstacles` — list of `box`/`cylinder` primitives
- `target_cube`, `pressure_plate`, `door` — landmark positions, sizes, and RGB colours

Object colours must match the HSV ranges in `color_detector_node.py` (cube: magenta ~280–340°; plate: green 80–160°; door: blue 200–260°). Cube avoids red (floor/wall artefacts) and yellow (CoppeliaSim default floor pattern); magenta is the safe distinct hue.

### Key source locations

- `src/escape_room/escape_room/nodes/` — ROS2 node implementations
- `src/escape_room/config/` — slam_toolbox and Nav2 YAML parameter files
- `src/escape_room/launch/discovery.launch.py` — main launch (lidar + slam + Nav2 + mission)
- `src/escape_room/scripts/build_scene.py` — scene builder (run standalone, not via colcon)
- `src/escape_room/scenarios/` — scenario JSON files
- `src/escape_room/models/` — CoppeliaSim `.ttm` robot model files (use `RoboMasterEP.ttm`, not the lidar variant)
- `src/robomaster_ros/` — upstream RoboMaster ROS2 driver (do not modify)
- `src/robomaster_sim/` — upstream CoppeliaSim simulation plugin (do not modify)
