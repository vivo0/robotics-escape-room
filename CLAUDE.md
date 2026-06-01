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

**Manual flow** (three terminals, all inside `pixi shell`):
```bash
# T1 — launch CoppeliaSim
pixi run coppelia

# T2 — build the scene (CoppeliaSim must be open, simulation stopped)
python src/escape_room/scripts/build_scene.py src/escape_room/scenarios/easy.json

# T3 — build the ROS package (only needed after code changes)
colcon build --packages-select escape_room
source install/setup.sh

# T3 — launch navigation stack + mission FSM (press Play in CoppeliaSim first)
ros2 launch escape_room discovery.launch.py
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

**Discovery**: A Lua script (`lidar_sensor.lua`) injected into the robot drives a CoppeliaSim **Ray-type proximity sensor** (it rotates the sensor and reads the measured hit distance — it does *not* read object positions) and publishes `sensor_msgs/LaserScan` on `/scan`. `robomaster_ros` publishes wheel-encoder odometry on `/odom` and the `odom→base_link` TF. `slam_toolbox` builds a 2D occupancy map from `/scan` and provides `map→odom` localisation — so obstacles are mapped by SLAM from sensor data, never from ground-truth poses. `color_detector_node` finds coloured landmarks via HSV masking on the camera image, transforms each detection to the map frame via TF, and publishes latched `PoseStamped` on `/targets/{cube,plate,door}`.

**Execution**: `explorer_node` sends `NavigateToPose` action goals to Nav2 for all navigation (exploration waypoints, go-to-key, go-to-plate). Short-range gripper manoeuvres still use direct `cmd_vel`: the cube pickup is **closed-loop visual servoing** on the live `/targets/cube_live` bearing (no ground-truth cube coordinate): it turns to centre the cube and approaches until ~0.9 m, where the monocular range (from the cube's pixel height) is still accurate, then commits to a straight **odometry-measured final approach** (the 0.20 m column's base clips out of the camera's bottom edge closer in, so the range can no longer be trusted) and closes the gripper. The plate drop aligns to the latched plate pose. Mission ends (state DONE) once the cube is dropped on the pressure plate and the robot backs clear. Gripper and cube visibility are driven via CoppeliaSim ZMQ.

### ROS nodes (`src/escape_room/escape_room/nodes/`)

| Node | Role |
|---|---|
| `lidar_sensor.lua` | Lua script injected into robot: drives a Ray proximity sensor → `/scan` (LaserScan, 10 Hz). Senses the scene physically; reads no object positions |
| `color_detector_node` | Camera → HSV → blob → `PoseStamped` (map frame) on `/targets/{cube,plate,door}`, all localised by **perception only** (no object pose read from sim). Cube/door: monocular depth from known height + `camera_optical_link` TF. Plate (flat on floor): centroid ray intersected with the floor plane. Cube is also streamed live in base_link on `/targets/cube_live` for the pickup. Each pose is refined while visible |
| `explorer_node` | Mission FSM: sends Nav2 `NavigateToPose` goals + gripper control via ZMQ. Ends in DONE once cube is on pressure plate |

**slam_toolbox** and **Nav2** run as external packages launched via `discovery.launch.py`.

All three nodes are fully implemented. The explorer_node is the mission FSM that delegates long-range navigation to Nav2 and drives short-range manoeuvres via `cmd_vel`.

### CoppeliaSim bridge

Python scripts connect to CoppeliaSim via `coppeliasim_zmqremoteapi_client` (ZMQ on localhost). `build_scene.py` uses this API to construct the room programmatically at startup. `color_detector_node` reads the camera FOV once at init. `explorer_node` drives the gripper and toggles cube detectability at runtime.

### Scenario JSON

Scenarios live in `src/escape_room/scenarios/`. Each file fully describes the room:
- `room` — dimensions and wall thickness
- `robot` — model path, spawn position, initial yaw
- `obstacles` — list of `box`/`cylinder` primitives
- `target_cube`, `pressure_plate` — landmark positions, sizes, and RGB colours
- `doors` (inside `room`) — door openings cut in the walls; doors are visual only and not part of the mission objective

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
