# Architecture

## Overview

Robot escape room: RoboMaster EP in CoppeliaSim must find a magenta cube, carry it to a green pressure plate, which opens a blue door, then drive through it. The mission runs autonomously via a ROS 2 / Nav2 / slam_toolbox stack.

```
CoppeliaSim (physics + ZMQ)
  ├── lidar_sensor.lua  ──────── /scan ────────────► slam_toolbox
  ├── robomaster_sim     ─── /odom + odom→base_link TF ─► Nav2
  └── ZMQ remote API ◄─── Python nodes (gripper, door, detector)

ROS 2 nodes
  ├── color_detector_node ── /targets/{cube,plate,door} ──► explorer_node
  ├── explorer_node  ─────── NavigateToPose + /cmd_vel ──► Nav2 / robot
  └── door_controller ─────── polls ZMQ, moves door panel
```

---

## Simulator Bridge

**Protocol**: `coppeliasim_zmqremoteapi_client` (ZMQ on localhost).

Three Python nodes hold persistent ZMQ connections:

| Node | What it does via ZMQ |
|---|---|
| `build_scene.py` | Constructs the scene at startup |
| `color_detector_node` | Reads object world positions for target localisation |
| `explorer_node` / `door_controller` | Drives gripper, reads/moves door, toggles cube detectability |

### build_scene.py

One-shot script. Connects to a running CoppeliaSim instance, stops any running simulation, clears previously managed objects (`Wall*`, `Door*`, `Obstacle_*`, `TargetCube`, `PressurePlate`, `LidarSensor`, `RoboMaster*`), then rebuilds:

1. **Walls** — four axis-aligned cuboids (North/East/South/West). Door openings are cut by computing solid segments around each door gap.
2. **Doors** — a separate respondable cuboid placed in each gap. Alias `Door_0`, `Door_1`, …
3. **Obstacles** — arbitrary `cuboid` / `cylinder` primitives from the scenario JSON.
4. **TargetCube** — a thin magenta cylinder (0.05 × 0.05 × 0.20 m). Dynamic, low mass (0.02 kg), high friction across Bullet/ODE/Newton engines, angular damping 0.5 to prevent spinning out of the gripper.
5. **PressurePlate** — a flat green cuboid. Static, non-respondable (robot drives over it).
6. **Robot** — loads a `.ttm` model, positions it, then:
   - Injects `gripper_helpers.lua` into `gripper_link_respondable`'s child script.
   - Raises friction on all gripper/finger shapes.
   - Attaches a `LidarSensor` dummy as child of `BaseLinkFrame` and injects `lidar_sensor.lua`.
   - Removes detectability from the entire robot subtree so lidar rays pass through the chassis.

---

## Lidar Sensor (`lidar_sensor.lua`)

Runs as a CoppeliaSim child script (sensing phase, 10 Hz).

- Creates a Ray-type proximity sensor programmatically at init.
- Each tick: rotates the ray sensor to each of 360 angles (−π … π), calls `sim.checkProximitySensorEx` with `handle_all`, records the hit distance or `MAX_RANGE` (5.0 m).
- Publishes `sensor_msgs/LaserScan` on `/scan`, frame `laser_link`, using the ROS 2 native clock via `simROS2.getTime()`.
- The `LidarSensor` dummy is a child of `BaseLinkFrame` at z-offset 0.12 m; it copies `BaseLinkFrame`'s orientation so +X = physical forward.

Static TF: `base_link → laser_link` published by `static_transform_publisher` (offset 0, 0, 0.12 m) with `TRANSIENT_LOCAL` durability so slam_toolbox always receives it regardless of startup order.

---

## Gripper Control (`gripper_helpers.lua`)

Injected by `build_scene.py` into the gripper's existing child script. Adds:

- Lua globals `_ext_target_state` / `_ext_current_state` backed by monkey-patched `sim.setInt32Signal` / `sim.getInt32Signal`. The stock gripper script still calls those signals; the shim intercepts them so the same state is accessible from Python via ZMQ.
- `_ext_set_target(state_int)` — called from `GripperIO.open()` / `.close()`.
- `_ext_get_state()` — called from `GripperIO.reached()`.
- `sim.setObjectParent` is silenced — prevents the stock script from reparenting the cube onto `attachPoint`. The robot holds the cube by friction contact alone, which avoids pose discontinuities when releasing.

`GRIPPER_OPEN = 1`, `GRIPPER_CLOSE = 2`.

---

## TF Tree

```
map
 └── odom          (slam_toolbox: map→odom)
      └── base_link (robomaster_ros: odom→base_link from wheel encoders)
           └── laser_link (static TF, z +0.12 m)
```

slam_toolbox operates in `mode: mapping` (online SLAM from scratch each run). Ceres solver with Levenberg-Marquardt. Loop closure **disabled** (`do_loop_closing: false`) — the room is small enough that odometry drift over one exploration pass is negligible, and loop closure causes a map-frame jump that corrupts the Nav2 costmap.

Map resolution: 5 cm/cell. Map update interval: 0.5 s.

---

## Nav2 Stack

Launched via `nav2_bringup/navigation_launch.py` with `autostart: true`, `use_sim_time: false`.

| Component | Config |
|---|---|
| **Planner** | NavFn (Dijkstra, `use_astar: false`), tolerance 0.5 m, `allow_unknown: true` |
| **Controller** | Regulated Pure Pursuit (RPP), desired velocity 0.15 m/s, lookahead 0.4 m |
| **Local costmap** | 2×2 m rolling window, 5 cm resolution, VoxelLayer + InflationLayer, inflation radius 0.35 m |
| **Global costmap** | Full map, StaticLayer + ObstacleLayer + InflationLayer |
| **Behavior server** | Spin, BackUp, DriveOnHeading, Wait, AssistedTeleop |
| **BT navigator** | `NavigateToPoseNavigator` + `NavigateThroughPosesNavigator` |
| **Velocity smoother** | max 0.2 m/s, max angular 0.6 rad/s |

Goal checker: `SimpleGoalChecker`, xy_tolerance 0.25 m, yaw_tolerance 0.25 rad.

Robot radius: 0.17 m (RoboMaster EP footprint).

---

## Color Detector Node (`color_detector_node.py`)

Subscribes: `/camera/image_color` (sensor_msgs/Image, 10 Hz from robomaster_sim).
Publishes (latched TRANSIENT_LOCAL):
- `/targets/cube`, `/targets/plate`, `/targets/door` — `geometry_msgs/PoseStamped` in `map` frame.
- `/targets/markers` — `visualization_msgs/MarkerArray` (sphere + text label per target, for RViz).

**HSV thresholds** (OpenCV H in [0, 179]):

| Target | Hue range | Min pixels | Real size |
|---|---|---|---|
| cube (magenta) | 140–170 | 80 | 0.12 m |
| plate (green) | 40–80 | 200 | 0.30 m |
| door (blue) | 100–130 | 300 | 0.80 m |

**Localisation pipeline** (per target, first detection only):

1. HSV mask → `cv2.connectedComponentsWithStats` → largest blob centroid.
2. Sim truth: `sim.getObjectPosition(target_handle, -1)` → world XYZ.
3. Express in `base_link` frame using robot's world pose from `sim.getObjectPosition(robot_handle, -1)` and yaw from `sim.getObjectQuaternion`.
4. Transform `base_link → map` via TF buffer lookup.
5. Publish latched PoseStamped.

If the `map→odom` TF is not yet available, detection is stored in `_pending` and retried on every subsequent image frame.

A secondary camera-based depth estimate (pinhole back-projection + monocular depth from known target size: `depth = f_pix × real_size / pixel_size`) is computed for logging/error comparison but not used for the published pose — sim truth is authoritative.

---

## Explorer Node (`explorer_node.py` + `explorer/`)

Mission FSM. Timer callback at `control_rate_hz` (default 4 Hz).

### State machine

```
explore
  → go_to_key       (Nav2 goal: standoff in front of cube)
  → pickup_open     (open gripper, wait)
  → pickup_align    (P-controller: face + approach cube)
  → pickup_close    (close gripper, wait; hide cube from lidar)
  → go_to_plate     (Nav2 goal: plate XY)
  → drop_align      (P-controller: face + approach plate)
  → drop_open       (open gripper, wait; show cube to lidar)
  → drop_backup     (reverse ~8 s at 0.05 m/s)
  → go_to_door      (Nav2 goal: door threshold point)
  → exit_drive      (forward ~5 s at 0.10 m/s — unmapped space)
  → done
```

Startup gate: FSM waits until Nav2 action server is ready, `/map` received, and `map→base_link` TF resolves.

### Submodules

| File | Responsibility |
|---|---|
| `params.py` | Declare + load all ROS parameters onto the node object |
| `sim_setup.py` | Resolve ZMQ handles; auto-detect `pickup_engage_dist` from `attachPoint` x-offset; derive `door_normal` from door bbox dimensions |
| `nav_client.py` | `ActionClient` wrapper for `NavigateToPose`; tracks `active` flag via goal result callback |
| `frontier.py` | Pure function: OccupancyGrid → world-frame centroids of frontier clusters (free cells adjacent to unknown, BFS merge, min 20 cells) |
| `explore.py` | Pick nearest frontier → send Nav2 goal |
| `gripper_io.py` | `GripperIO`: ZMQ `callScriptFunction` for open/close/state-check; toggling cube's `objectspecialproperty_detectable_all` |
| `gripper_phase.py` | `tick_gripper_wait`: dispatcher for `pickup_open`, `pickup_close`, `drop_open` states |
| `pickup.py` | `tick_pickup_align`: P-controller on yaw error then distance error; transitions to `pickup_close` |
| `drop.py` | `tick_drop_align`: same pattern toward plate; `tick_drop_backup`: timed reverse |
| `door.py` | `door_threshold_xy_yaw`: geometry to place approach pose `inset_m` inside the door along outward normal |
| `exit_drive.py` | `tick_exit_drive`: timed forward drive (Nav2 won't plan into unmapped space beyond the door) |
| `pose.py` | `lookup_pose`: TF buffer → `(x, y, yaw)` in map frame |
| `utils.py` | `clamp`, `wrap_angle` |

### Navigation details

**Explore**: Nav2 goals sent to frontier centroids (nearest first). Exploration ends when `len(targets) == 3`.

**go_to_key**: standoff point = `cube_pos - pickup_standoff * heading_to_cube`. Yaw aligned to face cube. Default standoff 0.50 m; `pickup_engage_dist` (the `attachPoint` x-offset, auto-read from sim) is the close-in distance for `pickup_align`.

**go_to_plate**: Nav2 goal directly to plate XY. No yaw constraint.

**go_to_door**: threshold point is `door_pos - inset_m * door_normal` (default 0.20 m inset). Yaw = `atan2(door_normal)`. After Nav2 arrives, `exit_drive` fires for `exit_drive_duration_s` (5 s) at `exit_drive_speed_mps` (0.10 m/s) — this clears the door opening which Nav2 never maps.

### Cube lidar visibility trick

During `pickup_close → go_to_plate`, the cube is hidden from the lidar (`setObjectSpecialProperty(cube_h, 0)`). Without this, the carried cube appears as a stationary obstacle directly in front of `base_link` and Nav2 refuses to move. After `drop_open`, cube detectability is restored so obstacles are correctly sensed again.

---

## Door Controller (`door_controller.py`)

Standalone ROS 2 node. Polls CoppeliaSim at 10 Hz via ZMQ.

**Trigger condition** (cube on plate):
```
|cube.x - plate.x| <= plate_half_x + xy_margin   (default: plate dims + 0.05 m)
|cube.y - plate.y| <= plate_half_y + xy_margin
-0.05 <= cube.z - plate.z <= z_max_offset          (default: 0.15 m)
```

When triggered: `sim.setObjectPosition(door, door_open_pos)` where `door_open_pos` lowers the door by `door_height + 0.05 m` (door slides down into the floor).

`latch: true` (default) — once open, door stays open even if cube is removed.

---

## Scenario JSON

```json
{
  "room":          { width, length, height, wall_thickness, doors: [{wall_side, width, center_offset, color}] },
  "robot":         { name, model_path, position, orientation },
  "obstacles":     [ {type, position, size, color}, … ],
  "target_cube":   { size, position, color },
  "pressure_plate":{ size, position, color }
}
```

`wall_side` 0=North, 1=East, 2=South, 3=West. `center_offset` is offset along the wall from its midpoint. Colors must match the HSV thresholds in `color_detector_node.py`.

Shipped scenarios:
- `easy.json` — 5×4 m room, 3 obstacles, door on North wall.
- `medium.json` — variant with different obstacle placement.

---

## Launch & Startup Sequence

`run.sh` orchestrates a full clean start:

1. Launch CoppeliaSim if not running.
2. Poll ZMQ until reachable (up to 60 s).
3. `python build_scene.py <scenario>` — builds scene, simulation stopped.
4. `colcon build --packages-select escape_room` + `source install/setup.sh`.
5. Start simulation via ZMQ (`startSimulation`, real-time mode required — robomaster_sim has a 3 s sim-time watchdog; without real-time the driver's 1 Hz heartbeat times out in ~1 s wall time).
6. `ros2 launch robomaster_ros ep.launch` (background) — publishes `/odom`, `odom→base_link` TF, `/camera/image_color`.
7. Wait for `/odom` topic.
8. `ros2 run escape_room door_controller` (background).
9. `ros2 launch escape_room discovery.launch.py` (foreground):
   - `static_transform_publisher`: `base_link → laser_link`
   - `rviz2`
   - `async_slam_toolbox_node`
   - `nav2_bringup navigation_launch.py`
   - `color_detector_node`
   - `explorer_node`

---

## Key Design Decisions

| Decision | Reason |
|---|---|
| sim-truth localisation in color_detector | Eliminates monocular depth error; camera estimate exists only for logging |
| Loop closure disabled | Room small enough; loop closure jump corrupts Nav2 costmap |
| Cube held by friction, not reparenting | Avoids pose discontinuity on release; gripper_helpers.lua silences `setObjectParent` |
| Hide cube from lidar while carried | Carried cube would appear as obstacle at `base_link` origin, blocking Nav2 planning |
| exit_drive past door (not Nav2) | Nav2 won't plan into unmapped space; timed open-loop drive is sufficient |
| door slides down (not sideways) | Avoids collision with robot; gravity makes it stay open |
| `TRANSIENT_LOCAL` on `/targets/*` and `/map` subs | Late subscribers (e.g. explorer starting after detector) still receive the last value |
