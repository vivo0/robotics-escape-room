# Architecture

## Overview

Robot escape room: RoboMaster EP in CoppeliaSim must find a magenta cylinder (key), carry it to
a green pressure plate, and drop it there. Mission ends when the cube lands on the plate.

```
CoppeliaSim (physics + ZMQ)
  ├── lidar_sensor.lua  ──────── /scan ────────────► slam_toolbox
  ├── robomaster_sim     ─── /odom + odom→base_link TF ─► Nav2
  └── ZMQ remote API ◄─── Python nodes (gripper, cube visibility, camera FOV)

ROS 2 nodes
  ├── color_detector_node ── /targets/{cube,plate,door} ──► explorer_node
  └── explorer_node  ─────── NavigateToPose + /cmd_vel ──► Nav2 / robot
```

---

## Simulator Bridge

**Protocol**: `coppeliasim_zmqremoteapi_client` (ZMQ on localhost).

| Node | What it does via ZMQ |
|---|---|
| `build_scene.py` | Constructs the scene at startup |
| `color_detector_node` | Reads camera FOV once at init (a scalar constant, not object positions) |
| `explorer_node` | Drives gripper open/close; toggles cube's `detectable_all` flag |

### build_scene.py

One-shot script. Connects to a running CoppeliaSim instance, stops any running simulation, clears
previously managed objects (`Wall*`, `Door*`, `Obstacle_*`, `TargetCube`, `PressurePlate`,
`LidarSensor`, `RoboMaster*`), then rebuilds:

1. **Walls** — four axis-aligned cuboids (North/East/South/West). Door openings are cut by computing solid segments around each door gap.
2. **Doors** — a separate respondable cuboid placed in each gap. Alias `Door_0`, `Door_1`, … (visual/structural only; not part of mission logic).
3. **Obstacles** — arbitrary `cuboid` / `cylinder` primitives from the scenario JSON.
4. **TargetCube** — a thin magenta cylinder (0.03 × 0.03 × 0.20 m). Dynamic, low mass (0.01 kg), high friction across Bullet/ODE/Newton engines, angular damping 0.8 to prevent spinning out of the gripper.
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
- The `LidarSensor` dummy is a child of `BaseLinkFrame` at z-offset 0.12 m; +X = physical forward.

Static TF: `base_link → laser_link` published by `static_transform_publisher` (offset 0, 0, 0.12 m)
with `TRANSIENT_LOCAL` durability so slam_toolbox always receives it regardless of startup order.

---

## Gripper Control (`gripper_helpers.lua`)

Injected by `build_scene.py` into the gripper's existing child script. Adds:

- Lua globals `_ext_target_state` / `_ext_current_state` backed by monkey-patched `sim.setInt32Signal` / `sim.getInt32Signal`. The stock gripper script still calls those signals; the shim intercepts them so the same state is accessible from Python via ZMQ.
- `_ext_set_target(state_int)` — called from `GripperIO.open()` / `.close()`.
- `_ext_get_state()` — called from `GripperIO.is_open()` / `.is_closed()`.
- `sim.setObjectParent` is silenced — prevents the stock script from reparenting the cube onto `attachPoint`. The robot holds the cube by friction contact alone, avoiding pose discontinuities on release.

`GRIPPER_OPEN = 1`, `GRIPPER_CLOSE = 2`.

---

## TF Tree

```
map
 └── odom          (slam_toolbox: map→odom)
      └── base_link (robomaster_ros: odom→base_link from wheel encoders)
           └── laser_link (static TF, z +0.12 m)
```

slam_toolbox operates in `mode: mapping` (online SLAM from scratch each run). Ceres solver with
Levenberg-Marquardt. Loop closure **disabled** (`do_loop_closing: false`) — the room is small enough
that odometry drift over one exploration pass is negligible, and loop closure causes a map-frame
jump that corrupts the Nav2 costmap.

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
- `/targets/cube_live` — `PoseStamped` in `base_link` frame, streamed every frame, for visual-servoing pickup.
- `/targets/markers` — `visualization_msgs/MarkerArray` (coloured sphere per target, for RViz).

**Localisation is fully perception-based** — no object poses are read from the simulator.

**HSV thresholds** (OpenCV H in [0, 179]):

| Target | Hue range | Min pixels | Real height used |
|---|---|---|---|
| cube (magenta) | 140–170 | 80 | 0.20 m |
| plate (green) | 40–80 | 200 | floor-ray (no height) |
| door (blue) | 100–130 | 300 | 0.50 m |

**Localisation pipeline** per target each frame:

1. HSV mask → `cv2.connectedComponentsWithStats` → largest blob centroid + bounding box.
2. **Cube / door** (upright objects, known height): pinhole monocular depth — `z = f_pix × height_m / h_px`. Back-project centroid to that depth → 3D point in `camera_optical_link`. Transform via TF to `map`. Skipped if blob touches top/bottom image edge (clipped height → wrong depth).
3. **Plate** (flat, no usable height): shoot a ray through the centroid pixel, transform origin + ray-tip to `map` via TF, intersect with floor plane `z = 0` → `(x, y)` in map.
4. Publish latched `PoseStamped`; refined while visible (jitter < 0.05 m suppressed for marker updates).

Camera FOV is read from the sim **once at init** (vision sensor `perspective_angle` param) for the focal-length calculation. This is a scalar constant, not a position.

---

## Explorer Node (`explorer_node.py` + `explorer/`)

Mission FSM. Timer callback at `control_rate_hz` (default 4 Hz).

### State machine

```
explore
  → go_to_key       (Nav2 goal: standoff 0.9 m in front of cube)
  → pickup_open     (open gripper, wait)
  → pickup_align    (visual servo: face + approach cube; lunge last 0.9 m on odometry)
  → pickup_close    (close gripper, wait; hide cube from lidar)
  → go_to_plate     (Nav2 goal: plate XY)
  → drop_align      (P-controller: face plate + drive to carried-cube offset)
  → drop_open       (open gripper, wait; show cube to lidar)
  → drop_backup     (reverse 0.25 m on odometry so gripper clears cube)
  → done
```

Startup gate: FSM waits until Nav2 action server is ready, `/map` received, and `map→base_link` TF resolves.

### Submodules (`explorer/`)

| File | Responsibility |
|---|---|
| `state.py` | `State` enum for all FSM states |
| `nav_client.py` | `ActionClient` wrapper for `NavigateToPose`; tracks `active` / `succeeded` flags |
| `frontier.py` | Pure function: OccupancyGrid → world-frame centroids of frontier clusters (free cells adjacent to unknown, BFS merge, min 20 cells) |
| `gripper.py` | `GripperIO`: ZMQ `callScriptFunction` for open/close/state-check; toggling cube's `objectspecialproperty_detectable_all` |

### Navigation details

**Explore**: Nav2 goals sent to nearest frontier centroid. Ends when all 3 targets seen.

**go_to_key**: standoff point = `cube_pos − pickup_standoff × heading_to_cube` (default 0.9 m). Yaw aligned to face cube.

**pickup_align (visual servo)**: P-control on yaw error from `/targets/cube_live` (base_link bearing), then forward. At 0.9 m the cube base clips out of the camera bottom edge so monocular range becomes unreliable — the remaining distance is frozen and driven open-loop on odometry (`_lunge_dist = range − engage_dist − lunge_margin`).

**go_to_plate**: Nav2 goal directly to plate XY; yaw toward plate.

**drop_align**: P-control on yaw error + distance error (target distance = `engage_dist + lunge_margin` so the carried cube lands at plate centre).

**drop_backup**: reverse straight measured in `odom` (smooth, no SLAM jumps) for 0.25 m before turning so the open gripper doesn't sweep the cube off the plate.

### Cube lidar visibility trick

During `pickup_close → go_to_plate`, the cube is hidden from the lidar (`setObjectSpecialProperty(cube_h, 0)`). Without this, the carried cube appears as a stationary obstacle at `base_link` and Nav2 refuses to plan. After `drop_open`, detectability is restored.

---

## Scenario JSON

```json
{
  "room":          { "width", "length", "height", "wall_thickness", "doors": [{"wall_side", "width", "center_offset", "color"}] },
  "robot":         { "name", "model_path", "position", "orientation" },
  "obstacles":     [ {"type", "position", "size", "color"}, … ],
  "target_cube":   { "size", "position", "color" },
  "pressure_plate":{ "size", "position", "color" }
}
```

`wall_side` 0=North, 1=East, 2=South, 3=West. `center_offset` is offset along the wall from its midpoint. Colors must match the HSV thresholds in `color_detector_node.py`.

Shipped scenarios:
- `easy.json` — 5×4 m room, 3 obstacles, door on North wall, cube and plate on opposite sides.
- `medium.json` — same room, 4 obstacles with a wall-spanning divider, offset door, different spawn.

---

## Launch & Startup Sequence

`run.sh` orchestrates a full clean start:

1. Launch CoppeliaSim if not running.
2. Poll ZMQ until reachable (up to 60 s).
3. `python build_scene.py <scenario>` — builds scene, simulation stopped.
4. `colcon build --packages-select escape_room` + `source install/setup.sh`.
5. Start simulation via ZMQ (`startSimulation`, real-time mode — robomaster_sim has a 3 s sim-time watchdog; without real-time the driver's 1 Hz heartbeat times out in ~1 s wall time).
6. `ros2 launch robomaster_ros ep.launch` (background) — publishes `/odom`, `odom→base_link` TF, `/camera/image_color`.
7. Wait for `/odom` topic.
8. `ros2 launch escape_room discovery.launch.py` (foreground):
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
| Camera-only localisation in color_detector | No sim truth reads; all perception from RGB image + TF. Monocular depth for upright objects, floor-ray for flat objects |
| Loop closure disabled | Room small enough; loop closure jump corrupts Nav2 costmap |
| Cube held by friction, not reparenting | Avoids pose discontinuity on release; gripper_helpers.lua silences `setObjectParent` |
| Hide cube from lidar while carried | Carried cube appears as obstacle at `base_link` origin, blocking Nav2 planning |
| Odometry for short moves (lunge, backup) | Smooth over short distance; no SLAM jumps that would corrupt measured displacement |
| Mission ends at DROP_BACKUP → DONE | Robot doesn't navigate to door; dropping cube on plate is the win condition |
| `TRANSIENT_LOCAL` on `/targets/*` and `/map` subs | Late subscribers still receive the last value |
