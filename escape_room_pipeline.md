# Escape Room Pipeline — RoboMaster EP

## Goal

The robot is locked in a room with obstacles. It must find a **key** (magenta cylinder), carry it
to a green **pressure plate**, and drop it there. Mission ends when the cube lands on the plate —
the robot does not need to exit through the door.

## Two-phase architecture

**Phase 1 — Discovery.** The robot explores the room using frontier-based exploration (Nav2 +
slam_toolbox), builds a live 2D SLAM map, and records each of the three landmark positions (cube,
plate, door) as soon as it sees them via the camera.

**Phase 2 — Execution.** Once all three landmarks are known the robot stops exploring and uses the
SLAM map to: navigate to the cube (Nav2), pick it up with visual servoing + odometry lunge,
navigate to the plate (Nav2), and deposit the cube with P-control on bearing and distance.

## Tech stack

```
CoppeliaSim (physics + ZMQ)
  ├── lidar_sensor.lua  ──── /scan (LaserScan 10 Hz) ──► slam_toolbox → /map + map→odom TF
  ├── robomaster_sim    ──── /odom + odom→base_link TF ──► Nav2
  └── ZMQ ◄── Python (gripper open/close, cube lidar-visibility)

ROS 2 nodes
  ├── color_detector_node  ── /targets/{cube,plate,door} (latched, map frame)
  │                        ── /targets/cube_live (base_link, every frame)
  └── explorer_node        ── NavigateToPose → Nav2
                           ── /cmd_vel (short-range manoeuvres)
```

- **CoppeliaSim 4.x** + `simExtROS2` plugin
- **ROS 2** + `robomaster_ros` driver
- **Python**: `rclpy`, `opencv-python`, `numpy`, `tf2_ros`, `cv_bridge`
- **Nav2**: NavFn planner + Regulated Pure Pursuit + BT navigator
- **slam_toolbox**: online async SLAM, loop closure disabled, 5 cm/cell
- **RViz2**: map, scan, costmaps, path visualisation

## ROS nodes

| Node | Responsibility | Subscribes | Publishes |
|---|---|---|---|
| `color_detector_node` | HSV blob → pose estimate **from camera only** (monocular depth or floor ray) + TF | `/camera/image_color` | `/targets/{cube,plate,door}` (latched), `/targets/cube_live`, `/targets/markers` |
| `explorer_node` | Mission state machine | `/targets/*`, `/map`, `/targets/cube_live` | `NavigateToPose` (action), `/cmd_vel` |

`slam_toolbox` and `Nav2` are external packages launched by `discovery.launch.py`.

## Perception (color_detector_node)

All **perception-only** — no object positions are read from the simulator.

- **Cube / door** (upright, known height): monocular depth — `z = f_pix × real_height / pixel_height`. Back-project centroid to that depth → 3D point in `camera_optical_link` → TF → `map`. Skipped when blob touches image edge (clipped height → wrong depth).
- **Plate** (flat, no usable height): ray through the centroid pixel, intersected with floor plane `z = 0` in map frame.
- Camera FOV read from sim **once at init** (scalar constant for focal-length calculation; not a pose read).
- Poses refined every frame while visible; latched on `/targets/*` so late subscribers still receive the last value.
- Live cube stream on `/targets/cube_live` in `base_link` for visual-servoing during pickup.

## State machine (explorer_node)

```
EXPLORE ──────────► all 3 landmarks seen ────────────► GO_TO_KEY
GO_TO_KEY ────────► Nav2 reached 0.9 m standoff ─────► PICKUP_OPEN
PICKUP_OPEN ──────► gripper fully open ──────────────► PICKUP_ALIGN
PICKUP_ALIGN ─────► visual servo + odometry lunge ───► PICKUP_CLOSE
PICKUP_CLOSE ─────► gripper closed, cube hidden ──────► GO_TO_PLATE
GO_TO_PLATE ──────► Nav2 reached plate ──────────────► DROP_ALIGN
DROP_ALIGN ───────► P-control on yaw + distance ──────► DROP_OPEN
DROP_OPEN ────────► gripper open, cube visible ───────► DROP_BACKUP
DROP_BACKUP ──────► 0.25 m reverse (odometry) ────────► DONE ✓
```

## Pickup details

**Long-range approach**: Nav2 drives the robot to a standoff point 0.9 m from the cube, facing it.

**Visual servoing** (PICKUP_ALIGN): P-controller on bearing from `/targets/cube_live` (base_link):
1. Rotate until cube is centred in the image.
2. Drive forward while keeping it centred.
3. At ≤ 0.9 m the cylinder base starts clipping out of the camera's bottom edge — monocular depth becomes unreliable.

**Odometry lunge**: at 0.9 m the remaining distance is frozen and the robot advances open-loop
measured by odometry (`odom` frame — smooth, no SLAM jumps). A 3 cm margin prevents nudging the
cylinder over.

## Drop details

**drop_align**: P-control on yaw error and distance error. Target distance = `engage_dist + lunge_margin`
(≈ 0.207 m) — the position of the cube held in the gripper ahead of `base_link`. The cube lands
at the plate centre.

**drop_backup**: straight reverse of 0.25 m (odometry-measured) before turning, so the open
gripper does not sweep the cube off the plate.

## Data persistence

- **Map**: slam_toolbox, published on `/map` (TRANSIENT_LOCAL).
- **Landmark poses**: `PoseStamped` with `TRANSIENT_LOCAL` QoS on `/targets/*` — late subscribers always receive the last value.
- **FSM state**: in-memory variable inside `explorer_node`.

## Colour conventions

| Landmark | Colour | OpenCV HSV hue (~) |
|---|---|---|
| Cube (key) | Magenta | 140–170 |
| Pressure plate | Green | 40–80 |
| Door | Blue | 100–130 |

Scenario JSON `color` fields must match these ranges. The cube avoids red (floor/wall artefacts)
and yellow (CoppeliaSim default floor pattern); magenta is the safe, distinct hue.
