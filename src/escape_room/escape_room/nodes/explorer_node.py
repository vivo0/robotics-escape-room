#!/usr/bin/env python3
"""Mission state machine: explore → fetch the key → drop it on the plate.

Subscribes:
    /map                    (nav_msgs/OccupancyGrid)
    /targets/{cube,plate,door}  (geometry_msgs/PoseStamped, latched)

Publishes:
    /cmd_vel               (geometry_msgs/Twist)
    /exploration/path      (nav_msgs/Path) — A* plan, for RViz
    /exploration/frontiers (geometry_msgs/PoseArray) — frontier centroids

Pose source is the CoppeliaSim ZMQ API (bypasses TF, exact pose every
tick). The state machine:

* ``explore`` — pick frontiers, plan A* toward them.
* ``go_to_key`` — once all three landmark poses are known, plan A*
  directly to the cube. If the corridor still crosses UNKNOWN cells
  fall back to the frontier closest to the cube; retry direct on
  the next replan.
* ``pickup_open / pickup_drive / pickup_close`` — real grasp: drive
  the gripper's prismatic joint open, drive ~10 cm forward to slip
  the cube between the fingers, drive the prismatic joint closed,
  then parent the cube to ``attachPoint`` (the same setObjectParent
  pattern the model's own gripper script uses internally).
* ``go_to_plate`` — same A* logic as ``go_to_key`` but toward the
  pressure plate.
* ``drop_align`` — first rotate in place until the heading points
  at the plate, then drive forward/backward at slow speed until the
  robot is exactly ``plate_drop_distance_m`` from the plate. Without
  this the pure-pursuit follower often arrives near the plate but
  with a heading along the last path segment AND with the inflated
  A* goal sometimes 30-40 cm off-centre, so the gripper would not
  be over the plate when opened.
* ``drop_open`` — open the gripper, detach the cube; gravity drops
  it onto the plate; the door_controller node sees the cube on the
  plate and slides the door open.
* ``drop_backup`` — reverse for a short duration so the gripper
  arms clear the cube before any rotation: without this, when the
  next state forces a yaw change to plan toward the door, the
  open gripper can sweep into the cube and knock it off the plate
  (which would close the door again before the robot can reach it).
* ``go_to_door`` — A* toward a goal slightly outside the door (the
  door position pushed away from room origin) so the robot drives
  through the now-open gap. Same direct/frontier fallback as
  ``go_to_key``.
* ``done`` — robot stops.

The gripper is driven by calling Lua helpers ``_ext_set_target`` /
``_ext_get_state`` injected into the ``gripper_link_respondable``
child script by ``build_scene.py``. The native CoppeliaSim signal
mechanism the model's script uses (``target_gripper#<h>`` /
``gripper#<h>``) is per-Lua-context in modern CoppeliaSim, so writes
from Python ZMQ never reach the script's own ``getInt32Signal``;
the injected helpers monkey-patch the signal API in-place inside
the gripper script, giving us a shared backing store.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from nav_msgs.msg import Path as PathMsg

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from escape_room.exploration import find_frontiers
from escape_room.mapping.occupancy_grid import (FREE, OCCUPIED,
                                                  GridSpec, OccupancyGrid)
from escape_room.planning import PurePursuit, plan_path
from escape_room.planning.pure_pursuit import PurePursuitConfig

# Gripper target/state values mirrored from
# robomaster_msgs/action/GripperControl.action.
GRIPPER_PAUSE = 0
GRIPPER_OPEN = 1
GRIPPER_CLOSE = 2


def yaw_from_pose_matrix(mat12: list) -> float:
    """Yaw from a CoppeliaSim 3x4 row-major pose matrix."""
    R = np.array(mat12, dtype=np.float64).reshape(3, 4)[:, :3]
    return math.atan2(R[1, 0], R[0, 0])


def occupancy_msg_to_grid(msg: OccupancyGridMsg) -> OccupancyGrid:
    """Rebuild an OccupancyGrid from a nav_msgs/OccupancyGrid: re-encode
    {-1, 0, 100} into boolean ``free`` / ``occ`` masks."""
    spec = GridSpec(
        width_m=msg.info.width * msg.info.resolution,
        height_m=msg.info.height * msg.info.resolution,
        resolution=msg.info.resolution,
        origin_x=msg.info.origin.position.x,
        origin_y=msg.info.origin.position.y,
    )
    grid = OccupancyGrid(spec)
    arr = np.array(msg.data, dtype=np.int8).reshape(
        msg.info.height, msg.info.width)
    grid.free[arr == FREE] = True
    grid.occ[arr == OCCUPIED] = True
    return grid


class ExplorerNode(Node):
    def __init__(self) -> None:
        super().__init__('explorer_node')

        # ---- parameters ---------------------------------------------
        self.declare_parameter('map_frame', 'world')
        self.declare_parameter('robot_alias', '/RoboMasterEP/BaseLinkFrame')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate_hz', 4.0)
        self.declare_parameter('replan_period_s', 1.0)
        self.declare_parameter('robot_radius_m', 0.20)
        self.declare_parameter('frontier_min_size', 4)
        self.declare_parameter('arrival_tol_m', 0.30)
        # Looser arrival tol for the plate: A* with inflation often
        # ends ~0.4 m short of the plate centre, so the cube tol
        # must cover that gap to trigger the drop sequence at all.
        # The precise positioning happens in drop_align afterwards.
        self.declare_parameter('plate_arrival_tol_m', 0.55)
        # How far past the door (in metres, away from room origin) to
        # place the exit goal so the robot actually drives through
        # the open gap rather than stopping at the wall.
        self.declare_parameter('door_push_m', 0.6)
        # Drop alignment: rotate to face plate, then park at this
        # distance so the gripper (offset ~0.30 m forward of base)
        # ends up over the plate centre.
        self.declare_parameter('align_yaw_tol_rad', 0.08)   # ~4.6°
        self.declare_parameter('align_kp', 1.5)
        self.declare_parameter('align_max_omega', 0.6)
        self.declare_parameter('plate_drop_distance_m', 0.30)
        self.declare_parameter('plate_drop_dist_tol_m', 0.04)
        self.declare_parameter('park_max_speed_mps', 0.06)
        # After dropping, reverse this far at this speed before
        # planning toward the door. Keeps the gripper clear of the
        # cube during the rotation that pure-pursuit will start.
        self.declare_parameter('drop_backup_speed_mps', 0.05)
        self.declare_parameter('drop_backup_duration_s', 8.0)
        # Slow-creep parameters used by the pickup sequence to slip
        # the cube between the gripper fingers after opening.
        self.declare_parameter('engage_speed_mps', 0.05)
        self.declare_parameter('engage_duration_s', 1.6)
        # Max time we wait for the gripper to reach the target state
        # via the int signal before giving up and advancing the SM.
        self.declare_parameter('gripper_timeout_s', 4.0)
        self.declare_parameter('cube_alias', '/TargetCube')

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.replan_period = float(
            self.get_parameter('replan_period_s').value)
        self.robot_radius = float(
            self.get_parameter('robot_radius_m').value)
        self.frontier_min_size = int(
            self.get_parameter('frontier_min_size').value)
        self._arrival_tol = float(
            self.get_parameter('arrival_tol_m').value)
        self._plate_arrival_tol = float(
            self.get_parameter('plate_arrival_tol_m').value)
        self._door_push = float(
            self.get_parameter('door_push_m').value)
        self._align_yaw_tol = float(
            self.get_parameter('align_yaw_tol_rad').value)
        self._align_kp = float(self.get_parameter('align_kp').value)
        self._align_max_omega = float(
            self.get_parameter('align_max_omega').value)
        self._plate_drop_distance = float(
            self.get_parameter('plate_drop_distance_m').value)
        self._plate_drop_dist_tol = float(
            self.get_parameter('plate_drop_dist_tol_m').value)
        self._park_max_speed = float(
            self.get_parameter('park_max_speed_mps').value)
        self._drop_backup_speed = float(
            self.get_parameter('drop_backup_speed_mps').value)
        self._drop_backup_duration = float(
            self.get_parameter('drop_backup_duration_s').value)
        self._engage_speed = float(
            self.get_parameter('engage_speed_mps').value)
        self._engage_duration = float(
            self.get_parameter('engage_duration_s').value)
        self._gripper_timeout = float(
            self.get_parameter('gripper_timeout_s').value)
        robot_alias = str(self.get_parameter('robot_alias').value)
        cube_alias = str(self.get_parameter('cube_alias').value)

        # ---- sim connection -----------------------------------------
        # Keep the client as a member: anonymous clients can be
        # garbage-collected, dropping the ZMQ connection.
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        try:
            self.robot_handle = self.sim.getObject(robot_alias)
        except Exception as e:
            raise RuntimeError(
                f"could not resolve robot alias '{robot_alias}'; "
                f"run build_scene first ({e})")

        # Gripper / cube handles. The arm + gripper subtree hangs off
        # the model root (e.g. ``/RoboMasterEP``), not off
        # ``BaseLinkFrame`` — BaseLinkFrame is a sibling of
        # ``arm_base_attachment_joint``, so walking its subtree would
        # never find the gripper. Resolve the model root from the
        # first segment of robot_alias and walk from there.
        model_alias = '/' + robot_alias.lstrip('/').split('/')[0]
        try:
            self._model_h = self.sim.getObject(model_alias)
        except Exception as e:
            raise RuntimeError(
                f"could not resolve model alias '{model_alias}' ({e})")
        self._attach_h = self._find_in_robot('attachPoint')
        # The gripper is driven via Lua helpers injected into the
        # gripper_link_respondable child script by build_scene.py
        # (see the module docstring for why signals don't work).
        gripper_link_h = self._find_in_robot('gripper_link_respondable')
        self._gripper_script_h: int | None = None
        if gripper_link_h is not None:
            try:
                sh = self.sim.getScript(1, gripper_link_h)   # 1=childscript
                if sh and sh > 0:
                    self._gripper_script_h = int(sh)
            except Exception as e:
                self.get_logger().warn(
                    f'could not resolve gripper child script: {e}')
        try:
            self._cube_h: int | None = self.sim.getObject(cube_alias)
        except Exception as e:
            self.get_logger().warn(
                f"could not resolve cube alias '{cube_alias}': {e}")
            self._cube_h = None
        self.get_logger().info(
            f'grasp handles: gripper_link={gripper_link_h}, '
            f'gripper_script={self._gripper_script_h}, '
            f'attach={self._attach_h}, cube={self._cube_h}')

        # ---- ROS pub/sub --------------------------------------------
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGridMsg,
            str(self.get_parameter('map_topic').value),
            self._on_map,
            latched_qos,
        )
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self.path_pub = self.create_publisher(
            PathMsg, '/exploration/path', 10)
        self.frontiers_pub = self.create_publisher(
            PoseArray, '/exploration/frontiers', 10)

        # Landmark topics from color_detector_node; we need all three
        # before abandoning exploration.
        self._target_xy: dict[str, tuple[float, float]] = {}
        self._required_targets = ('cube', 'plate', 'door')
        self._target_subs = [
            self.create_subscription(
                PoseStamped, f'/targets/{name}',
                lambda msg, n=name: self._on_target(n, msg),
                latched_qos,
            )
            for name in self._required_targets
        ]

        # ---- state ---------------------------------------------------
        self._latest_map: OccupancyGrid | None = None
        self._planner: PurePursuit | None = None
        self._goal_xy: tuple[float, float] | None = None
        self._last_replan_time: float = 0.0
        self._blacklist: set[tuple[int, int]] = set()
        # explore | go_to_key
        # | pickup_open | pickup_drive | pickup_close
        # | go_to_plate | drop_align | drop_open | drop_backup
        # | go_to_door | done
        self._mode: str = 'explore'
        self._engage_start_time: float = 0.0  # timer for pickup_drive
        self._action_start_time: float = 0.0  # timer for gripper open/close

        # ---- control loop -------------------------------------------
        self.create_timer(
            1.0 / float(self.get_parameter('control_rate_hz').value),
            self._tick,
        )

        self.get_logger().info(
            f'ready. robot_radius={self.robot_radius:.2f} m, '
            f'replan every {self.replan_period:.1f} s')

    def _find_in_robot(self, alias: str) -> int | None:
        """Walk the model's subtree and return the first object whose
        alias matches ``alias`` (case-sensitive). Returns None on
        miss or on any sim API error."""
        try:
            for h in self.sim.getObjectsInTree(self._model_h):
                try:
                    if self.sim.getObjectAlias(int(h), 0) == alias:
                        return int(h)
                except Exception:
                    continue
        except Exception:
            pass
        return None

    # ===== callbacks =====================================================

    def _on_map(self, msg: OccupancyGridMsg) -> None:
        self._latest_map = occupancy_msg_to_grid(msg)

    def _on_target(self, name: str, msg: PoseStamped) -> None:
        if name in self._target_xy:
            return
        self._target_xy[name] = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
        )
        self.get_logger().info(
            f"target '{name}' locked at "
            f"({self._target_xy[name][0]:.2f}, {self._target_xy[name][1]:.2f}) "
            f"[{len(self._target_xy)}/{len(self._required_targets)}]"
        )
        if (self._mode == 'explore'
                and all(t in self._target_xy
                        for t in self._required_targets)):
            self._mode = 'go_to_key'
            self._planner = None  # force a replan toward the key
            self._goal_xy = None
            self.get_logger().info(
                f'all landmarks seen — GO_TO_KEY '
                f'(cube @ {self._target_xy["cube"]})')

    def _tick(self) -> None:
        # Pickup / drop substates run their own logic and bypass A*.
        if self._mode == 'done':
            self._publish_stop()
            return
        if self._mode in ('pickup_open', 'pickup_close', 'drop_open'):
            self._handle_gripper_wait()
            return
        if self._mode == 'pickup_drive':
            self._handle_engage_drive()
            return
        if self._mode == 'drop_align':
            self._handle_drop_align()
            return
        if self._mode == 'drop_backup':
            self._handle_drop_backup()
            return

        # Normal navigation modes: explore / go_to_key / go_to_plate.
        if self._latest_map is None:
            return
        pose = self._lookup_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        if self._mode == 'go_to_key' and self._reached(
                rx, ry, self._target_xy['cube']):
            self._begin_pickup()
            return
        if self._mode == 'go_to_plate':
            d = math.hypot(self._target_xy['plate'][0] - rx,
                           self._target_xy['plate'][1] - ry)
            if d <= self._plate_arrival_tol:
                self._begin_drop()
                return
        if self._mode == 'go_to_door' and self._reached(
                rx, ry, self._door_exit_xy()):
            self.get_logger().info('arrived at the door — escape complete')
            self._mode = 'done'
            self._publish_stop()
            self._planner = None
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._needs_replan(rx, ry, now):
            self._replan(rx, ry)
            self._last_replan_time = now

        if self._planner is None:
            self._publish_stop()
            return

        v, w = self._planner.step(rx, ry, ryaw)
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

    def _reached(self, rx: float, ry: float,
                 xy: tuple[float, float]) -> bool:
        return math.hypot(xy[0] - rx, xy[1] - ry) <= self._arrival_tol

    def _door_exit_xy(self) -> tuple[float, float]:
        """Door target pushed ``door_push_m`` outward from the room
        origin so the robot drives through the now-open gap rather
        than stopping at the wall. Assumes the room is centred at
        (0, 0) — the convention used in build_scene.py."""
        dx, dy = self._target_xy['door']
        norm = math.hypot(dx, dy)
        if norm < 1e-3:
            return dx, dy
        return (dx + self._door_push * dx / norm,
                dy + self._door_push * dy / norm)

    # ===== pickup / drop sequencing ======================================

    def _begin_pickup(self) -> None:
        """Open the gripper, drive forward to engage, close, attach."""
        self._publish_stop()
        self._planner = None
        self._mode = 'pickup_open'
        self._action_start_time = self._now()
        self._set_gripper_target(GRIPPER_OPEN)
        self.get_logger().info('PICKUP: opening gripper')

    def _begin_drop(self) -> None:
        """Rotate to face the plate, then open the gripper. The gripper
        opens in the ``drop_open`` substate; alignment first prevents
        the cube from falling next to the plate when the robot's final
        heading from pure-pursuit is misaligned."""
        self._publish_stop()
        self._planner = None
        self._mode = 'drop_align'
        self.get_logger().info(
            f'DROP: aligning toward plate {self._target_xy["plate"]}')

    def _handle_drop_align(self) -> None:
        """Two-phase parking before drop:
        1. Rotate in place until heading points at the plate.
        2. Drive forward/backward at slow speed until the robot is
           exactly ``plate_drop_distance_m`` from the plate (gripper
           offset), keeping yaw locked on plate.
        Then transition to ``drop_open``."""
        pose = self._lookup_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose
        px, py = self._target_xy['plate']
        dist = math.hypot(px - rx, py - ry)
        target_yaw = math.atan2(py - ry, px - rx)
        yaw_err = math.atan2(math.sin(target_yaw - ryaw),
                             math.cos(target_yaw - ryaw))

        # Phase 1: rotate-to-face (no linear motion until aligned).
        if abs(yaw_err) > self._align_yaw_tol:
            omega = max(-self._align_max_omega,
                        min(self._align_max_omega, self._align_kp * yaw_err))
            twist = Twist()
            twist.angular.z = float(omega)
            self.cmd_pub.publish(twist)
            return

        # Phase 2: distance parking. dist_err > 0 → too far, advance.
        dist_err = dist - self._plate_drop_distance
        if abs(dist_err) <= self._plate_drop_dist_tol:
            self._publish_stop()
            self._mode = 'drop_open'
            self._action_start_time = self._now()
            self._set_gripper_target(GRIPPER_OPEN)
            self.get_logger().info(
                f'DROP: parked at {dist:.2f} m from plate, opening gripper')
            return
        speed = max(-self._park_max_speed,
                    min(self._park_max_speed, 0.5 * dist_err))
        twist = Twist()
        twist.linear.x = float(speed)
        # Keep micro-correcting yaw during parking.
        twist.angular.z = float(0.5 * yaw_err)
        self.cmd_pub.publish(twist)

    def _handle_gripper_wait(self) -> None:
        """Poll the gripper state signal; advance once it reaches the
        target (or on timeout)."""
        self._publish_stop()
        if self._mode == 'pickup_open':
            if self._gripper_reached(GRIPPER_OPEN):
                self._mode = 'pickup_drive'
                self._engage_start_time = self._now()
                self.get_logger().info(
                    f'PICKUP: driving forward {self._engage_duration:.1f} s')
        elif self._mode == 'pickup_close':
            if self._gripper_reached(GRIPPER_CLOSE):
                self._attach_cube()
                self._mode = 'go_to_plate'
                self._planner = None
                self.get_logger().info(
                    f'PICKUP done — GO_TO_PLATE '
                    f'(plate @ {self._target_xy["plate"]})')
        elif self._mode == 'drop_open':
            if self._gripper_reached(GRIPPER_OPEN):
                self._detach_cube()
                self._mode = 'drop_backup'
                self._engage_start_time = self._now()
                self.get_logger().info(
                    f'cube released — backing up '
                    f'{self._drop_backup_duration:.1f} s')

    def _handle_engage_drive(self) -> None:
        """Slow forward creep so the cube slips between the fingers."""
        if (self._now() - self._engage_start_time) >= self._engage_duration:
            self._publish_stop()
            self._mode = 'pickup_close'
            self._action_start_time = self._now()
            self._set_gripper_target(GRIPPER_CLOSE)
            self.get_logger().info('PICKUP: closing gripper')
            return
        twist = Twist()
        twist.linear.x = self._engage_speed
        self.cmd_pub.publish(twist)

    def _handle_drop_backup(self) -> None:
        """Slow reverse so the open gripper clears the cube before
        any rotation. After ``drop_backup_duration_s`` we kick off
        the navigation toward the door."""
        if (self._now() - self._engage_start_time) >= \
                self._drop_backup_duration:
            self._publish_stop()
            self._mode = 'go_to_door'
            self._planner = None  # force replan toward the door
            self.get_logger().info(
                f'backed up — GO_TO_DOOR '
                f'(exit @ {self._door_exit_xy()})')
            return
        twist = Twist()
        twist.linear.x = -self._drop_backup_speed
        self.cmd_pub.publish(twist)

    # ===== sim plumbing for the gripper / cube ===========================

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_gripper_target(self, state: int) -> None:
        """Drive the gripper open / close / pause by calling the Lua
        helper injected into the gripper child script."""
        if self._gripper_script_h is None:
            self.get_logger().warn(
                'gripper script handle unavailable — cannot drive gripper '
                '(was build_scene.py run with the helper injection?)')
            return
        try:
            self.sim.callScriptFunction(
                '_ext_set_target', self._gripper_script_h, int(state))
        except Exception as e:
            self.get_logger().warn(f'_ext_set_target failed: {e}')

    def _gripper_reached(self, target_state: int) -> bool:
        """True if the gripper script reports it at ``target_state`` or
        the timeout elapsed."""
        if (self._now() - self._action_start_time) >= self._gripper_timeout:
            self.get_logger().warn(
                f'gripper timeout waiting for state={target_state}')
            return True
        if self._gripper_script_h is None:
            return True
        try:
            current = self.sim.callScriptFunction(
                '_ext_get_state', self._gripper_script_h)
        except Exception:
            return False
        return current is not None and int(current) == target_state

    def _attach_cube(self) -> None:
        if self._cube_h is None or self._attach_h is None:
            self.get_logger().warn('attach skipped: missing handle')
            return
        try:
            self.sim.setObjectParent(self._cube_h, self._attach_h, True)
            self.sim.resetDynamicObject(self._cube_h)
            self.get_logger().info('cube attached to gripper')
        except Exception as e:
            self.get_logger().warn(f'attach failed: {e}')

    def _detach_cube(self) -> None:
        if self._cube_h is None:
            return
        try:
            self.sim.setObjectParent(self._cube_h, -1, True)
            self.sim.resetDynamicObject(self._cube_h)
        except Exception as e:
            self.get_logger().warn(f'detach failed: {e}')

    # ===== planning ======================================================

    def _needs_replan(self, rx: float, ry: float, now: float) -> bool:
        if self._planner is None:
            return True
        if (now - self._last_replan_time) >= self.replan_period:
            return True
        if self._planner.is_finished((rx, ry)):
            return True
        return False

    def _replan(self, rx: float, ry: float) -> None:
        grid = self._latest_map
        if grid is None:
            return
        # Inflate by robot radius so A* can plan with a point robot.
        inflate_cells = max(
            1, int(math.ceil(self.robot_radius / grid.spec.resolution)))
        inflated = grid.inflate(inflate_cells)

        if self._mode == 'go_to_key':
            self._replan_to_target(
                grid, inflated, rx, ry,
                self._target_xy['cube'], 'GO_TO_KEY')
        elif self._mode == 'go_to_plate':
            self._replan_to_target(
                grid, inflated, rx, ry,
                self._target_xy['plate'], 'GO_TO_PLATE')
        elif self._mode == 'go_to_door':
            self._replan_to_target(
                grid, inflated, rx, ry,
                self._door_exit_xy(), 'GO_TO_DOOR')
        else:
            self._replan_frontier(grid, inflated, rx, ry)

    def _replan_to_target(self, grid: OccupancyGrid,
                          inflated: OccupancyGrid,
                          rx: float, ry: float,
                          target_xy: tuple[float, float],
                          label: str) -> None:
        """A* directly to ``target_xy``. Falls back to the frontier
        closest to the target when the corridor still crosses UNKNOWN
        (which A* treats as blocked); next replan retries direct."""
        # 1) Direct: try the target pose, then a free ring around it.
        for goal_xy in [target_xy] + self._nearby_free_world(
                inflated, target_xy):
            path = plan_path(inflated, (rx, ry), goal_xy)
            if path is not None and len(path) >= 2:
                self._adopt_path(path, goal_xy)
                self.get_logger().info(
                    f'{label}: direct path to '
                    f'({goal_xy[0]:.2f}, {goal_xy[1]:.2f})')
                return

        # 2) Fall back to the closest frontier to the target.
        frontiers = find_frontiers(grid, min_size=self.frontier_min_size)
        self._publish_frontiers(frontiers)
        scored = sorted(
            ((math.hypot(f.centroid_xy[0] - target_xy[0],
                         f.centroid_xy[1] - target_xy[1]), f)
             for f in frontiers
             if self._is_frontier_eligible(inflated, f)),
            key=lambda x: x[0])

        for d_to_target, f in scored:
            path = plan_path(inflated, (rx, ry), f.centroid_xy)
            if path is None or len(path) < 2:
                self._blacklist.add(inflated.world_to_grid(*f.centroid_xy))
                continue
            self._adopt_path(path, f.centroid_xy)
            self.get_logger().info(
                f'{label}: via frontier @ '
                f'({f.centroid_xy[0]:.2f}, {f.centroid_xy[1]:.2f}) '
                f'(d={d_to_target:.2f} m)')
            return

        self.get_logger().warn(
            f'{label}: no path to target and no reachable frontier')
        self._planner = None

    def _replan_frontier(self, grid: OccupancyGrid,
                         inflated: OccupancyGrid,
                         rx: float, ry: float) -> None:
        frontiers = find_frontiers(grid, min_size=self.frontier_min_size)
        self._publish_frontiers(frontiers)
        if not frontiers:
            self.get_logger().info('exploration complete')
            self._mode = 'done'
            self._planner = None
            return

        # Score = size / distance: prefer big nearby frontiers.
        scored: list[tuple[float, object]] = []
        for f in frontiers:
            if not self._is_frontier_eligible(inflated, f):
                continue
            d = math.hypot(f.centroid_xy[0] - rx, f.centroid_xy[1] - ry)
            if d < 1e-3:
                continue
            scored.append((f.size / d, f))
        scored.sort(reverse=True)

        for score, f in scored:
            path = plan_path(inflated, (rx, ry), f.centroid_xy)
            if path is None or len(path) < 2:
                self._blacklist.add(inflated.world_to_grid(*f.centroid_xy))
                continue
            self._adopt_path(path, f.centroid_xy)
            self.get_logger().info(
                f'heading to frontier @ ({f.centroid_xy[0]:.2f}, '
                f'{f.centroid_xy[1]:.2f}) [size={f.size}, score={score:.2f}]')
            return

        self.get_logger().warn('no reachable frontier this tick')
        self._planner = None

    def _is_frontier_eligible(self, inflated: OccupancyGrid, f) -> bool:
        cc, cr = inflated.world_to_grid(*f.centroid_xy)
        return ((cc, cr) not in self._blacklist
                and inflated.is_traversable(cc, cr))

    def _adopt_path(self, path: list[tuple[float, float]],
                    goal_xy: tuple[float, float]) -> None:
        self._planner = PurePursuit(path, PurePursuitConfig())
        self._goal_xy = goal_xy
        self._publish_path(path)

    def _nearby_free_world(self, grid: OccupancyGrid,
                           xy: tuple[float, float]
                           ) -> list[tuple[float, float]]:
        """Free-cell world coords in a small ring around ``xy`` in the
        inflated grid, ordered by distance."""
        cc, cr = grid.world_to_grid(*xy)
        res = grid.spec.resolution
        ox, oy = grid.spec.origin_x, grid.spec.origin_y
        out: list[tuple[float, tuple[float, float]]] = []
        for dr in range(-4, 5):
            for dc in range(-4, 5):
                if dc == 0 and dr == 0:
                    continue
                nc, nr = cc + dc, cr + dr
                if not grid.in_bounds(nc, nr):
                    continue
                if not grid.is_traversable(nc, nr):
                    continue
                wx = ox + (nc + 0.5) * res
                wy = oy + (nr + 0.5) * res
                out.append((math.hypot(wx - xy[0], wy - xy[1]), (wx, wy)))
        out.sort()
        return [w for _, w in out]

    # ===== helpers =======================================================

    def _lookup_pose(self) -> tuple[float, float, float] | None:
        try:
            mat = self.sim.getObjectMatrix(self.robot_handle, -1)
        except Exception:
            return None
        return float(mat[3]), float(mat[7]), yaw_from_pose_matrix(mat)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())  # all zeros

    def _publish_path(self, path: list[tuple[float, float]]) -> None:
        msg = PathMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for x, y in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _publish_frontiers(self, frontiers) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for f in frontiers:
            p = Pose()
            p.position.x = float(f.centroid_xy[0])
            p.position.y = float(f.centroid_xy[1])
            p.orientation.w = 1.0
            msg.poses.append(p)
        self.frontiers_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # final stop
        rclpy.shutdown()


if __name__ == '__main__':
    main()
