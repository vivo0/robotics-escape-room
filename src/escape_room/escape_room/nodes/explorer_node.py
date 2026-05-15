#!/usr/bin/env python3
"""Mission state machine for the escape room.

Navigation is delegated to Nav2 (NavigateToPose action).  Exploration uses
predefined boustrophedon waypoints instead of custom frontier detection.
Robot pose is read from the map→base_link TF published by slam_toolbox.

Gripper control still uses the CoppeliaSim ZMQ API (Lua helpers injected by
build_scene.py), as does cube attach/detach.

State sequence:

    explore
      → go_to_key
      → pickup_open → pickup_drive → pickup_close
      → go_to_plate
      → drop_align → drop_open → drop_backup
      → go_to_door
      → done
"""
from __future__ import annotations

import math

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# Gripper state values mirror robomaster_msgs/action/GripperControl.
GRIPPER_OPEN  = 1
GRIPPER_CLOSE = 2

# Default boustrophedon coverage for easy.json (5×4 m room, walls at ±2.5/±2).
_DEFAULT_WAYPOINTS = [
    (-1.5, -1.5), ( 0.0, -1.5), ( 1.5, -1.5),
    ( 1.5,  0.0), ( 0.0,  0.0), (-1.5,  0.0),
    (-1.5,  1.5), ( 0.0,  1.5), ( 1.5,  1.5),
]


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class ExplorerNode(Node):

    def __init__(self) -> None:
        super().__init__('explorer_node')

        # ----- parameters --------------------------------------------
        p = self.declare_parameter
        p('robot_alias',            '/RoboMasterEP/BaseLinkFrame')
        p('cube_alias',             '/TargetCube')
        p('map_frame',              'map')
        p('base_frame',             'base_link')
        p('control_rate_hz',        4.0)
        p('robot_radius_m',         0.20)
        p('arrival_tol_m',          0.30)
        p('plate_arrival_tol_m',    0.55)
        p('door_push_m',            0.6)
        p('engage_speed_mps',       0.05)
        p('engage_duration_s',      1.6)
        p('drop_backup_speed_mps',  0.05)
        p('drop_backup_duration_s', 8.0)
        p('plate_drop_distance_m',  0.30)
        p('plate_drop_dist_tol_m',  0.04)
        p('park_max_speed_mps',     0.06)
        p('align_yaw_tol_rad',      0.08)
        p('align_kp',               1.5)
        p('align_max_omega',        0.6)
        p('gripper_timeout_s',      4.0)

        g = lambda n: self.get_parameter(n).value
        self._map_frame        = g('map_frame')
        self._base_frame       = g('base_frame')
        self.arrival_tol       = float(g('arrival_tol_m'))
        self.plate_arrival_tol = float(g('plate_arrival_tol_m'))
        self.door_push         = float(g('door_push_m'))
        self.engage_speed      = float(g('engage_speed_mps'))
        self.engage_duration   = float(g('engage_duration_s'))
        self.backup_speed      = float(g('drop_backup_speed_mps'))
        self.backup_duration   = float(g('drop_backup_duration_s'))
        self.drop_distance     = float(g('plate_drop_distance_m'))
        self.drop_dist_tol     = float(g('plate_drop_dist_tol_m'))
        self.park_max_speed    = float(g('park_max_speed_mps'))
        self.align_yaw_tol     = float(g('align_yaw_tol_rad'))
        self.align_kp          = float(g('align_kp'))
        self.align_max_omega   = float(g('align_max_omega'))
        self.gripper_timeout   = float(g('gripper_timeout_s'))

        # ----- ZMQ (gripper + cube attach only) ----------------------
        self._client = RemoteAPIClient()
        self._sim    = self._client.require('sim')

        robot_alias = g('robot_alias')
        model_alias = '/' + robot_alias.lstrip('/').split('/')[0]
        self._robot_h = self._sim.getObject(robot_alias)
        model_h       = self._sim.getObject(model_alias)
        self._cube_h  = self._sim.getObject(g('cube_alias'))
        self._attach_h      = self._find_in_tree(model_h, 'attachPoint')
        gripper_link_h      = self._find_in_tree(model_h, 'gripper_link_respondable')
        self._gripper_script_h = self._sim.getScript(1, gripper_link_h)

        # ----- TF (robot pose in map frame) --------------------------
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ----- Nav2 action client ------------------------------------
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_goal_active  = False
        self._nav_goal_handle  = None

        # ----- ROS pub/sub -------------------------------------------
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(PathMsg, '/exploration/path', 10)
        for name in ('cube', 'plate', 'door'):
            self.create_subscription(
                PoseStamped, f'/targets/{name}',
                lambda m, n=name: self._on_target(n, m), latched)

        # ----- mission state -----------------------------------------
        self.targets: dict[str, tuple[float, float]] = {}
        self.mode      = 'explore'
        self.action_t  = 0.0
        self._coverage_wps = list(_DEFAULT_WAYPOINTS)
        self._wp_idx       = 0

        self._handlers = {
            'explore':      self._tick_navigate,
            'go_to_key':    self._tick_navigate,
            'go_to_plate':  self._tick_navigate,
            'go_to_door':   self._tick_navigate,
            'pickup_open':  self._tick_gripper_wait,
            'pickup_close': self._tick_gripper_wait,
            'drop_open':    self._tick_gripper_wait,
            'pickup_drive': self._tick_engage,
            'drop_align':   self._tick_drop_align,
            'drop_backup':  self._tick_drop_backup,
            'done':         self._stop,
        }

        self.create_timer(1.0 / float(g('control_rate_hz')), self._tick)
        self.get_logger().info('ready; waiting for Nav2 and slam_toolbox...')

    # ===== handle resolution =========================================

    def _find_in_tree(self, root_h: int, alias: str) -> int:
        for h in self._sim.getObjectsInTree(root_h):
            if self._sim.getObjectAlias(int(h), 0) == alias:
                return int(h)
        raise RuntimeError(f'object not found in robot tree: {alias}')

    # ===== ROS callbacks =============================================

    def _on_target(self, name: str, msg: PoseStamped) -> None:
        if name in self.targets:
            return
        self.targets[name] = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f"saw '{name}' at ({self.targets[name][0]:.2f}, "
            f"{self.targets[name][1]:.2f}) [{len(self.targets)}/3]")
        if self.mode == 'explore' and len(self.targets) == 3:
            self._enter('go_to_key')

    # ===== main loop =================================================

    def _tick(self) -> None:
        self._handlers[self.mode]()

    # --- navigation tick (explore / go_to_key / go_to_plate / go_to_door) --

    def _tick_navigate(self) -> None:
        if self._nav_goal_active:
            return

        if self.mode == 'explore':
            if len(self.targets) == 3:
                self._enter('go_to_key')
                return
            self._send_next_waypoint()
        elif self.mode == 'go_to_key':
            self._begin_pickup()
        elif self.mode == 'go_to_plate':
            self._begin_drop()
        elif self.mode == 'go_to_door':
            self._enter('done')
            self._stop()

    # --- Nav2 goal management ----------------------------------------

    def _send_nav_goal(self, x: float, y: float, yaw: float = 0.0) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('Nav2 action server not available yet')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        self._nav_goal_active = True
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_accepted)

        self._publish_nav_goal_path(x, y)

    def _on_goal_accepted(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected goal')
            self._nav_goal_active = False
            return
        self._nav_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        status = future.result().status
        self._nav_goal_active = False
        self._nav_goal_handle = None
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Nav2 goal finished with status {status}')

    def _cancel_nav(self) -> None:
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
        self._nav_goal_active = False
        self._nav_goal_handle = None

    def _send_next_waypoint(self) -> None:
        if self._wp_idx >= len(self._coverage_wps):
            self._wp_idx = 0
        x, y = self._coverage_wps[self._wp_idx]
        self._wp_idx += 1
        self._send_nav_goal(x, y)

    # --- pickup substates --------------------------------------------

    def _begin_pickup(self) -> None:
        self._stop()
        self._enter('pickup_open')
        self.action_t = self._clock_s()
        self._set_gripper(GRIPPER_OPEN)

    def _tick_engage(self) -> None:
        if self._clock_s() - self.action_t >= self.engage_duration:
            self._stop()
            self._enter('pickup_close')
            self.action_t = self._clock_s()
            self._set_gripper(GRIPPER_CLOSE)
            return
        twist = Twist()
        twist.linear.x = self.engage_speed
        self.cmd_pub.publish(twist)

    # --- drop substates ----------------------------------------------

    def _begin_drop(self) -> None:
        self._stop()
        self._enter('drop_align')

    def _tick_drop_align(self) -> None:
        pose = self._get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose
        px, py = self.targets['plate']
        dist       = math.hypot(px - rx, py - ry)
        target_yaw = math.atan2(py - ry, px - rx)
        yaw_err    = wrap_angle(target_yaw - ryaw)

        if abs(yaw_err) > self.align_yaw_tol:
            twist = Twist()
            twist.angular.z = clamp(
                self.align_kp * yaw_err,
                -self.align_max_omega, self.align_max_omega)
            self.cmd_pub.publish(twist)
            return

        dist_err = dist - self.drop_distance
        if abs(dist_err) <= self.drop_dist_tol:
            self._stop()
            self._enter('drop_open')
            self.action_t = self._clock_s()
            self._set_gripper(GRIPPER_OPEN)
            return
        twist = Twist()
        twist.linear.x  = clamp(
            0.5 * dist_err, -self.park_max_speed, self.park_max_speed)
        twist.angular.z = 0.5 * yaw_err
        self.cmd_pub.publish(twist)

    def _tick_drop_backup(self) -> None:
        if self._clock_s() - self.action_t >= self.backup_duration:
            self._stop()
            self._enter('go_to_door')
            return
        twist = Twist()
        twist.linear.x = -self.backup_speed
        self.cmd_pub.publish(twist)

    # --- gripper state-wait (pickup_open / pickup_close / drop_open) -

    def _tick_gripper_wait(self) -> None:
        self._stop()
        if self.mode == 'pickup_open' and self._gripper_reached(GRIPPER_OPEN):
            self._enter('pickup_drive')
            self.action_t = self._clock_s()
        elif self.mode == 'pickup_close' and self._gripper_reached(GRIPPER_CLOSE):
            self._attach_cube()
            self._enter('go_to_plate')
        elif self.mode == 'drop_open' and self._gripper_reached(GRIPPER_OPEN):
            self._detach_cube()
            self._enter('drop_backup')
            self.action_t = self._clock_s()

    # ===== sim plumbing ==============================================

    def _set_gripper(self, state: int) -> None:
        self._sim.callScriptFunction(
            '_ext_set_target', self._gripper_script_h, int(state))

    def _gripper_reached(self, target_state: int) -> bool:
        if self._clock_s() - self.action_t >= self.gripper_timeout:
            self.get_logger().warn(
                f'gripper timeout waiting for {target_state}')
            return True
        cur = self._sim.callScriptFunction(
            '_ext_get_state', self._gripper_script_h)
        return cur is not None and int(cur) == target_state

    def _attach_cube(self) -> None:
        self._sim.setObjectParent(self._cube_h, self._attach_h, True)
        self._sim.resetDynamicObject(self._cube_h)

    def _detach_cube(self) -> None:
        self._sim.setObjectParent(self._cube_h, -1, True)
        self._sim.resetDynamicObject(self._cube_h)

    def _get_robot_pose(self) -> tuple[float, float, float] | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame,
                Time(), timeout=Duration(seconds=0.1))
            x   = tf.transform.translation.x
            y   = tf.transform.translation.y
            q   = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
            return float(x), float(y), float(yaw)
        except Exception:
            return None

    def _clock_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ===== geometric helpers =========================================

    def _door_exit_xy(self) -> tuple[float, float]:
        dx, dy = self.targets['door']
        n = math.hypot(dx, dy)
        if n < 1e-3:
            return dx, dy
        return dx + self.door_push * dx / n, dy + self.door_push * dy / n

    # ===== state transitions / publishing ============================

    def _enter(self, mode: str) -> None:
        self.get_logger().info(f'mode: {self.mode} → {mode}')
        self._cancel_nav()
        self.mode = mode
        if mode == 'go_to_key':
            self._send_nav_goal(*self.targets['cube'])
        elif mode == 'go_to_plate':
            self._send_nav_goal(*self.targets['plate'])
        elif mode == 'go_to_door':
            self._send_nav_goal(*self._door_exit_xy())
        elif mode == 'explore':
            self._wp_idx = 0

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _publish_nav_goal_path(self, x: float, y: float) -> None:
        msg = PathMsg()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        msg.poses = [ps]
        self.path_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ExplorerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.cmd_pub.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
