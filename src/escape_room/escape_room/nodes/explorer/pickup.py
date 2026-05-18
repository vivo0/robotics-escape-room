"""Pickup phase: align to cube and close gripper."""

import math

from geometry_msgs.msg import Twist

from .utils import clamp, wrap_angle


def tick_pickup_align(node) -> None:
    """P-controller: face cube, approach to pickup_engage_dist, then close."""
    pose = node.get_robot_pose()
    if pose is None:
        return
    rx, ry, ryaw = pose
    cx, cy = node.targets["cube"]
    dist = math.hypot(cx - rx, cy - ry)
    target_yaw = math.atan2(cy - ry, cx - rx)
    yaw_err = wrap_angle(target_yaw - ryaw)

    if abs(yaw_err) > node.align_yaw_tol:
        twist = Twist()
        twist.angular.z = clamp(
            node.align_kp * yaw_err, -node.align_max_omega, node.align_max_omega
        )
        node.cmd_pub.publish(twist)
        return

    dist_err = dist - node.pickup_engage_dist
    if abs(dist_err) <= node.pickup_engage_dist_tol:
        node.stop()
        node.enter("pickup_close")
        node.action_t = node.clock_s()
        node.gripper.close()
        return
    twist = Twist()
    twist.linear.x = clamp(
        0.5 * dist_err, -node.park_max_speed, node.park_max_speed
    )
    twist.angular.z = 0.5 * yaw_err
    node.cmd_pub.publish(twist)
