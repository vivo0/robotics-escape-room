"""Drop phase: align to plate, open gripper, back up."""

import math

from geometry_msgs.msg import Twist

from .utils import clamp, wrap_angle


def tick_drop_align(node) -> None:
    """P-controller: face plate, approach to drop_distance, then open."""
    rx, ry, ryaw = node.get_robot_pose()
    px, py = node.targets["plate"]
    dist = math.hypot(px - rx, py - ry)
    target_yaw = math.atan2(py - ry, px - rx)
    yaw_err = wrap_angle(target_yaw - ryaw)

    if abs(yaw_err) > node.align_yaw_tol:
        twist = Twist()
        twist.angular.z = clamp(
            node.align_kp * yaw_err, -node.align_max_omega, node.align_max_omega
        )
        node.cmd_pub.publish(twist)
        return

    dist_err = dist - node.drop_distance
    if abs(dist_err) <= node.drop_dist_tol:
        node.stop()
        node.enter("drop_open")
        node.action_t = node.clock_s()
        node.gripper.open()
        return
    twist = Twist()
    twist.linear.x = clamp(
        0.5 * dist_err, -node.park_max_speed, node.park_max_speed
    )
    twist.angular.z = 0.5 * yaw_err
    node.cmd_pub.publish(twist)


def tick_drop_backup(node) -> None:
    if node.clock_s() - node.action_t >= node.backup_duration:
        node.stop()
        node.enter("go_to_door")
        return
    twist = Twist()
    twist.linear.x = -node.backup_speed
    node.cmd_pub.publish(twist)
