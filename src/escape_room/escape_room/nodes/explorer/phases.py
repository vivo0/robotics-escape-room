"""FSM phase tick functions: gripper-wait, pickup-align, drop-align/backup, exit-drive."""

import math

from geometry_msgs.msg import Twist

from .sim import GRIPPER_CLOSE, GRIPPER_OPEN


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _tick_align(
    node,
    target_xy: tuple[float, float],
    engage_dist: float,
    dist_tol: float,
    next_state: str,
) -> None:
    """P-controller: rotate to face target, drive to engage_dist, enter next_state."""
    pose = node.get_robot_pose()
    if pose is None:
        return
    rx, ry, ryaw = pose
    tx, ty = target_xy
    dist = math.hypot(tx - rx, ty - ry)
    target_yaw = math.atan2(ty - ry, tx - rx)
    yaw_err = _wrap(target_yaw - ryaw)

    if abs(yaw_err) > node.align_yaw_tol:
        twist = Twist()
        twist.angular.z = _clamp(
            node.align_kp * yaw_err, -node.align_max_omega, node.align_max_omega
        )
        node.cmd_pub.publish(twist)
        return

    dist_err = dist - engage_dist
    if abs(dist_err) <= dist_tol:
        node.stop()
        node.enter(next_state)
        return

    twist = Twist()
    twist.linear.x = _clamp(0.5 * dist_err, -node.park_max_speed, node.park_max_speed)
    twist.angular.z = 0.5 * yaw_err
    node.cmd_pub.publish(twist)


def tick_gripper_wait(node) -> None:
    node.stop()
    elapsed = node.clock_s() - node.action_t
    logger = node.get_logger()
    if node.mode == "pickup_open" and node.gripper.reached(
        GRIPPER_OPEN, elapsed, node.gripper_timeout, logger
    ):
        node.enter("pickup_align")
    elif node.mode == "pickup_close" and node.gripper.reached(
        GRIPPER_CLOSE, elapsed, node.gripper_timeout, logger
    ):
        node.gripper.hide_cube_from_lidar()
        node.enter("go_to_plate")
    elif node.mode == "drop_open" and node.gripper.reached(
        GRIPPER_OPEN, elapsed, node.gripper_timeout, logger
    ):
        node.gripper.show_cube_to_lidar()
        node.enter("drop_backup")


def tick_pickup_align(node) -> None:
    """P-controller: face cube, approach to pickup_engage_dist, then close gripper."""
    _tick_align(
        node,
        target_xy=node.targets["cube"],
        engage_dist=node.pickup_engage_dist,
        dist_tol=node.pickup_engage_dist_tol,
        next_state="pickup_close",
    )


def tick_drop_align(node) -> None:
    """P-controller: face plate, approach to drop_distance, then open gripper."""
    _tick_align(
        node,
        target_xy=node.targets["plate"],
        engage_dist=node.drop_distance,
        dist_tol=node.drop_dist_tol,
        next_state="drop_open",
    )


def tick_drop_backup(node) -> None:
    if node.clock_s() - node.action_t >= node.backup_duration:
        node.stop()
        node.enter("go_to_door")
        return
    twist = Twist()
    twist.linear.x = -node.backup_speed
    node.cmd_pub.publish(twist)


def tick_exit_drive(node) -> None:
    if node.clock_s() - node.action_t >= node.exit_drive_duration:
        node.stop()
        node.enter("done")
        return
    twist = Twist()
    twist.linear.x = node.exit_drive_speed
    node.cmd_pub.publish(twist)
