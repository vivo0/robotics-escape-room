"""Dispatcher for the three gripper-wait states (pickup_open/close, drop_open)."""

from .gripper_io import GRIPPER_CLOSE, GRIPPER_OPEN


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
        node.action_t = node.clock_s()
