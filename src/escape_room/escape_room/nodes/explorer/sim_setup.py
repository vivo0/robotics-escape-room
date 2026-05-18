"""CoppeliaSim ZMQ setup: gripper handles + engage distance auto-detect."""

import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from .gripper_io import GripperIO


def setup_sim(node) -> GripperIO:
    """Resolve sim handles + read pose-independent geometry from the scene:

      - ``node.pickup_engage_dist``: attachPoint x-offset from BaseLinkFrame
        (the grasp point centred between the open fingers).
      - ``node.door_normal``: outward unit normal of the door panel, derived
        from its bbox + world position. Always cardinal because doors are
        axis-aligned cuboids on a wall.
      - ``node.sim_targets``: cube/plate/door positions in SLAM map frame,
        computed from sim ground truth at startup. Pre-populates targets so
        the FSM does not need to wait for color_detector.

    Returns the GripperIO for gripper open/close + cube-lidar visibility.
    """
    sim = RemoteAPIClient().require("sim")
    model_alias = "/" + node.robot_alias.lstrip("/").split("/")[0]
    model_h = sim.getObject(model_alias)
    gripper_h = _find_in_tree(sim, model_h, "gripper_link_respondable")
    script_h = sim.getScript(1, gripper_h)
    cube_h = sim.getObject(node.cube_alias)
    gripper = GripperIO(sim, script_h, cube_h)

    attach_h = _find_in_tree(sim, model_h, "attachPoint")
    base_h = _find_in_tree(sim, model_h, "BaseLinkFrame")
    off = sim.getObjectPosition(attach_h, base_h)
    node.pickup_engage_dist = float(off[0])

    door_h = sim.getObject("/Door_0")
    bbox_dx = sim.getObjectFloatParam(
        door_h, sim.objfloatparam_objbbox_max_x
    ) - sim.getObjectFloatParam(door_h, sim.objfloatparam_objbbox_min_x)
    bbox_dy = sim.getObjectFloatParam(
        door_h, sim.objfloatparam_objbbox_max_y
    ) - sim.getObjectFloatParam(door_h, sim.objfloatparam_objbbox_min_y)
    door_pos = sim.getObjectPosition(door_h, -1)
    if bbox_dx < bbox_dy:
        node.door_normal = (1.0 if door_pos[0] > 0 else -1.0, 0.0)
    else:
        node.door_normal = (0.0, 1.0 if door_pos[1] > 0 else -1.0)

    node.get_logger().info(
        f"pickup_engage_dist = {node.pickup_engage_dist:.3f} m; "
        f"door_normal = ({node.door_normal[0]:+.0f}, {node.door_normal[1]:+.0f})"
    )

    # --- sim ground-truth → SLAM map frame --------------------------------
    # SLAM initialises with robot at map (0, 0, yaw=0), so converting sim
    # world positions to the robot's base_link at startup gives map coords.
    r_pos = sim.getObjectPosition(base_h, -1)
    r_q = sim.getObjectQuaternion(base_h, -1)
    rx, ry, rz, rw = r_q
    r_yaw = math.atan2(2.0 * (rw * rz + rx * ry), 1.0 - 2.0 * (ry * ry + rz * rz))
    c, s = math.cos(-r_yaw), math.sin(-r_yaw)

    def _to_map(h: int) -> tuple[float, float]:
        p = sim.getObjectPosition(h, -1)
        dx, dy = p[0] - r_pos[0], p[1] - r_pos[1]
        return (c * dx - s * dy, s * dx + c * dy)

    plate_h = sim.getObject("/PressurePlate")
    node.sim_targets = {
        "cube":  _to_map(cube_h),
        "plate": _to_map(plate_h),
        "door":  _to_map(door_h),
    }
    for name, (mx, my) in node.sim_targets.items():
        node.get_logger().info(f"sim-truth {name}: map ({mx:.2f}, {my:.2f})")

    return gripper


def _find_in_tree(sim, root_h: int, alias: str) -> int:
    for h in sim.getObjectsInTree(root_h):
        if sim.getObjectAlias(int(h), 0) == alias:
            return int(h)
    raise RuntimeError(f"object not found in robot tree: {alias}")
