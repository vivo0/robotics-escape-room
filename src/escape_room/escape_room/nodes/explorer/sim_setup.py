"""CoppeliaSim ZMQ setup: gripper handles + engage distance auto-detect."""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from .gripper_io import GripperIO


def setup_sim(node) -> GripperIO:
    """Resolve sim handles + read pose-independent geometry from the scene:

      - ``node.pickup_engage_dist``: attachPoint x-offset from BaseLinkFrame
        (the grasp point centred between the open fingers).
      - ``node.door_normal``: outward unit normal of the door panel, derived
        from its bbox + world position. Always cardinal because doors are
        axis-aligned cuboids on a wall.

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
    # Thinnest XY axis = wall-normal direction; sign comes from which
    # side of the world origin the door panel sits.
    if bbox_dx < bbox_dy:
        node.door_normal = (1.0 if door_pos[0] > 0 else -1.0, 0.0)
    else:
        node.door_normal = (0.0, 1.0 if door_pos[1] > 0 else -1.0)

    node.get_logger().info(
        f"pickup_engage_dist = {node.pickup_engage_dist:.3f} m; "
        f"door_normal = ({node.door_normal[0]:+.0f}, {node.door_normal[1]:+.0f})"
    )

    return gripper


def _find_in_tree(sim, root_h: int, alias: str) -> int:
    for h in sim.getObjectsInTree(root_h):
        if sim.getObjectAlias(int(h), 0) == alias:
            return int(h)
    raise RuntimeError(f"object not found in robot tree: {alias}")
