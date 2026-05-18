"""CoppeliaSim ZMQ: gripper I/O and scene setup (handles + geometry)."""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

GRIPPER_OPEN = 1
GRIPPER_CLOSE = 2


class GripperIO:
    """Talks to the CoppeliaSim gripper script and toggles the cube's
    detectable flag while it's carried — otherwise the lidar reports
    a stationary obstacle right in front of base_link and Nav2 refuses
    to advance."""

    def __init__(self, sim, gripper_script_h: int, cube_h: int) -> None:
        self._sim = sim
        self._script_h = gripper_script_h
        self._cube_h = cube_h

    def open(self) -> None:
        self._sim.callScriptFunction("_ext_set_target", self._script_h, GRIPPER_OPEN)

    def close(self) -> None:
        self._sim.callScriptFunction("_ext_set_target", self._script_h, GRIPPER_CLOSE)

    def reached(self, target: int, elapsed_s: float, timeout_s: float, logger) -> bool:
        if elapsed_s >= timeout_s:
            logger.warn(f"gripper timeout waiting for state {target}")
            return True
        cur = self._sim.callScriptFunction("_ext_get_state", self._script_h)
        return cur is not None and int(cur) == target

    def hide_cube_from_lidar(self) -> None:
        self._sim.setObjectSpecialProperty(self._cube_h, 0)

    def show_cube_to_lidar(self) -> None:
        self._sim.setObjectSpecialProperty(
            self._cube_h, self._sim.objectspecialproperty_detectable_all
        )


def setup_sim(node) -> GripperIO:
    """Resolve sim handles and read pose-independent geometry from the scene.

    Sets on node:
      - ``pickup_engage_dist``: attachPoint x-offset from BaseLinkFrame.
      - ``door_normal``: outward unit normal of the door panel (cardinal).

    Returns GripperIO for gripper open/close + cube lidar visibility.
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
    # Thinnest XY axis = wall-normal direction; sign from which side of origin.
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
