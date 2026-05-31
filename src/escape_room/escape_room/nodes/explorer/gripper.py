"""CoppeliaSim gripper I/O: open/close the jaws and toggle cube visibility.

The cube's detectable flag is cleared while carried so the lidar does not
report a stationary obstacle directly in front of base_link.
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class GripperIO:
    """Wraps the CoppeliaSim gripper script and cube lidar-visibility toggle."""

    _OPEN = 1     # target codes understood by the gripper's Lua script
    _CLOSE = 2

    def __init__(self, robot_alias: str, cube_alias: str, logger) -> None:
        self._logger = logger
        sim = RemoteAPIClient().require("sim")
        # robot_alias is like "/RoboMasterEP/BaseLinkFrame"; take the model root
        model_alias = "/" + robot_alias.lstrip("/").split("/")[0]
        model_h = sim.getObject(model_alias)

        # the gripper's child script exposes _ext_* helpers we call below
        gripper_h = self._find_in_tree(sim, model_h, "gripper_link_respondable")
        self._sim = sim
        self._script_h = sim.getScript(1, gripper_h)   # 1 = child script
        self._cube_h = sim.getObject(cube_alias)

    def open(self) -> None:
        # ask the script to drive the jaws open (non-blocking)
        self._sim.callScriptFunction("_ext_set_target", self._script_h, self._OPEN)

    def close(self) -> None:
        self._sim.callScriptFunction("_ext_set_target", self._script_h, self._CLOSE)

    def is_open(self, elapsed_s: float, timeout_s: float) -> bool:
        return self._reached(self._OPEN, elapsed_s, timeout_s)

    def is_closed(self, elapsed_s: float, timeout_s: float) -> bool:
        return self._reached(self._CLOSE, elapsed_s, timeout_s)

    def set_cube_visible(self, visible: bool) -> None:
        # toggle whether sensors (lidar/camera) can detect the cube; we hide it
        # while it's carried so it isn't mapped as an obstacle in front of us
        prop = self._sim.objectspecialproperty_detectable_all if visible else 0
        self._sim.setObjectSpecialProperty(self._cube_h, prop)

    def _reached(self, target: int, elapsed_s: float, timeout_s: float) -> bool:
        # has the gripper reached the target state? give up after a timeout so
        # the mission can't stall if the script never reports
        if elapsed_s >= timeout_s:
            self._logger.warn(f"gripper timeout waiting for state {target}")
            return True
        cur = self._sim.callScriptFunction("_ext_get_state", self._script_h)
        return cur is not None and int(cur) == target

    @staticmethod
    def _find_in_tree(sim, root_h: int, alias: str) -> int:
        # search the robot's object tree for a child with the given alias
        for h in sim.getObjectsInTree(root_h):
            if sim.getObjectAlias(int(h), 0) == alias:
                return int(h)
        raise RuntimeError(f"object not found in robot tree: {alias}")
