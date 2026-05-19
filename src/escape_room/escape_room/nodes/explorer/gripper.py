"""CoppeliaSim gripper I/O and scene-geometry initialisation."""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class GripperIO:
    """Wraps the CoppeliaSim gripper script and cube lidar-visibility toggle.

    Also reads two pose-independent geometry constants from the scene at
    construction time and exposes them as attributes:

      ``pickup_engage_dist`` — attachPoint x-offset from BaseLinkFrame (m).
      ``door_normal``        — outward unit normal of the door panel (cardinal).

    The cube's detectable flag is cleared while carried so the lidar does not
    report a stationary obstacle directly in front of base_link.
    """

    _OPEN = 1
    _CLOSE = 2

    def __init__(self, robot_alias: str, cube_alias: str, logger) -> None:
        self._logger = logger
        sim = RemoteAPIClient().require("sim")
        model_alias = "/" + robot_alias.lstrip("/").split("/")[0]
        model_h = sim.getObject(model_alias)

        gripper_h = self._find_in_tree(sim, model_h, "gripper_link_respondable")
        self._sim = sim
        self._script_h = sim.getScript(1, gripper_h)
        self._cube_h = sim.getObject(cube_alias)

        attach_h = self._find_in_tree(sim, model_h, "attachPoint")
        base_h = self._find_in_tree(sim, model_h, "BaseLinkFrame")
        self.pickup_engage_dist: float = float(
            sim.getObjectPosition(attach_h, base_h)[0]
        )

        logger.info(f"pickup_engage_dist = {self.pickup_engage_dist:.3f} m")

    def open(self) -> None:
        self._sim.callScriptFunction("_ext_set_target", self._script_h, self._OPEN)

    def close(self) -> None:
        self._sim.callScriptFunction("_ext_set_target", self._script_h, self._CLOSE)

    def is_open(self, elapsed_s: float, timeout_s: float) -> bool:
        return self._reached(self._OPEN, elapsed_s, timeout_s)

    def is_closed(self, elapsed_s: float, timeout_s: float) -> bool:
        return self._reached(self._CLOSE, elapsed_s, timeout_s)

    def set_cube_visible(self, visible: bool) -> None:
        prop = self._sim.objectspecialproperty_detectable_all if visible else 0
        self._sim.setObjectSpecialProperty(self._cube_h, prop)

    def _reached(self, target: int, elapsed_s: float, timeout_s: float) -> bool:
        if elapsed_s >= timeout_s:
            self._logger.warn(f"gripper timeout waiting for state {target}")
            return True
        cur = self._sim.callScriptFunction("_ext_get_state", self._script_h)
        return cur is not None and int(cur) == target

    @staticmethod
    def _find_in_tree(sim, root_h: int, alias: str) -> int:
        for h in sim.getObjectsInTree(root_h):
            if sim.getObjectAlias(int(h), 0) == alias:
                return int(h)
        raise RuntimeError(f"object not found in robot tree: {alias}")
