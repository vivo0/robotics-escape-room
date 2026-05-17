"""ZMQ wrapper for gripper open/close and cube lidar visibility."""

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
