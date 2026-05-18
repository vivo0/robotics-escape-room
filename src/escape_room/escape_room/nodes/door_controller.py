#!/usr/bin/env python3
"""
ROS2 node that opens the escape-room door in CoppeliaSim once the target
cube is pressed onto the pressure plate.

Run (after `colcon build` and sourcing the workspace):
    ros2 run escape_room door_controller

Or directly:
    python -m escape_room.nodes.door_controller
"""

import math

import rclpy
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from rclpy.node import Node


class DoorController(Node):
    def __init__(self):
        super().__init__("door_controller")

        self.declare_parameter("cube_alias", "/TargetCube")
        self.declare_parameter("plate_alias", "/PressurePlate")
        self.declare_parameter("door_alias", "/Door_0")
        self.declare_parameter("xy_margin", 0.05)
        self.declare_parameter("z_max_offset", 0.15)
        self.declare_parameter("open_distance", 0.0)
        self.declare_parameter("tick_period", 0.1)
        self.declare_parameter("latch", True)

        self.cube_alias = self.get_parameter("cube_alias").value
        self.plate_alias = self.get_parameter("plate_alias").value
        self.door_alias = self.get_parameter("door_alias").value
        self.xy_margin = float(self.get_parameter("xy_margin").value)
        self.z_max_offset = float(self.get_parameter("z_max_offset").value)
        self.open_distance = float(self.get_parameter("open_distance").value)
        self.latch = bool(self.get_parameter("latch").value)
        period = float(self.get_parameter("tick_period").value)

        self.get_logger().info("Connecting to CoppeliaSim ZMQ remote API...")
        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")

        self.cube = self._resolve(self.cube_alias)
        self.plate = self._resolve(self.plate_alias)
        self.door = self._resolve(self.door_alias)

        self.plate_bbox_xy = self._half_extents_xy(self.plate)
        door_bbox = self._half_extents_xyz(self.door)
        self.door_height = 2.0 * door_bbox[2]
        if self.open_distance <= 0.0:
            self.open_distance = self.door_height + 0.05

        self.door_closed_pos = self.sim.getObjectPosition(self.door, -1)
        self.door_open_pos = list(self.door_closed_pos)
        self.door_open_pos[2] -= self.open_distance

        self.is_open = False
        self._last_status_log = 0.0

        self.timer = self.create_timer(period, self.tick)
        cube_pos = self.sim.getObjectPosition(self.cube, -1)
        plate_pos = self.sim.getObjectPosition(self.plate, -1)
        self.get_logger().info(
            f"Ready. cube={self.cube_alias} @ "
            f"({cube_pos[0]:.2f},{cube_pos[1]:.2f},{cube_pos[2]:.2f}); "
            f"plate={self.plate_alias} @ "
            f"({plate_pos[0]:.2f},{plate_pos[1]:.2f},{plate_pos[2]:.2f}); "
            f"plate half-extents=({self.plate_bbox_xy[0]:.2f},{self.plate_bbox_xy[1]:.2f}) "
            f"+ margin {self.xy_margin:.2f}. "
            f"Door slides {self.open_distance:.2f} m down on trigger."
        )

    def _resolve(self, alias):
        try:
            return self.sim.getObject(alias)
        except Exception as e:
            self.get_logger().error(
                f"Could not find object {alias} in scene. "
                f"Did you run build_scene.py? ({e})"
            )
            raise

    def _half_extents_xy(self, handle):
        bb = self._half_extents_xyz(handle)
        return bb[0], bb[1]

    def _half_extents_xyz(self, handle):
        # min/max of OBB along each local axis; works for axis-aligned primitives.
        xmin = self.sim.getObjectFloatParam(
            handle, self.sim.objfloatparam_objbbox_min_x
        )
        xmax = self.sim.getObjectFloatParam(
            handle, self.sim.objfloatparam_objbbox_max_x
        )
        ymin = self.sim.getObjectFloatParam(
            handle, self.sim.objfloatparam_objbbox_min_y
        )
        ymax = self.sim.getObjectFloatParam(
            handle, self.sim.objfloatparam_objbbox_max_y
        )
        zmin = self.sim.getObjectFloatParam(
            handle, self.sim.objfloatparam_objbbox_min_z
        )
        zmax = self.sim.getObjectFloatParam(
            handle, self.sim.objfloatparam_objbbox_max_z
        )
        return ((xmax - xmin) / 2.0, (ymax - ymin) / 2.0, (zmax - zmin) / 2.0)

    def open_door(self):
        self.sim.setObjectPosition(self.door, self.door_open_pos, -1)
        try:
            self.sim.resetDynamicObject(self.door)
        except Exception:
            pass
        self.is_open = True
        self.get_logger().info("Door opened.")

    def close_door(self):
        self.sim.setObjectPosition(self.door, self.door_closed_pos, -1)
        try:
            self.sim.resetDynamicObject(self.door)
        except Exception:
            pass
        self.is_open = False
        self.get_logger().info("Door closed.")

    def tick(self):
        try:
            cube_pos = self.sim.getObjectPosition(self.cube, -1)
            plate_pos = self.sim.getObjectPosition(self.plate, -1)
        except Exception as e:
            self.get_logger().warn(f"Sim query failed: {e}")
            return

        hx, hy = self.plate_bbox_xy
        dx = cube_pos[0] - plate_pos[0]
        dy = cube_pos[1] - plate_pos[1]
        dz = cube_pos[2] - plate_pos[2]
        triggered = (
            abs(dx) <= hx + self.xy_margin
            and abs(dy) <= hy + self.xy_margin
            and -0.05 <= dz <= self.z_max_offset
        )

        # Heartbeat: every ~3 s log how far the cube is from the plate, so
        # the user can see the controller is alive and how far they need to push.
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_status_log > 3.0:
            xy_dist = math.hypot(dx, dy)
            self.get_logger().info(
                f"cube-plate xy_dist={xy_dist:.2f} m, dz={dz:+.3f} m, "
                f"door_open={self.is_open}, triggered={triggered}"
            )
            self._last_status_log = now

        if triggered and not self.is_open:
            self.get_logger().info("Cube on plate — opening door.")
            self.open_door()
        elif not triggered and self.is_open and not self.latch:
            self.get_logger().info("Cube left plate — closing door.")
            self.close_door()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DoorController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
