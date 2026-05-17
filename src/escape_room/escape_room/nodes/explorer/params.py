"""ROS parameter declaration + loading for the explorer node."""


def declare_explorer_params(node) -> None:
    """Declare every ROS param and copy its value as an attribute on node."""
    p = node.declare_parameter
    p("robot_alias", "/RoboMasterEP/BaseLinkFrame")
    p("cube_alias", "/TargetCube")
    p("map_frame", "map")
    p("base_frame", "base_link")
    p("control_rate_hz", 4.0)
    p("door_threshold_inset_m", 0.20)
    p("exit_drive_speed_mps", 0.10)
    p("exit_drive_duration_s", 5.0)
    p("pickup_standoff_m", 0.50)
    p("pickup_engage_dist_tol_m", 0.03)
    p("drop_backup_speed_mps", 0.05)
    p("drop_backup_duration_s", 8.0)
    p("plate_drop_distance_m", 0.30)
    p("plate_drop_dist_tol_m", 0.04)
    p("park_max_speed_mps", 0.06)
    p("align_yaw_tol_rad", 0.08)
    p("align_kp", 1.5)
    p("align_max_omega", 0.6)
    p("gripper_timeout_s", 4.0)

    def g(n):
        return node.get_parameter(n).value

    node.robot_alias = g("robot_alias")
    node.cube_alias = g("cube_alias")
    node.map_frame = g("map_frame")
    node.base_frame = g("base_frame")
    node.control_rate_hz = float(g("control_rate_hz"))
    node.door_threshold_inset = float(g("door_threshold_inset_m"))
    node.exit_drive_speed = float(g("exit_drive_speed_mps"))
    node.exit_drive_duration = float(g("exit_drive_duration_s"))
    node.pickup_standoff = float(g("pickup_standoff_m"))
    node.pickup_engage_dist_tol = float(g("pickup_engage_dist_tol_m"))
    node.backup_speed = float(g("drop_backup_speed_mps"))
    node.backup_duration = float(g("drop_backup_duration_s"))
    node.drop_distance = float(g("plate_drop_distance_m"))
    node.drop_dist_tol = float(g("plate_drop_dist_tol_m"))
    node.park_max_speed = float(g("park_max_speed_mps"))
    node.align_yaw_tol = float(g("align_yaw_tol_rad"))
    node.align_kp = float(g("align_kp"))
    node.align_max_omega = float(g("align_max_omega"))
    node.gripper_timeout = float(g("gripper_timeout_s"))
