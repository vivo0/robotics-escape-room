#!/usr/bin/env python3
"""
Builds an escape-room scene in CoppeliaSim from a JSON scenario file.

Usage:
    pixi shell
    python src/escape_room/scripts/build_scene.py src/escape_room/scenarios/easy.json
"""

import json
import math
import sys
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def make_shape(
    sim,
    prim_type,
    size,
    center,
    color=(0.7, 0.7, 0.7),
    name=None,
    static=True,
    respondable=True,
):
    """Create a primitive shape and place it. Sizes are full extents in meters.
    Center is the shape's center in world coordinates (CoppeliaSim convention).
    """
    handle = sim.createPrimitiveShape(prim_type, list(size), 0)

    if handle is None or handle < 0:
        raise RuntimeError(f"createPrimitiveShape returned invalid handle: {handle}")

    sim.setObjectPosition(handle, list(center), -1)
    sim.setShapeColor(handle, "", sim.colorcomponent_ambient_diffuse, list(color))
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1 if static else 0)
    sim.setObjectInt32Param(
        handle, sim.shapeintparam_respondable, 1 if respondable else 0
    )
    if name:
        sim.setObjectAlias(handle, name)
    return handle


def _segments_around_doors(wall_length, door_centers):
    """Return the solid wall segments that remain after cutting out door openings.

    All coordinates are relative to the wall's center (positive = right).
    The wall spans [-wall_length/2, +wall_length/2].

    door_centers: list of (door_center, door_width), both relative to wall center.
    Returns [(seg_length, seg_center), ...] sorted left-to-right, also relative
    to wall center.
    """
    half = wall_length / 2
    # Sort doors left-to-right by their left edge so the sweep works in one pass.
    sorted_doors = sorted(door_centers, key=lambda d: d[0] - d[1] / 2)
    segments = []
    # x is a cursor that starts at the wall's left edge and advances to each
    # door's right edge, marking the start of the next potential solid segment.
    x = -half
    for door_center, door_w in sorted_doors:
        door_l = door_center - door_w / 2  # left edge of this door
        door_r = door_center + door_w / 2  # right edge of this door
        if door_l < -half or door_r > half:
            raise ValueError(
                f"Door (center={door_center:.3f}, width={door_w:.3f}) "
                f"exceeds wall bounds [{-half:.3f}, {half:.3f}]."
            )
        if door_l <= x:
            raise ValueError(
                f"Door (center={door_center:.3f}, width={door_w:.3f}) "
                f"overlaps or is adjacent to the previous door (ends at {x:.3f})."
            )
        # Solid segment from x to door_l; its center is their midpoint.
        segments.append((door_l - x, (x + door_l) / 2))
        x = door_r
    # Solid segment from last door's right edge to the wall's right edge.
    if x < half:
        segments.append((half - x, (x + half) / 2))
    return segments



def build_walls(sim, room_cfg):
    width = room_cfg["width"]
    length = room_cfg["length"]
    height = room_cfg["height"]
    thickness = room_cfg["wall_thickness"]
    hw, hl = width / 2, length / 2
    doors = room_cfg.get("doors", [])

    # (wall_side, axis, full_wall_length, center_x, center_y, alias_prefix)
    # N/S walls extend past corners (+2*thickness); E/W walls span the interior only.
    walls = [
        ("x", width + 2 * thickness, 0.0, hl + thickness / 2, "WallNorth"),
        ("y", length, hw + thickness / 2, 0.0, "WallEast"),
        ("x", width + 2 * thickness, 0.0, -(hl + thickness / 2), "WallSouth"),
        ("y", length, -(hw + thickness / 2), 0.0, "WallWest"),
    ]

    handles = []
    for wall_side, (axis, wall_len, center_x, center_y, alias) in enumerate(walls):
        doors = [d for d in doors if d["wall_side"] == wall_side]
        door_centers = [(d["center_pos"], d["width"]) for d in doors]

        for seg_idx, (seg_len, seg_center) in enumerate(
            _segments_around_doors(wall_len, door_centers)
        ):
            if axis == "x":
                size = (seg_len, thickness, height)
                seg_center_x, seg_center_y = center_x + seg_center, center_y
            else:
                size = (thickness, seg_len, height)
                seg_center_x, seg_center_y = center_x, center_y + seg_center

            handle = make_shape(
                sim,
                getattr(sim, "primitiveshape_cuboid"),
                size,
                (seg_center_x, seg_center_y, height / 2),
                color=(0.85, 0.85, 0.85),
                name=f"{alias}_{seg_idx}",
            )
            sim.setObjectSpecialProperty(
                handle, sim.objectspecialproperty_detectable_all
            )
            handles.append(handle)
    return handles


def build_door(sim, room_cfg, door_cfg, door_idx):
    """Place a door panel that fills the gap left in one of the room walls."""
    width = room_cfg["width"]
    length = room_cfg["length"]
    thickness = room_cfg["wall_thickness"]
    height = room_cfg["height"]
    hw, hl = width / 2, length / 2

    wall_side = door_cfg["wall_side"]
    door_w = door_cfg["width"]
    color = door_cfg.get("color", (0.55, 0.30, 0.15))

    door_center = door_cfg["center_pos"]

    if wall_side == 0:
        size = (door_w, thickness, height)
        center_x, center_y = door_center, hl + thickness / 2
    elif wall_side == 1:
        size = (thickness, door_w, height)
        center_x, center_y = hw + thickness / 2, door_center
    elif wall_side == 2:
        size = (door_w, thickness, height)
        center_x, center_y = door_center, -(hl + thickness / 2)
    elif wall_side == 3:
        size = (thickness, door_w, height)
        center_x, center_y = -(hw + thickness / 2), door_center
    else:
        raise ValueError(f"Unknown wall_side: {wall_side}")

    handle = make_shape(
        sim,
        getattr(sim, "primitiveshape_cuboid"),
        size,
        (center_x, center_y, height / 2),
        color=color,
        name=f"Door_{door_idx}",
        static=True,
        respondable=True,
    )
    sim.setObjectSpecialProperty(handle, sim.objectspecialproperty_detectable_all)
    return handle


_PRIM_TYPE = {"box": "cuboid", "cylinder": "cylinder"}


def build_obstacle(sim, obs, idx):
    color = obs.get("color", (0.4, 0.4, 0.5))
    prim = getattr(sim, f"primitiveshape_{_PRIM_TYPE[obs['type']]}")
    handle = make_shape(
        sim,
        prim,
        obs["size"],
        obs["position"],
        color=color,
        name=f"Obstacle_{idx}",
    )
    sim.setObjectSpecialProperty(handle, sim.objectspecialproperty_detectable_all)
    return handle


def build_target_key(sim, cfg):
    """Place the "key" the robot must grasp and move.

    A tall thin cylinder, much easier for the RoboMaster gripper to
    pick up than a low cube. ``cfg["size"]`` is [diameter, diameter,
    height] in metres.
    """
    handle = make_shape(
        sim,
        getattr(sim, "primitiveshape_cylinder"),
        cfg["size"],
        cfg["position"],
        color=cfg.get("color", (0.9, 0.2, 0.2)),
        name="TargetCube",
        static=False,
    )
    sim.setObjectSpecialProperty(handle, sim.objectspecialproperty_detectable_all)
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 0)
    return handle


def build_pressure_plate(sim, cfg):
    h = make_shape(
        sim,
        getattr(sim, "primitiveshape_cuboid"),
        cfg["size"],
        cfg["position"],
        color=cfg.get("color", (0.2, 0.6, 0.9)),
        name="PressurePlate",
        static=True,
        respondable=False,
    )
    return h


def load_robot(sim, robot_cfg):
    """Load a RoboMaster model preserving its natural pose offsets.

    Resolution order for the model file:
      1. robot_cfg['model_path'] — explicit path (absolute, or relative to cwd)
      2. <coppelia models>/robots/mobile/<robot_cfg['model']>.ttm — fallback
    The in-scene alias is always robot_cfg['model'] so downstream code
    (clear_scene, robomaster_ros TF chain) stays stable.
    """
    model_name = robot_cfg.get("model", "RoboMasterEP")
    explicit_path = robot_cfg.get("model_path")

    if explicit_path:
        model_path = str(Path(explicit_path).expanduser().resolve())
        if not Path(model_path).is_file():
            raise FileNotFoundError(f"Robot model not found: {model_path}")
    else:
        coppelia_root = sim.getStringParam(sim.stringparam_scenedefaultdir)
        model_path = f"{coppelia_root}/../models/robots/mobile/{model_name}.ttm"

    handle = sim.loadModel(model_path)
    if handle < 0:
        raise RuntimeError(f"loadModel failed for {model_path}")

    pos = robot_cfg["position"]
    sim.setObjectPosition(handle, pos, -1)

    orient = robot_cfg.get("orientation", [0, 0, 0])
    m = sim.getObjectMatrix(handle, -1)
    m = sim.rotateAroundAxis(m, [1, 0, 0], pos, math.radians(orient[0]))
    m = sim.rotateAroundAxis(m, [0, 1, 0], pos, math.radians(orient[1]))
    m = sim.rotateAroundAxis(m, [0, 0, 1], pos, math.radians(orient[2]))
    sim.setObjectMatrix(handle, -1, m)
    sim.setObjectAlias(handle, model_name)

    # Reset physics for the whole model subtree to avoid teleport explosions
    for h in sim.getObjectsInTree(handle):
        try:
            sim.resetDynamicObject(h)
        except Exception:
            pass

    return handle


def clear_scene(sim):
    """
    Remove only objects we manage (walls, obstacles, target cube, plate, robot).
    Identified by alias prefix. Leaves the default Coppelia scene (floor,
    cameras, lights) untouched.
    """
    managed_prefixes = (
        "Wall",
        "Obstacle_",
        "TargetCube",
        "PressurePlate",
        "Door",
        "RoboMasterEP",
        "RoboMasterS1",
    )
    handles_to_remove = []
    for handle in sim.getObjectsInTree(sim.handle_scene):
        try:
            alias = sim.getObjectAlias(handle, 0)
        except Exception:
            continue
        if alias.startswith(managed_prefixes):
            handles_to_remove.append(handle)
    if handles_to_remove:
        try:
            sim.removeObjects(handles_to_remove)
            print(f"[builder] Removed {len(handles_to_remove)} managed objects.")
        except Exception as e:
            print(f"[builder] clear_scene warning: {e}")


def main(scenario_path: str):
    cfg = json.loads(Path(scenario_path).read_text())

    print("[builder] Connecting to CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require("sim")

    if sim.getSimulationState() != sim.simulation_stopped:
        print("[builder] Stopping running simulation...")
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            pass

    print("[builder] Clearing scene...")
    clear_scene(sim)

    room = cfg["room"]
    doors = room.get("doors", [])

    print("[builder] Building walls...")
    build_walls(sim, room)

    for i, door_cfg in enumerate(doors):
        print(f"[builder] Placing door {i} on wall_side {door_cfg['wall_side']}...")
        build_door(sim, room, door_cfg, i)

    print(f"[builder] Building {len(cfg.get('obstacles', []))} obstacles...")
    for i, obs in enumerate(cfg.get("obstacles", [])):
        build_obstacle(sim, obs, i)

    if "target_cube" in cfg:
        print("[builder] Placing target cube...")
        build_target_key(sim, cfg["target_cube"])

    if "pressure_plate" in cfg:
        print("[builder] Placing pressure plate...")
        build_pressure_plate(sim, cfg["pressure_plate"])

    if "robot" in cfg:
        print(f"[builder] Loading robot: {cfg['robot']['model']}...")
        try:
            load_robot(sim, cfg["robot"])
        except Exception as e:
            print(f"[builder] WARNING: could not load robot model: {e}")
            print("[builder] Drag the robot from the model browser manually for now.")

    print("[builder] Done. Scene ready.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: build_scene.py <scenario.json>")
        sys.exit(1)
    main(sys.argv[1])
