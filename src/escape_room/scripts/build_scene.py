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
    position,
    color=(0.7, 0.7, 0.7),
    name=None,
    static=True,
    respondable=True,
):
    """Create a primitive shape and place it. Sizes are full extents in meters."""
    handle = sim.createPrimitiveShape(prim_type, list(size), 0)

    if handle is None or handle < 0:
        raise RuntimeError(f"createPrimitiveShape returned invalid handle: {handle}")

    sim.setObjectPosition(handle, list(position), -1)
    sim.setShapeColor(handle, "", sim.colorcomponent_ambient_diffuse, list(color))
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1 if static else 0)
    sim.setObjectInt32Param(
        handle, sim.shapeintparam_respondable, 1 if respondable else 0
    )
    if name:
        sim.setObjectAlias(handle, name)
    return handle


def _segments_around_gap(length, gap_w, gap_offset):
    """Wall segments around a centered gap. Returns [(seg_length, seg_center), ...]."""
    half = length / 2
    gap_l = gap_offset - gap_w / 2
    gap_r = gap_offset + gap_w / 2
    segments = []
    if gap_l > -half:
        segments.append((gap_l - (-half), (-half + gap_l) / 2))
    if gap_r < half:
        segments.append((half - gap_r, (gap_r + half) / 2))
    return segments


def build_walls(sim, prim, width, depth, height, thickness, door_cfg=None):
    hw, hd = width / 2, depth / 2
    walls = {
        "north": dict(
            axis="x",
            length=width + 2 * thickness,
            center=(0, hd + thickness / 2, height / 2),
            name="WallNorth",
        ),
        "south": dict(
            axis="x",
            length=width + 2 * thickness,
            center=(0, -hd - thickness / 2, height / 2),
            name="WallSouth",
        ),
        "east": dict(
            axis="y",
            length=depth,
            center=(hw + thickness / 2, 0, height / 2),
            name="WallEast",
        ),
        "west": dict(
            axis="y",
            length=depth,
            center=(-hw - thickness / 2, 0, height / 2),
            name="WallWest",
        ),
    }

    door_side = door_cfg["wall"] if door_cfg else None
    door_w = door_cfg["width"] if door_cfg else 0.0
    door_off = door_cfg.get("offset", 0.0) if door_cfg else 0.0

    handles = []
    for side, w in walls.items():
        if side == door_side:
            for i, (seg_len, seg_ctr) in enumerate(
                _segments_around_gap(w["length"], door_w, door_off)
            ):
                if w["axis"] == "x":
                    size = (seg_len, thickness, height)
                    pos = (w["center"][0] + seg_ctr, w["center"][1], w["center"][2])
                else:
                    size = (thickness, seg_len, height)
                    pos = (w["center"][0], w["center"][1] + seg_ctr, w["center"][2])
                h = make_shape(
                    sim,
                    prim["box"],
                    size,
                    pos,
                    color=(0.85, 0.85, 0.85),
                    name=f"{w['name']}_{i}",
                )
                sim.setObjectSpecialProperty(
                    h, sim.objectspecialproperty_detectable_all
                )
                handles.append(h)
        else:
            if w["axis"] == "x":
                size = (w["length"], thickness, height)
            else:
                size = (thickness, w["length"], height)
            h = make_shape(
                sim,
                prim["box"],
                size,
                w["center"],
                color=(0.85, 0.85, 0.85),
                name=w["name"],
            )
            sim.setObjectSpecialProperty(h, sim.objectspecialproperty_detectable_all)
            handles.append(h)
    return handles


def build_door(sim, prim, room, door_cfg):
    """Place a Door box that fills the gap left in one of the room walls."""
    width, depth = room["size"][0], room["size"][1]
    thickness = room["wall_thickness"]
    height = room["wall_height"]
    hw, hd = width / 2, depth / 2

    side = door_cfg["wall"]
    door_w = door_cfg["width"]
    offset = door_cfg.get("offset", 0.0)
    color = door_cfg.get("color", (0.55, 0.30, 0.15))

    if side == "north":
        size = (door_w, thickness, height)
        pos = (offset, hd + thickness / 2, height / 2)
    elif side == "south":
        size = (door_w, thickness, height)
        pos = (offset, -hd - thickness / 2, height / 2)
    elif side == "east":
        size = (thickness, door_w, height)
        pos = (hw + thickness / 2, offset, height / 2)
    elif side == "west":
        size = (thickness, door_w, height)
        pos = (-hw - thickness / 2, offset, height / 2)
    else:
        raise ValueError(f"Unknown door wall: {side}")

    h = make_shape(
        sim,
        prim["box"],
        size,
        pos,
        color=color,
        name="Door",
        static=True,
        respondable=True,
    )
    sim.setObjectSpecialProperty(h, sim.objectspecialproperty_detectable_all)
    return h


def build_obstacle(sim, prim, obs, idx):
    color = obs.get("color", (0.4, 0.4, 0.5))
    h = make_shape(
        sim,
        prim[obs["type"]],
        obs["size"],
        obs["position"],
        color=color,
        name=f"Obstacle_{idx}",
    )
    sim.setObjectSpecialProperty(h, sim.objectspecialproperty_detectable_all)
    return h


def build_target_cube(sim, prim, cfg):
    s = cfg["size"]
    h = make_shape(
        sim,
        prim["box"],
        [s, s, s],
        cfg["position"],
        color=cfg.get("color", (0.9, 0.2, 0.2)),
        name="TargetCube",
        static=False,
    )
    sim.setObjectSpecialProperty(h, sim.objectspecialproperty_detectable_all)
    sim.setObjectInt32Param(h, sim.shapeintparam_static, 0)
    return h


def build_pressure_plate(sim, prim, cfg):
    h = make_shape(
        sim,
        prim["box"],
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

    # Read the pose the model assumed naturally on load
    natural_pos = sim.getObjectPosition(handle, -1)
    natural_orient = sim.getObjectOrientation(handle, -1)

    # Override only x, y from the JSON; keep the model's natural z
    target_pos = [
        robot_cfg["position"][0],
        robot_cfg["position"][1],
        natural_pos[2],
    ]

    # Override only yaw; keep natural roll and pitch (should already be ~0).
    # Accept either a scalar yaw or a [roll, pitch, yaw] triple in degrees.
    orient_deg = robot_cfg.get("orientation_deg", 0)
    yaw_deg = orient_deg[-1] if isinstance(orient_deg, (list, tuple)) else orient_deg
    yaw_rad = math.radians(yaw_deg)
    target_orient = [natural_orient[0], natural_orient[1], yaw_rad]

    sim.setObjectPosition(handle, target_pos, -1)
    sim.setObjectOrientation(handle, target_orient, -1)
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

    # Resolve primitive type constants from the running sim
    prim = {
        "box": sim.primitiveshape_cuboid,
        "cylinder": sim.primitiveshape_cylinder,
    }

    if sim.getSimulationState() != sim.simulation_stopped:
        print("[builder] Stopping running simulation...")
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            pass

    print("[builder] Clearing scene...")
    clear_scene(sim)

    print("[builder] Building walls...")
    room = cfg["room"]
    door_cfg = cfg.get("door")
    build_walls(
        sim,
        prim,
        width=room["size"][0],
        depth=room["size"][1],
        height=room["wall_height"],
        thickness=room["wall_thickness"],
        door_cfg=door_cfg,
    )

    if door_cfg is not None:
        print(f"[builder] Placing door on {door_cfg['wall']} wall...")
        build_door(sim, prim, room, door_cfg)

    print(f"[builder] Building {len(cfg.get('obstacles', []))} obstacles...")
    for i, obs in enumerate(cfg.get("obstacles", [])):
        build_obstacle(sim, prim, obs, i)

    if "target_cube" in cfg:
        print("[builder] Placing target cube...")
        build_target_cube(sim, prim, cfg["target_cube"])

    if "pressure_plate" in cfg:
        print("[builder] Placing pressure plate...")
        build_pressure_plate(sim, prim, cfg["pressure_plate"])

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
