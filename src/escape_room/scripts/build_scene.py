#!/usr/bin/env python3
"""Build an escape-room scene in CoppeliaSim from a JSON scenario.

Usage:
    pixi shell
    python src/escape_room/scripts/build_scene.py src/escape_room/scenarios/easy.json
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ── constants ─────────────────────────────────────────────────────────────

_MANAGED_PREFIXES = (
    "Wall",
    "Door",
    "Obstacle_",
    "TargetCube",
    "PressurePlate",
    "RoboMasterEP",
    "RoboMasterS1",
)

# Door geometry by wall_side index (0=North, 1=East, 2=South, 3=West).
# Lambda args: (door_w, thickness, height, hw, hl, centre_offset)
# Returns: (size_tuple, position_tuple)
_DOOR_GEOM = {
    0: lambda dw, th, h, hw, hl, c: ((dw, th, h), (c, hl + th / 2, h / 2)),
    1: lambda dw, th, h, hw, hl, c: ((th, dw, h), (hw + th / 2, c, h / 2)),
    2: lambda dw, th, h, hw, hl, c: ((dw, th, h), (c, -hl - th / 2, h / 2)),
    3: lambda dw, th, h, hw, hl, c: ((th, dw, h), (-hw - th / 2, c, h / 2)),
}


# ── geometry helpers ───────────────────────────────────────────────────────


def _segments_around_doors(wall_length, doors):
    """Solid wall segments left after cutting out door openings.

    Coordinates are along the wall, relative to the wall's centre.
    ``doors`` is a list of (centre_offset, width). Returns
    [(seg_length, seg_centre), ...] left-to-right.
    """
    half = wall_length / 2
    sorted_doors = sorted(doors, key=lambda d: d[0] - d[1] / 2)
    segments = []
    cursor = -half
    for centre, width in sorted_doors:
        left = centre - width / 2
        right = centre + width / 2
        if left < -half or right > half:
            raise ValueError(
                f"Door (centre={centre:.3f}, width={width:.3f}) "
                f"exceeds wall bounds [{-half:.3f}, {half:.3f}]"
            )
        if left <= cursor:
            raise ValueError(
                f"Door (centre={centre:.3f}, width={width:.3f}) "
                f"overlaps the previous door (ends at {cursor:.3f})"
            )
        segments.append((left - cursor, (cursor + left) / 2))
        cursor = right
    if cursor < half:
        segments.append((half - cursor, (cursor + half) / 2))
    return segments


def _wall_layout(width, length, thickness):
    """Per-wall geometry: (axis, length, centre_x, centre_y, alias).

    wall_side index follows the layout order: 0=North, 1=East,
    2=South, 3=West (matches scenario JSON).
    """
    hw, hl = width / 2, length / 2
    return [
        ("x", width + 2 * thickness, 0.0, hl + thickness / 2, "WallNorth"),
        ("y", length, hw + thickness / 2, 0.0, "WallEast"),
        ("x", width + 2 * thickness, 0.0, -hl - thickness / 2, "WallSouth"),
        ("y", length, -hw - thickness / 2, 0.0, "WallWest"),
    ]


# ── CoppeliaSim shape factory ──────────────────────────────────────────────


def make_shape(
    sim, prim_type, size, center, *, color, name, static=True, respondable=True
):
    """Create a primitive shape, set its position, color, physics, and alias."""
    handle = sim.createPrimitiveShape(prim_type, list(size), 0)
    sim.setObjectPosition(handle, list(center), -1)
    sim.setShapeColor(handle, "", sim.colorcomponent_ambient_diffuse, list(color))
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1 if static else 0)
    sim.setObjectInt32Param(
        handle, sim.shapeintparam_respondable, 1 if respondable else 0
    )
    sim.setObjectAlias(handle, name)
    return handle


# ── scene object builders ──────────────────────────────────────────────────


def build_walls(sim, room_cfg):
    """Build all four walls, cutting door openings where specified."""
    width = room_cfg["width"]
    length = room_cfg["length"]
    height = room_cfg["height"]
    thickness = room_cfg["wall_thickness"]
    all_doors = room_cfg.get("doors", [])

    for wall_side, (axis, wall_len, cx, cy, alias) in enumerate(
        _wall_layout(width, length, thickness)
    ):
        wall_doors = [
            (d["center_offset"], d["width"])
            for d in all_doors
            if d["wall_side"] == wall_side
        ]
        for i, (seg_len, seg_off) in enumerate(
            _segments_around_doors(wall_len, wall_doors)
        ):
            if axis == "x":
                size = (seg_len, thickness, height)
                pos = (cx + seg_off, cy, height / 2)
            else:
                size = (thickness, seg_len, height)
                pos = (cx, cy + seg_off, height / 2)
            h = make_shape(
                sim,
                sim.primitiveshape_cuboid,
                size,
                pos,
                color=(0.85, 0.85, 0.85),
                name=f"{alias}_{i}",
            )
            sim.setObjectSpecialProperty(h, sim.objectspecialproperty_detectable_all)


def build_door(sim, room_cfg, door_cfg, idx):
    """Place a door panel that fills the gap left in one wall."""
    width = room_cfg["width"]
    length = room_cfg["length"]
    thickness = room_cfg["wall_thickness"]
    height = room_cfg["height"]
    hw, hl = width / 2, length / 2

    side = door_cfg["wall_side"]
    door_w = door_cfg["width"]
    centre = door_cfg["center_offset"]
    color = door_cfg.get("color", (0.55, 0.30, 0.15))

    geom = _DOOR_GEOM.get(side)
    if geom is None:
        raise ValueError(f"unknown wall_side {side}")
    size, pos = geom(door_w, thickness, height, hw, hl, centre)

    h = make_shape(
        sim, sim.primitiveshape_cuboid, size, pos, color=color, name=f"Door_{idx}"
    )
    sim.setObjectSpecialProperty(h, sim.objectspecialproperty_detectable_all)
    return h


def build_obstacle(sim, obs, idx):
    """Build an obstacle from scenario config.

    ``obs['type']`` must be a CoppeliaSim primitive name (e.g. 'cuboid',
    'cylinder') — it is passed directly to ``sim.primitiveshape_<type>``.
    """
    prim = getattr(sim, f"primitiveshape_{obs['type']}", None)
    if prim is None:
        raise ValueError(f"unknown obstacle type: {obs['type']!r}")
    h = make_shape(
        sim,
        prim,
        obs["size"],
        obs["position"],
        color=obs.get("color", (0.4, 0.4, 0.5)),
        name=f"Obstacle_{idx}",
    )
    sim.setObjectSpecialProperty(h, sim.objectspecialproperty_detectable_all)
    return h


def build_target_cube(sim, cfg):
    """The "key" the robot must grasp. A tall thin cylinder is much
    easier for the gripper to pick up than a flat cube."""
    h = make_shape(
        sim,
        sim.primitiveshape_cylinder,
        cfg["size"],
        cfg["position"],
        color=cfg.get("color", (0.9, 0.2, 0.9)),
        name="TargetCube",
        static=False,
    )
    sim.setObjectSpecialProperty(h, sim.objectspecialproperty_detectable_all)
    return h


def build_pressure_plate(sim, cfg):
    """Build the pressure plate the robot drops the cube onto."""
    return make_shape(
        sim,
        sim.primitiveshape_cuboid,
        cfg["size"],
        cfg["position"],
        color=cfg.get("color", (0.1, 0.8, 0.2)),
        name="PressurePlate",
        static=True,
        respondable=False,
    )


# ── robot ──────────────────────────────────────────────────────────────────


def _resolve_model_path(robot_cfg, sim):
    explicit = robot_cfg.get("model_path")
    if explicit:
        path = Path(explicit).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"robot model not found: {path}")
        return str(path)
    coppelia_root = sim.getStringParam(sim.stringparam_scenedefaultdir)
    return f"{coppelia_root}/../models/robots/mobile/{robot_cfg['model']}.ttm"


# Loaded once at import time; path is relative to this file so the script
# can be invoked from any working directory.
_GRIPPER_HELPERS_LUA = (
    Path(__file__).resolve().parent / "gripper_helpers.lua"
).read_text()


def _inject_gripper_helpers(sim, robot_h, model_name):
    """Append the Lua signal-shim + ``_ext_*`` helpers to the gripper
    script. Idempotent: re-running the builder doesn't duplicate."""
    gripper_h = next(
        (
            int(h)
            for h in sim.getObjectsInTree(robot_h)
            if sim.getObjectAlias(int(h), 0) == "gripper_link_respondable"
        ),
        None,
    )
    if gripper_h is None:
        raise RuntimeError(f"gripper_link_respondable not found in {model_name}")
    script_h = sim.getScript(1, gripper_h)  # 1 = child script
    src = sim.getScriptStringParam(script_h, sim.scriptstringparam_text)
    if "function _ext_set_target" in src:
        return
    sim.setScriptStringParam(
        script_h, sim.scriptstringparam_text, src + _GRIPPER_HELPERS_LUA
    )
    print(f"[builder] injected gripper helpers into {model_name} script")


def load_robot(sim, robot_cfg):
    """Load a RoboMaster model, place it, and inject the gripper
    helpers needed by the explorer node. The in-scene alias is set
    to ``robot_cfg['model']`` so downstream code can resolve it."""
    model_name = robot_cfg.get("model", "RoboMasterEP")
    model_path = _resolve_model_path(robot_cfg, sim)

    handle = sim.loadModel(model_path)
    if handle < 0:
        raise RuntimeError(f"loadModel failed for {model_path}")

    pos = robot_cfg["position"]
    sim.setObjectPosition(handle, pos, -1)

    rpy = robot_cfg.get("orientation", [0, 0, 0])
    m = sim.getObjectMatrix(handle, -1)
    m = sim.rotateAroundAxis(m, [1, 0, 0], pos, math.radians(rpy[0]))
    m = sim.rotateAroundAxis(m, [0, 1, 0], pos, math.radians(rpy[1]))
    m = sim.rotateAroundAxis(m, [0, 0, 1], pos, math.radians(rpy[2]))
    sim.setObjectMatrix(handle, -1, m)
    sim.setObjectAlias(handle, model_name)

    # Reset physics for the whole subtree so the teleport doesn't
    # explode joints with built-up momentum.
    for h in sim.getObjectsInTree(handle):
        sim.resetDynamicObject(h)

    _inject_gripper_helpers(sim, handle, model_name)
    return handle


# ── scene management + entry ───────────────────────────────────────────────


def clear_scene(sim):
    """Remove only objects we manage (walls, obstacles, key, plate,
    door, robot). Default Coppelia floor / cameras / lights stay."""
    to_remove = [
        h
        for h in sim.getObjectsInTree(sim.handle_scene)
        if sim.getObjectAlias(h, 0).startswith(_MANAGED_PREFIXES)
    ]
    if to_remove:
        sim.removeObjects(to_remove)
        print(f"[builder] removed {len(to_remove)} managed object(s)")


def main(scenario_path: str) -> None:
    """Build the full scene from a scenario JSON file."""
    cfg = json.loads(Path(scenario_path).read_text())

    print("[builder] connecting to CoppeliaSim...")
    sim = RemoteAPIClient().require("sim")

    if sim.getSimulationState() != sim.simulation_stopped:
        print("[builder] stopping running simulation...")
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            pass

    clear_scene(sim)

    room = cfg["room"]
    print("[builder] building walls...")
    build_walls(sim, room)

    for i, door_cfg in enumerate(room.get("doors", [])):
        print(f"[builder] placing door {i} on wall_side {door_cfg['wall_side']}")
        build_door(sim, room, door_cfg, i)

    obstacles = cfg.get("obstacles", [])
    print(f"[builder] building {len(obstacles)} obstacle(s)...")
    for i, obs in enumerate(obstacles):
        build_obstacle(sim, obs, i)

    if "target_cube" in cfg:
        print("[builder] placing target cube...")
        build_target_cube(sim, cfg["target_cube"])

    if "pressure_plate" in cfg:
        print("[builder] placing pressure plate...")
        build_pressure_plate(sim, cfg["pressure_plate"])

    if "robot" in cfg:
        print(f"[builder] loading robot {cfg['robot']['model']}...")
        load_robot(sim, cfg["robot"])

    print("[builder] done. Scene ready.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: build_scene.py <scenario.json>")
        sys.exit(1)
    main(sys.argv[1])
