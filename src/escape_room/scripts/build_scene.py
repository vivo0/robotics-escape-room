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



def make_shape(sim, prim_type, size, position, color=(0.7, 0.7, 0.7),
               name=None, static=True, respondable=True):
    """Create a primitive shape and place it. Sizes are full extents in meters."""
    handle = sim.createPrimitiveShape(prim_type, list(size), 0)

    if handle is None or handle < 0:
        raise RuntimeError(f"createPrimitiveShape returned invalid handle: {handle}")

    sim.setObjectPosition(handle, list(position), -1)
    sim.setShapeColor(handle, '', sim.colorcomponent_ambient_diffuse, list(color))
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1 if static else 0)
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 1 if respondable else 0)
    if name:
        sim.setObjectAlias(handle, name)
    return handle


def build_walls(sim, prim, width, depth, height, thickness):
    hw, hd = width / 2, depth / 2
    walls = [
        ((width + 2*thickness, thickness, height),
         (0,  hd + thickness/2, height/2), 'WallNorth'),
        ((width + 2*thickness, thickness, height),
         (0, -hd - thickness/2, height/2), 'WallSouth'),
        ((thickness, depth, height),
         ( hw + thickness/2, 0, height/2), 'WallEast'),
        ((thickness, depth, height),
         (-hw - thickness/2, 0, height/2), 'WallWest'),
    ]
    handles = []
    for size, pos, name in walls:
        h = make_shape(sim, prim['box'], size, pos,
                       color=(0.85, 0.85, 0.85), name=name)
        sim.setObjectSpecialProperty(h, sim.objectspecialproperty_detectable_all)
        handles.append(h)
    return handles


def build_obstacle(sim, prim, obs, idx):
    color = obs.get('color', (0.4, 0.4, 0.5))
    if obs['type'] == 'box':
        h = make_shape(sim, prim['box'], obs['size'], obs['position'],
                       color=color, name=f'Obstacle_{idx}')
    elif obs['type'] == 'cylinder':
        d = obs['radius'] * 2
        size = [d, d, obs['height']]
        h = make_shape(sim, prim['cylinder'], size, obs['position'],
                       color=color, name=f'Obstacle_{idx}')
    else:
        raise ValueError(f"Unknown obstacle type: {obs['type']}")
    sim.setObjectSpecialProperty(h, sim.objectspecialproperty_detectable_all)
    return h


def build_target_cube(sim, prim, cfg):
    s = cfg['size']
    h = make_shape(sim, prim['box'], [s, s, s], cfg['position'],
                   color=cfg.get('color', (0.9, 0.2, 0.2)),
                   name='TargetCube',
                   static=False)
    sim.setObjectSpecialProperty(h, sim.objectspecialproperty_detectable_all)
    sim.setObjectInt32Param(h, sim.shapeintparam_static, 0)
    return h


def build_pressure_plate(sim, prim, cfg):
    h = make_shape(sim, prim['box'], cfg['size'], cfg['position'],
                   color=cfg.get('color', (0.2, 0.6, 0.9)),
                   name='PressurePlate',
                   static=True, respondable=False)
    return h

def load_robot(sim, robot_cfg):
    """Load a RoboMaster model preserving its natural pose offsets."""
    model_name = robot_cfg.get('model', 'RoboMasterEP')
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
        robot_cfg['position'][0],
        robot_cfg['position'][1],
        natural_pos[2],
    ]

    # Override only yaw; keep natural roll and pitch (should already be ~0).
    # Accept either a scalar yaw or a [roll, pitch, yaw] triple in degrees.
    orient_deg = robot_cfg.get('orientation_deg', 0)
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
        'Wall', 'Obstacle_', 'TargetCube', 'PressurePlate',
        'RoboMasterEP', 'RoboMasterS1',
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

    print(f"[builder] Connecting to CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require('sim')

    # Resolve primitive type constants from the running sim
    prim = {
        'box':      sim.primitiveshape_cuboid,
        'cylinder': sim.primitiveshape_cylinder,
    }

    if sim.getSimulationState() != sim.simulation_stopped:
        print("[builder] Stopping running simulation...")
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            pass

    print("[builder] Clearing scene...")
    clear_scene(sim)

    print("[builder] Building walls...")
    room = cfg['room']
    build_walls(sim, prim,
                width=room['size'][0],
                depth=room['size'][1],
                height=room['wall_height'],
                thickness=room['wall_thickness'])

    print(f"[builder] Building {len(cfg.get('obstacles', []))} obstacles...")
    for i, obs in enumerate(cfg.get('obstacles', [])):
        build_obstacle(sim, prim, obs, i)

    if 'target_cube' in cfg:
        print("[builder] Placing target cube...")
        build_target_cube(sim, prim, cfg['target_cube'])

    if 'pressure_plate' in cfg:
        print("[builder] Placing pressure plate...")
        build_pressure_plate(sim, prim, cfg['pressure_plate'])

    if 'robot' in cfg:
        print(f"[builder] Loading robot: {cfg['robot']['model']}...")
        try:
            load_robot(sim, cfg['robot'])
        except Exception as e:
            print(f"[builder] WARNING: could not load robot model: {e}")
            print("[builder] Drag the robot from the model browser manually for now.")

    print("[builder] Done. Scene ready.")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: build_scene.py <scenario.json>")
        sys.exit(1)
    main(sys.argv[1])