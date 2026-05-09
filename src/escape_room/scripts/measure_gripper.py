#!/usr/bin/env python3
"""Print the world Z of the gripper proximity sensor and fingers, so
we can size the target cube/key to be reachable.

Run with the scene built and simulation in Play (or stopped, doesn't
matter — these are static positions).
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def main() -> None:
    sim = RemoteAPIClient().require('sim')
    model = sim.getObject('/RoboMasterEP')
    aliases = {sim.getObjectAlias(int(h), 0): int(h)
               for h in sim.getObjectsInTree(model)}

    targets = ('attachProxSensor', 'attachPoint', 'gripper_link_respondable',
               'left_gripper_5_respondable', 'right_gripper_5_respondable',
               'Prismatic_joint')
    print('world XYZ of relevant gripper objects:')
    for alias in targets:
        h = aliases.get(alias)
        if h is None:
            continue
        x, y, z = sim.getObjectPosition(h, -1)
        print(f'  {alias:30s}  ({x:+.3f}, {y:+.3f}, {z:+.3f})')

    cube = sim.getObject('/TargetCube')
    cx, cy, cz = sim.getObjectPosition(cube, -1)
    bb_min_z = sim.getObjectFloatParam(cube, sim.objfloatparam_objbbox_min_z)
    bb_max_z = sim.getObjectFloatParam(cube, sim.objfloatparam_objbbox_max_z)
    print(f'\nTargetCube center=({cx:+.3f}, {cy:+.3f}, {cz:+.3f}), '
          f'top_z = {cz + bb_max_z:.3f}, bottom_z = {cz + bb_min_z:.3f}')

    # Recommendation: top of cube should reach attachProxSensor Z
    h = aliases.get('attachProxSensor')
    if h is not None:
        sx, sy, sz = sim.getObjectPosition(h, -1)
        gap = sz - (cz + bb_max_z)
        print(f'\nattachProxSensor is at z={sz:.3f}; cube top is at '
              f'{cz + bb_max_z:.3f}; gap = {gap:.3f} m.')
        if gap > 0.005:
            print(f'  → cube needs to be ~{gap:.2f} m taller, or its '
                  f'center raised by ~{gap:.2f} m.')


if __name__ == '__main__':
    main()
