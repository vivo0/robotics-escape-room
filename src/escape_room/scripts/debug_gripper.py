#!/usr/bin/env python3
"""Probe how to drive the RoboMaster gripper from outside CoppeliaSim.

Run with the simulation **in Play**:
    pixi run python src/escape_room/scripts/debug_gripper.py

Steps:
  1. Confirms the simulation is actually advancing.
  2. Resolves the gripper / model / Prismatic joint handles.
  3. Tries to retrieve ``rm_handle`` (returned by simRobomaster.create_ep)
     from the RoboMasterEP child script via executeScriptString and
     callScriptFunction.
  4. If we get rm_handle, calls ``simRobomaster.set_gripper_target``
     and reports whether the prismatic joint actually moves.
"""
from __future__ import annotations
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def main() -> None:
    sim = RemoteAPIClient().require('sim')

    # 1) Simulation state -------------------------------------------------
    state = sim.getSimulationState()
    print(f'simulation state: {state} '
          f'(advancing_running expected = {sim.simulation_advancing_running})')
    if state != sim.simulation_advancing_running:
        print('!! simulation is NOT running — press Play in CoppeliaSim')
        return

    # 2) Handles ----------------------------------------------------------
    model = sim.getObject('/RoboMasterEP')
    aliases = {sim.getObjectAlias(int(h), 0): int(h)
               for h in sim.getObjectsInTree(model)}
    gripper_link_h = aliases.get('gripper_link_respondable')
    prismatic_h = aliases.get('Prismatic_joint')
    print(f'model={model}, gripper_link_respondable={gripper_link_h}, '
          f'Prismatic_joint={prismatic_h}')

    # 3) Get rm_handle ----------------------------------------------------
    script_h = sim.getScript(1, model)   # 1 = child script
    print(f'RoboMasterEP child script: {script_h}')

    rm_handle = None

    # 3a) try executeScriptString
    try:
        ret = sim.executeScriptString('return rm_handle', script_h)
        print(f'executeScriptString("return rm_handle") -> {ret!r}')
        if isinstance(ret, (int, float)) and ret >= 0:
            rm_handle = int(ret)
    except Exception as e:
        print(f'executeScriptString failed: {e}')

    # 3b) inject getter and call it
    if rm_handle is None:
        try:
            src = sim.getScriptStringParam(
                script_h, sim.scriptstringparam_text)
            if 'function get_rm_handle' not in src:
                src = src + ('\nfunction get_rm_handle()\n'
                             '  return rm_handle\nend\n')
                sim.setScriptStringParam(
                    script_h, sim.scriptstringparam_text, src)
                print('injected get_rm_handle() into RoboMasterEP script '
                      '(simulation needs restart for it to take effect)')
            ret = sim.callScriptFunction('get_rm_handle', script_h)
            print(f'callScriptFunction("get_rm_handle") -> {ret!r}')
            if isinstance(ret, (int, float)) and ret >= 0:
                rm_handle = int(ret)
        except Exception as e:
            print(f'callScriptFunction failed: {e}')

    if rm_handle is None:
        print('!! could not retrieve rm_handle — stop and restart the '
              'simulation in CoppeliaSim, then re-run this script')
        return
    print(f'rm_handle = {rm_handle}')

    # 4) Ensure the helper bundle (signal shim + setters + diag) is
    #    present in the gripper_link_respondable child script. The
    #    marker is the _ext_diag function — older partial injections
    #    won't have it and will be replaced.
    gripper_script_h = sim.getScript(1, gripper_link_h)
    print(f'gripper_link_respondable child script: {gripper_script_h}')
    src = sim.getScriptStringParam(
        gripper_script_h, sim.scriptstringparam_text)
    helpers = (
        '\n-- ===== EXT GRIPPER CONTROL (injected) =====\n'
        '_ext_target_state = 0\n'
        '_ext_current_state = 0\n'
        'local _ext_orig_set = sim.setInt32Signal\n'
        'local _ext_orig_get = sim.getInt32Signal\n'
        'sim.setInt32Signal = function(name, val)\n'
        '  if name == target_state_signal then _ext_target_state = val\n'
        '  elseif name == current_state_signal then _ext_current_state = val\n'
        '  else _ext_orig_set(name, val) end\n'
        'end\n'
        'sim.getInt32Signal = function(name)\n'
        '  if name == target_state_signal then return _ext_target_state end\n'
        '  if name == current_state_signal then return _ext_current_state end\n'
        '  return _ext_orig_get(name)\n'
        'end\n'
        'function _ext_set_target(state_int)\n'
        '  _ext_target_state = state_int\n'
        'end\n'
        'function _ext_get_state()\n'
        '  return _ext_current_state\n'
        'end\n'
        'function _ext_diag()\n'
        '  return string.format(\n'
        '    "tgt_sig=%s cur_sig=%s h=%s tgt=%s cur=%s pos=%s",\n'
        '    tostring(target_state_signal),\n'
        '    tostring(current_state_signal),\n'
        '    tostring(h),\n'
        '    tostring(_ext_target_state),\n'
        '    tostring(_ext_current_state),\n'
        '    tostring(h and sim.getJointPosition(h) or "nil"))\n'
        'end\n'
    )
    if 'function _ext_diag' not in src:
        sim.setScriptStringParam(
            gripper_script_h, sim.scriptstringparam_text, src + helpers)
        print('!! injected gripper helpers + signal shim into the '
              'gripper_link_respondable child script.')
        print('   STOP and PLAY the simulation in CoppeliaSim, then '
              're-run this script.')
        return

    # 5) Diagnostic + drive ----------------------------------------------
    def joint_pos() -> float:
        try:
            return float(sim.getJointPosition(prismatic_h))
        except Exception:
            return float('nan')

    try:
        diag = sim.callScriptFunction('_ext_diag', gripper_script_h)
    except Exception as e:
        diag = f'<error: {e!r}>'
    print(f'\n[before any drive] {diag}')

    State = {'pause': 0, 'open': 1, 'close': 2}
    for label, value in (('close', State['close']), ('open', State['open'])):
        print(f'\ncalling _ext_set_target({value})  # {label}')
        try:
            sim.callScriptFunction(
                '_ext_set_target', gripper_script_h, value)
        except Exception as e:
            print(f'  callScriptFunction failed: {e!r}')
            continue
        time.sleep(2.0)
        try:
            diag = sim.callScriptFunction('_ext_diag', gripper_script_h)
        except Exception as e:
            diag = f'<error: {e!r}>'
        print(f'  prismatic = {joint_pos():.4f}  diag: {diag}')


if __name__ == '__main__':
    main()
