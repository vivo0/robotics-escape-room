-- ===== EXT GRIPPER CONTROL (injected by build_scene.py) =====
-- Intercepts the gripper script's signal API and backs the two state
-- signals with Lua globals, so the _ext_* helpers called from Python
-- share state with sysCall_init / sysCall_actuation callbacks.
_ext_target_state = 0
_ext_current_state = 0
local _ext_orig_set = sim.setInt32Signal
local _ext_orig_get = sim.getInt32Signal
sim.setInt32Signal = function(name, val)
  if name == target_state_signal then _ext_target_state = val
  elseif name == current_state_signal then _ext_current_state = val
  else _ext_orig_set(name, val) end
end
sim.getInt32Signal = function(name)
  if name == target_state_signal then return _ext_target_state end
  if name == current_state_signal then return _ext_current_state end
  return _ext_orig_get(name)
end
function _ext_set_target(state_int) _ext_target_state = state_int end
function _ext_get_state() return _ext_current_state end

-- Block the stock gripper script from reparenting grasped objects.
-- We want the cube held by physical contact (friction), not by
-- becoming a child of attachPoint. This override is local to the
-- gripper script's Lua state; other scripts are unaffected.
sim.setObjectParent = function(_obj_h, _parent_h, _keep_pose) end
