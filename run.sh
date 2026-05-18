#!/usr/bin/env bash
# One-shot launcher for the escape-room demo.
#
# Usage:
#   pixi shell           # enter the pixi env first
#   ./run.sh             # uses src/escape_room/scenarios/easy.json
#   ./run.sh path/to/scenario.json
#
# Steps:
#   1. Launch CoppeliaSim in the background (skipped if already running)
#   2. Wait for the ZMQ remote API to accept connections
#   3. Build the scene from the scenario JSON
#   4. colcon build the escape_room package and re-source the workspace
#   5. Start the simulation in CoppeliaSim
#   6. Launch robomaster_ros driver in the background, wait for /odom
#   7. Launch the door controller in the background
#   8. Run the discovery launch (mapper + TF) in the foreground (Ctrl-C to stop)
#
# CoppeliaSim and the background ROS processes are left running after Ctrl-C.
# Kill them with: pkill -f robomaster_driver; pkill -f door_controller

set -euo pipefail

SCENARIO="${1:-src/escape_room/scenarios/easy.json}"

_KILL_PATTERNS=(
    "discovery.launch.py"
    "escape_room.nodes"
    "async_slam_toolbox_node"
    "nav2"
    "rviz2"
    "ros2 launch robomaster_ros"
    "robomaster_driver"
    "robot_state_publisher"
    "joint_state_publisher"
    "door_controller"
)

kill_all() {
    trap - EXIT INT TERM
    echo "[run] Stopping all escape_room processes..."
    for pat in "${_KILL_PATTERNS[@]}"; do
        pkill -f "$pat" 2>/dev/null || true
    done
    sleep 2
    for pat in "${_KILL_PATTERNS[@]}"; do
        pkill -9 -f "$pat" 2>/dev/null || true
    done
}


REPO_ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$REPO_ROOT"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "[run] 'ros2' not found on PATH. Enter the pixi env first: 'pixi shell'." >&2
    exit 1
fi

if [[ ! -f "$SCENARIO" ]]; then
    echo "[run] Scenario file not found: $SCENARIO" >&2
    exit 1
fi

# 1. Launch CoppeliaSim if not already up
if pgrep -f -i coppeliaSim >/dev/null 2>&1; then
    echo "[run] CoppeliaSim already running."
else
    echo "[run] Launching CoppeliaSim..."
    pixi run coppelia >/tmp/coppeliaSim.log 2>&1 &
fi

# 2. Wait for the ZMQ remote API to be reachable
echo "[run] Waiting for CoppeliaSim ZMQ remote API..."
for i in $(seq 1 60); do
    if python -c "from coppeliasim_zmqremoteapi_client import RemoteAPIClient; \
                  c = RemoteAPIClient(); c.require('sim')" >/dev/null 2>&1; then
        echo "[run] CoppeliaSim is ready."
        break
    fi
    sleep 1
    if [[ "$i" == "60" ]]; then
        echo "[run] Timed out waiting for CoppeliaSim. See /tmp/coppeliaSim.log." >&2
        exit 1
    fi
done

# 3. Build the scene
echo "[run] Building scene from $SCENARIO..."
python src/escape_room/scripts/build_scene.py "$SCENARIO"

# 4. Build and source the ROS workspace
echo "[run] colcon build (incremental)..."
colcon build --packages-select escape_room
# ROS / colcon setup scripts reference some optional vars; relax 'nounset' here.
set +u
# shellcheck disable=SC1091
source install/setup.sh
set -u

# 5. Start the simulation in CoppeliaSim
echo "[run] Starting simulation..."
python - <<'PY'
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
sim = RemoteAPIClient().require('sim')
# Real-time mode required: robomaster_sim has a 3-second simulation-time
# heartbeat watchdog. Without real-time mode the sim runs ~3x faster than
# wall clock, so the driver's 1 Hz heartbeat times out in ~1 real second,
# dropping all chassis topics (/odom, /imu, position) permanently.
sim.setBoolParam(sim.boolparam_realtime_simulation, True)
if sim.getSimulationState() == sim.simulation_stopped:
    sim.startSimulation()
PY

# 6. Restart the robomaster_ros driver (the build_scene reload disconnects
# the previous one from the simulated robot; reusing it would publish stale
# /odom but no TF). Kill the launch process and all children, then relaunch.
echo "[run] (Re)starting robomaster_ros driver..."
ros2 launch robomaster_ros ep.launch >/tmp/robomaster_ros.log 2>&1 &

echo "[run] Waiting for /odom topic..."
for i in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -q '^/odom$'; then
        echo "[run] /odom is up."
        break
    fi
    sleep 1
    if [[ "$i" == "30" ]]; then
        echo "[run] /odom did not appear in 30s. See /tmp/robomaster_ros.log." >&2
        exit 1
    fi
done

# 7. Launch the door controller in the background
echo "[run] Launching door_controller in background..."
ros2 run escape_room door_controller >/tmp/door_controller.log 2>&1 &

# 8. Discovery launch in the foreground (mapper + static TF)
echo "[run] Launching discovery (SLAM + Nav2 + mission). Ctrl-C to stop..."
trap 'kill_all' EXIT INT TERM
ros2 launch escape_room discovery.launch.py
