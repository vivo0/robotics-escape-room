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

# Kill any leftover nodes from a previous run to avoid duplicate TF publishers.
echo "[run] Killing previous escape_room processes..."
pkill -f "discovery.launch.py"        2>/dev/null || true
pkill -f "escape_room.nodes.lidar_node"          2>/dev/null || true
pkill -f "escape_room.nodes.explorer_node"       2>/dev/null || true
pkill -f "escape_room.nodes.color_detector_node" 2>/dev/null || true
pkill -f "escape_room.nodes.door_controller"     2>/dev/null || true
pkill -f "async_slam_toolbox_node"    2>/dev/null || true
pkill -f "nav2"                       2>/dev/null || true
pkill -f "rviz2"                      2>/dev/null || true
pkill -f "ros2 launch robomaster_ros" 2>/dev/null || true
pkill -f robomaster_driver            2>/dev/null || true
pkill -f "robot_state_publisher"      2>/dev/null || true
pkill -f "joint_state_publisher"      2>/dev/null || true
sleep 1

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
        echo "[run] /odom is up; giving driver 3s for TF tree to settle..."
        sleep 3
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
echo "[run] Launching discovery (mapper + velodyne TF). Ctrl-C to stop..."
ros2 launch escape_room discovery.launch.py
