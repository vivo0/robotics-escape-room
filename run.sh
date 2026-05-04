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
#   6. Run the door controller in the foreground (Ctrl-C to stop)
#
# CoppeliaSim is left running after the script exits.

set -euo pipefail

SCENARIO="${1:-src/escape_room/scenarios/easy.json}"
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

# 6. Run the door controller in the foreground (Ctrl-C to stop)
echo "[run] Launching door controller (Ctrl-C to stop)..."
ros2 run escape_room door_controller
