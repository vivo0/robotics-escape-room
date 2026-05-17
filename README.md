# robotics-escape-room

CoppeliaSim + ROS 2 escape-room demo: a RoboMaster EP explores an unknown
room, picks up a key, drops it on a pressure plate, and drives out the
opened door. See [`CLAUDE.md`](CLAUDE.md) for architecture details.

## Setup (macOS)

### 1. Install CoppeliaSim

Install CoppeliaSim into `/Applications/coppeliaSim.app` (drag-and-drop
from the official ZIP).

### 2. Install the ROS 2 plugin

CoppeliaSim ships without ROS 2 support. A compiled
`libsimROS2.dylib` for macOS is bundled in this repo under `docs/`.
Copy it into the CoppeliaSim plugins folder:

```bash
cp docs/libsimROS2.dylib /Applications/coppeliaSim.app/Contents/MacOS/
```

Alternatively, if you plan to rebuild the plugin yourself, symlink the
build output instead:

```bash
ln -s "$(pwd)/build/sim_ros2_interface/libsimROS2.dylib" \
      /Applications/coppeliaSim.app/Contents/MacOS/libsimROS2.dylib
```

Verify the plugin loads by launching CoppeliaSim and looking for the
`simROS2` entry in the plugin list (no error in the console at startup).

### 3. Enter the pixi environment

All commands must run inside the pixi env:

```bash
pixi shell
```

## Run the demo

```bash
./run.sh                                         # default scenario (easy.json)
./run.sh src/escape_room/scenarios/easy.json     # explicit scenario
```

`run.sh` launches CoppeliaSim, builds the scene, builds the ROS workspace,
starts the simulation, and brings up the navigation stack and mission FSM.
