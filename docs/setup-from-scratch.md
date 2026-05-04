# Setup from scratch

The following instructions walk you through the process of setting up a Pixi project from scratch.
We leave them here as a reference, in case you find them useful for future projects.

**They are not needed if you clone this repository and set it up as described in the README!**

---

## Install ROS
Set up an empty Pixi project and install ROS2 as described in the ROS Installation Guide on iCorsi. Next, enter the project directory and follow the instructions below.

## 🤖 Installing RoboMaster SDK

This is a fork of the official DJI RoboMaster Python API modified to solve some issues in using these robots with ROS2.

Inside the `<ROS_PROJECT_FOLDER>`.

```bash
pixi shell
pixi add libopus
pixi add --pypi "robomaster@git+https://github.com/jeguzzi/RoboMaster-SDK.git"
pixi add --pypi "libmedia_codec@git+https://github.com/jeguzzi/RoboMaster-SDK.git#subdirectory=lib/libmedia_codec"
```

---

## 💻 Installing RoboMaster Sim

This package provides RoboMaster models for CoppeliaSim. It includes 3D graphics, dynamics and sensors models and a control interface that mimics the real RoboMaster's network protocol.

Inside the `<ROS_PROJECT_FOLDER>`.

### 1. Install the Dependencies

```bash
pixi shell
pixi add spdlog boost cmake ffmpeg libxslt xmlschema
```

### 2. Configure environment

Create a `pixi-toolchain.cmake` file in your `<ROS_PROJECT_FOLDER>` with the following content:

```cmake
# Force CMake to look for all shared libraries (spdlog, Boost, etc.) inside the pixi environment
set(CMAKE_FIND_ROOT_PATH "$ENV{CONDA_PREFIX}")
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Force Coppelia to load the exact version of the dylibs used at build time, instead of letting
# the OSes to resolve a version at runtime with their usual opaque logic.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH True)
```

Add the following to your `pixi.toml` so that CMake finds the toolchain file:

```toml
[activation.env]
CMAKE_TOOLCHAIN_FILE = "$PIXI_PROJECT_ROOT/pixi-toolchain.cmake"
```

### 3. Clone the repo

```bash
mkdir -p src
cd src
git clone git@github.com:jeguzzi/robomaster_sim.git
```

### 4. Build

```bash
cd ..
colcon build --symlink-install
```

The symlink install option install Python ROS2 packages using symlink from the install to the src folder making them editable. This only applies to Python files and not to config or launch files.

### 5. Test your installation

1. Open CoppeliaSim.
2. Add RoboMaster via: `Model browser (the left sidebar) -> robots -> mobile -> RoboMasterEP`
3. Press play and ensure no errors are printed in the Coppelia log (Bottom).

With CoppeliaSim running and a Robomaster in the scene.

```bash
cd src/robomaster_sim/examples
pixi shell
python discover.py
```

This script should find the RoboMaster in the scene, print some info about it, and close itself.

---

## 🚀 Installing RoboMaster ROS

This package provides ROS drivers for the RoboMaster, whether real or simulated. Inside the `<ROS_PROJECT_FOLDER>`:

```bash
pixi shell
pixi add ros-humble-xacro ros-humble-launch-xml ros-humble-cv-bridge ros-humble-launch-testing-ament-cmake ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-joy
pixi add --pypi numpy-quaternion
```

### 1. Clone the repo

```bash
cd src
git clone https://github.com/jeguzzi/robomaster_ros.git
```

### 2. Build

```bash
cd ..
colcon build --symlink-install
```

### 3. Launch

Source the setup file:

```bash
source install/setup.zsh
```

or

```bash
source install/setup.bash
```

Sourcing this file add your custom packages to the available ROS2 packages (sourced using `pixi shell`) allowing you to launch them.

Launch the RoboMaster driver ROS nodes:

```bash
ros2 launch robomaster_ros ep.launch
```

### 4. Test

In another terminal, inside the `<ROS_PROJECT_FOLDER>`.

```bash
pixi shell
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

The robot should start to rotate counter-clockwise.

Use this command to stop it:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 🔗 Installing simExtROS2

Finally, let's install the CoppeliaSim ROS2 plugin. This allows to interact with the simulation from ROS2, providing topics and services to start and stop the simulation, read the simulation clock, etc.

### 1. Clone the repo

```bash
cd src
git clone git@github.com:jeguzzi/simExtROS2.git -b iron-4.6
```

### 2. Build

```bash
pixi shell
colcon build --symlink-install
```
