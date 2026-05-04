# Run instructions

## Prerequisites

Remove python 3.13 from current nix direct environment if it creates problems

## To install

Add to `pixi-toolchain.cmake`

```
set(CMAKE_OSX_SYSROOT "/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk")
```

then run

```
pixi install
```

if it fails because of old paths cached

```
rm -rf ~/Library/Caches/rattler/cache/uv-cache
```

## To build

Add to `pixi.toml`

```
build = "colcon build --symlink-install --cmake-args -DCMAKE_OSX_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk
```

then run

```
pixi run build
```

this is the expected output

```
Finished <<< robomaster_msgs [5.11s]
Starting >>> robomaster_ros
Finished <<< robomaster_ros [1.54s]
Starting >>> robomaster_example
Finished <<< robomaster_example [1.56s]

Summary: 6 packages finished [8.36s]
  1 package had stderr output: robomaster_msgs
```

## Terminal 1

```
cd project root
pixi shell
source install/setup.zsh
pixi run coppelia
```

## Terminal 2

if it fails check that

```
ls install/robomaster_msgs/lib/python3.10/site-packages/robomaster_msgs/
```

doesnt show files created with python 3.13

```
cd project root
pixi shell
source install/setup.zsh
ros2 launch robomaster_ros ep.launch name:=/rm0
ros2 launch robomaster_example ep_tof.launch name:=/rm0
```

## Terminal 3

```
cd project root
pixi shell
source install/setup.zsh
ros2 launch robomaster_example controller.launch name:=/rm0
ros2 launch robomaster_task open_loop_eight.launch name:=/rm0
ros2 launch robomaster_task wall_align.launch name:=/rm0
ros2 launch robomaster_task standard.launch name:=/rm0
ros2 launch robomaster_task advanced.launch name:=/rm0
```
