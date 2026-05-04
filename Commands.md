## All-in-one (inside `pixi shell`):
./run.sh                                      # uses easy.json by default
./run.sh src/escape_room/scenarios/easy.json  # or pass an explicit scenario

---

## Manual flow

## Terminal 1 (Open Coppelia):
pixi run coppelia

## Terminal 2 (Create the scene):
python src/escape_room/scripts/build_scene.py src/escape_room/scenarios/easy.json

## Terminal 3 (Build the workspace, only first time / after code changes):
colcon build --packages-select escape_room
source install/setup.sh

## Terminal 3 (Run the door controller — press Play in Coppelia first):
ros2 run escape_room door_controller