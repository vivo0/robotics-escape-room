# Presentation Script — Robotic Escape Room

3 people · 3–4 minutes total

---

## Person 1 — Introduction & Initial Work (~1 min)

Our project is a robotic escape room. A RoboMaster EP robot is placed in an unknown room in CoppeliaSim. It has to autonomously explore the room, find a coloured cube, pick it up, carry it to a pressure plate, drop it there to trigger the door to open, and then drive out.

For the stack we use ROS 2, Nav2 for navigation, and slam_toolbox for building the map in real time. The scene is built programmatically from a JSON scenario file — walls, obstacles, the cube, the plate, the door — all constructed via the CoppeliaSim ZMQ API at startup. The robot uses a 2D lidar for SLAM, camera-based colour detection to locate the three targets, and a gripper for manipulation.

Our initial work covered: building the scene builder, wiring up the ROS 2 / Nav2 / slam_toolbox stack, implementing frontier-based exploration, and writing the mission FSM that sequences explore → pick up → drop → exit.

---

## Person 2 — Preliminary Results & Lidar Challenge (~1 min 15 sec)

In terms of preliminary results: exploration and mapping work reliably. The robot builds a clean occupancy map, Nav2 plans paths around obstacles, and the door controller correctly opens the door when the cube lands on the plate.

We started with a Velodyne, but because it was too complex (3d point cloud, hard to integrate) we then replaced it with a custom Lua sensor that casts 360 rays and publishes `/scan`. That required compiling the CoppeliaSim ROS 2 plugin from source, since it's not bundled by default.

The open challenge is detection. The lidar covers 360° so mapping is fine, but the camera only looks forward — the robot can finish exploring without ever seeing the targets. For now we use simulator ground-truth positions directly, but the goal is to make it work with real camera detection.

---

## Person 3 — Gripper Challenge & Updated Goals (~1 min 15 sec)

The second big challenge was the gripper. The RoboMaster model ships with a gripper script that, when it closes on an object, reparents it — it makes the cube a child of the gripper's attach point. That sounds convenient, but when the gripper opens again, the cube teleports because the attach point and the world origin disagree. We needed friction-based holding instead.

The fix was to inject our own Lua code into the gripper script that silently blocks `sim.setObjectParent`, so the cube stays in the physics simulation and is held purely by contact. We then tuned the cube's mass, friction, and angular damping until the gripper could hold it reliably through turns.

We also hit a version compatibility issue: on our version of CoppeliaSim, the gripper script's signal API works differently, so the Python side couldn't read or set gripper state. We solved this by patching the signal functions at the Lua level to back them with globals we can call directly from Python via ZMQ.

As for updated goals: the full pipeline runs end-to-end. Next we want to improve robustness — better recovery when Nav2 fails or the gripper misses the cube — and test on the medium difficulty scenario with a more complex room layout.

---

*Total: ~3 min 30 sec*
