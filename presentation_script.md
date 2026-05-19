# Presentation Script — Robotic Escape Room

3 people · 3–4 minutes total

---

## Person 1 — Introduction & Initial Work (~1 min)

Our project is a robotic escape room. A RoboMaster EP robot is placed in an unknown room in CoppeliaSim. It has to autonomously explore the room, find a coloured cylinder, pick it up, carry it to a pressure plate, drop it there to trigger the door to open, and then drive out.

For the stack we use ROS 2, Nav2 for navigation, and slam_toolbox for building the map in real time. The scene is built programmatically from a JSON scenario file — walls, obstacles, the cylinder, the plate, the door — all constructed via the CoppeliaSim ZMQ API at startup. The robot uses a 2D lidar for SLAM, camera-based colour detection to locate the three targets, and a gripper for manipulation.

Our initial work covered: building the scene builder, wiring up the ROS 2 / Nav2 / slam_toolbox stack, implementing frontier-based exploration, and writing the mission FSM that sequences explore → pick up → drop → exit.

---

## Person 2 — Preliminary Results & Lidar Challenge (~1 min 15 sec)

In terms of preliminary results: exploration and mapping work reliably. The robot builds a clean occupancy map, Nav2 plans paths around obstacles, and the door controller correctly opens the door when the cylinder lands on the plate.

We started with a Velodyne, but because it was too complex (3d point cloud, hard to integrate) we then replaced it with a custom Lua sensor that casts 360 rays and publishes `/scan`. That required compiling the CoppeliaSim ROS 2 plugin from source, since it's not bundled by default.

The open challenge is detection. The lidar covers 360° so mapping is fine, but the camera only looks forward — the robot can finish exploring without ever seeing the targets. For now we use simulator ground-truth positions directly, but the goal is to make it work with real camera detection.

---

## Person 3 — Gripper Challenge & Updated Goals (~1 min 15 sec)

The second big challenge was getting the gripper to work, and it gave us two distinct problems.

The first problem was control. The gripper uses a built-in Lua script that listens for open and close commands through a CoppeliaSim “signal,” which is basically a shared variable. However, in our version of CoppeliaSim, signals are separated by Lua context. This meant that the signal sent from Python was stored in a different place from the one the gripper script was checking. Even though the signals had the same name, the gripper never received the command.

To solve this, we added a small Lua snippet directly into the gripper script. This snippet stores the open/close commands as normal Lua global variables inside the gripper’s own context and provides two helper functions: one to set the gripper state and one to read it.

The second problem was making the gripper actually hold the cylinder. One simple solution is to attach the cylinder directly to the gripper when it closes. But we wanted the cylinder to be held only through physical contact.

Without attaching it, the cylinder kept slipping out of the gripper. To fix this, we adjusted the physics settings in the scene. We made the cylinder very light (about 20 grams), increased the friction on both the cylinder and the gripper fingers, and added angular damping to stop the cylinder from spinning out. We also automatically detect the correct gripping distance from the gripper’s attach point, so the cylinder is centred between the fingers before the gripper closes.

As for updated goals: the full pipeline now runs end-to-end. Next we want to make it more robust — better recovery when navigation fails or the gripper misses the cylinder — and test it on a harder scenario with a more complex room layout.

---

*Total: ~3 min 30 sec*
