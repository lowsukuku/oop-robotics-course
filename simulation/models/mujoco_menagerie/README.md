# Model attribution and adaptations

The robot and gripper assets are based on these MuJoCo Menagerie models:

- [Universal Robots UR5e](https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur5e), distributed under the BSD 3-Clause License.
- [Robotiq 2F-85](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotiq_2f85), distributed under the BSD 2-Clause License.

The original license, README, and changelog files are retained in each model
directory.

Local adaptations:

- `robotiq_2f85/2f85.xml`: a `tcp` site between the finger pads (the tool centre
  point of the ball-sorting task) and a tendon-driven position actuator named after
  the driver joint, so that `mujoco_ros2_control` exposes it as one joint; its
  `forcerange` limits the grasp force to about 32 N per pad.
- `universal_robots_ur5e/ur5e.xml`: the scene of the archived cube pick-and-place
  prototype (`archive/ur5_pick_place`): the arm with the gripper attached, a floor,
  a cube, a goal marking and a `home` keyframe.
- The ball-sorting scene (`simulation/ball_sorting/scene.xml`) reuses the UR5e body
  tree and meshes from this directory and attaches the gripper model.
