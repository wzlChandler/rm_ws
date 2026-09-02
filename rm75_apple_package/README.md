# RM75 ROH Current-Pose Apple Scene

This package reuses the exact URDF and live TF tree from `rm75_roh_moveit_config/launch/realrobot.launch`. It sends no control request and displays an apple in the current hand pose.

Start `realrobot.launch` first so `/joint_states` is available.

```bash
source ~/rm_ws/devel/setup.bash
roslaunch rm75_roh_apple_grasp_sim current_pose_apple.launch
```
