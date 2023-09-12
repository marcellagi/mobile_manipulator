# mobile_manipulator


## How to use

```sh
$ ros2 launch soft_mobile_manipulator_control husky_playpen.launch.py
```

```sh
$ ros2 launch cobra_control cobra_teleop.launch.py 
```

```sh
$ ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names: [cobra_body_1_joint, cobra_body_1_aux_joint, cobra_body_2_joint, cobra_body_2_aux_joint, cobra_body_3_joint, cobra_body_3_aux_joint, cobra_body_4_joint, cobra_body_4_aux_joint, cobra_body_5_joint, cobra_body_5_aux_joint, cobra_body_6_joint, cobra_body_6_aux_joint]
points: 
- positions: [0.0, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 0.0]
  time_from_start: {sec: 1, nanosec: 0}" --once