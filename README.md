# mobile_manipulator


## How to use
```sh
$ git clone -b 2-section git@github.com:Brazilian-Institute-of-Robotics/soft_gazebo_cobra_simulation.git 
```
```sh
$ git clone -b humble-devel git@github.com:ros-visualization/interactive_marker_twist_server.git
```

```sh
$ git clone -b humble-devel git@github.com:husky/husky.git 
```
```sh
$ git clone git@github.com:marcellagi/mobile_manipulator.git
```

```sh
$ ros2 launch soft_mobile_manipulator_control husky_playpen.launch.py
```

## remote control
```sh
$ ros2 run soft_mobile_manipulator_control joint_control_node
```

## remote control
```sh
$ ros2 run soft_mobile_manipulator_control imu_to_euler_node.py
```

## topic 
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
```