# mobile_manipulator


## How to use

```sh
$ ros2 launch soft_mobile_manipulator_control husky_playpen.launch.py
```

```sh
$ ros2 launch cobra_description test_cobra.launch.py
```

```sh
$ ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.7,0.7,0.7,0.7,0.7,0.7]"