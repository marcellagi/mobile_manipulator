from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    
    world_file = PathJoinSubstitution(
        [FindPackageShare("husky_gazebo"),
        "worlds",
        "clearpath_playpen.world"],
    )

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("husky_gazebo"),
        "launch",
        "gazebo.launch.py"],
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={'world_path': world_file}.items(),
    )

    # tf_publisher = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments = ['--x', '0.0', '--y', '0.0', '--z', '0.67', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'manipulator_link', '--child-frame-id', 'first_segment']
    # )

    # tf_publisher2 = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments = ['--x', '0.0', '--y', '0.0', '--z', '0.704', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'first_segment', '--child-frame-id', 'sec_segment']
    # )

    ld = LaunchDescription()
    ld.add_action(gazebo_sim)
    # ld.add_action(tf_publisher)
    # ld.add_action(tf_publisher2)

    return ld
