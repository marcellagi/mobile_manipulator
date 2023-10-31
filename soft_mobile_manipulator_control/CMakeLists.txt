cmake_minimum_required(VERSION 3.8)
project(soft_mobile_manipulator_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



add_executable(joint_control_node src/joint_control_node.cpp)
ament_target_dependencies(joint_control_node rclcpp sensor_msgs control_msgs trajectory_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY 
  launch
  config
  helper_funcs
  src
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  joint_control_node
  DESTINATION lib/${PROJECT_NAME})

  # Install Python modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  src/imu_to_euler_node.py
  src/tf.py
  helper_funcs/curves.py
  helper_funcs/plot_segment_general.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
