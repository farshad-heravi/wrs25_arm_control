cmake_minimum_required(VERSION 3.10)
project(wrs25_arm_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclpy REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPose.action"
  "action/GripperControl.action"
  "action/VisionCheck.action"
  "action/RobotiqGripperControl.action"
  DEPENDENCIES geometry_msgs
)

add_library(go_to_pose_action_server SHARED src/go_to_pose_action_server.cpp)
target_include_directories(go_to_pose_action_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
ament_target_dependencies(go_to_pose_action_server
  rclcpp
  rclcpp_action
  rclcpp_components
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
)

# Link against the action type support library that we generated in this package
target_link_libraries(go_to_pose_action_server ${PROJECT_NAME}__rosidl_typesupport_cpp)

install(TARGETS go_to_pose_action_server
  DESTINATION lib)

rclcpp_components_register_node(
  go_to_pose_action_server
  PLUGIN "wrs25_arm_actions::GoToPoseActionServer"
  EXECUTABLE go_to_pose_action_server_exe
)

# --- Add rules for the action client executable ---
add_executable(go_to_pose_action_client src/go_to_pose_action_client.cpp)
target_include_directories(go_to_pose_action_client PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
ament_target_dependencies(go_to_pose_action_client
  rclcpp
  rclcpp_action
  geometry_msgs
  visualization_msgs
  tf2
)
target_link_libraries(go_to_pose_action_client ${PROJECT_NAME}__rosidl_typesupport_cpp)
install(TARGETS go_to_pose_action_client
  DESTINATION lib/${PROJECT_NAME})

# --- Add rules for the gripper action server component ---
add_library(gripper_control_action_server SHARED src/gripper_control_action_server.cpp)
target_include_directories(gripper_control_action_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
ament_target_dependencies(gripper_control_action_server
  rclcpp
  rclcpp_action
  rclcpp_components
  trajectory_msgs
  sensor_msgs
)
target_link_libraries(gripper_control_action_server ${PROJECT_NAME}__rosidl_typesupport_cpp)

install(TARGETS gripper_control_action_server
  DESTINATION lib)

rclcpp_components_register_node(
  gripper_control_action_server
  PLUGIN "wrs25_arm_actions::GripperControlActionServer"
  EXECUTABLE gripper_control_action_server_exe
)

# --- Add rules for the gripper action client executable ---
add_executable(gripper_control_action_client src/gripper_control_action_client.cpp)
target_include_directories(gripper_control_action_client PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
ament_target_dependencies(gripper_control_action_client
  rclcpp
  rclcpp_action
)
target_link_libraries(gripper_control_action_client ${PROJECT_NAME}__rosidl_typesupport_cpp)
install(TARGETS gripper_control_action_client
  DESTINATION lib/${PROJECT_NAME})




install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/robotiq_gripper.py
  scripts/robotiq_gripper_action_server.py
  scripts/robotiq_gripper_action_client.py
  scripts/test_robotiq_gripper.py
  DESTINATION lib/${PROJECT_NAME}
)


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

ament_package()