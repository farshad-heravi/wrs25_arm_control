cmake_minimum_required(VERSION 3.10.2)
project(wrs_cell_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    rviz
    meshes
    scripts
  DESTINATION
    share/${PROJECT_NAME}
)

# Install Python script with executable permissions
install(
  PROGRAMS
    scripts/gripper_joint_state_publisher.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
