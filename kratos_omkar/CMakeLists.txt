cmake_minimum_required(VERSION 3.8)
project(kratos_omkar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



install(PROGRAMS
  scripts/q1_hello_world_talker.py
  scripts/q1_hello_world_listener.py
  scripts/q2_s1_talker.py
  scripts/q2_s2_talker.py
  scripts/q3_rover_status_publisher.py
  scripts/q4_clock_publisher.py
  scripts/q4_clock_subscriber.py
  scripts/bonus_move_arm_action_client.py
  scripts/bonus_move_arm_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverStatus.msg"
  "action/MoveArm.action"
  DEPENDENCIES geometry_msgs builtin_interfaces std_msgs action_msgs
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
