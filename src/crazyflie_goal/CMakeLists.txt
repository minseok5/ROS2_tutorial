cmake_minimum_required(VERSION 3.8)
project(crazyflie_goal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(crazyflie_cmd_publisher src/crazyflie_cmd_publisher.cpp)
ament_target_dependencies(crazyflie_cmd_publisher rclcpp std_msgs geometry_msgs)
install(TARGETS
  crazyflie_cmd_publisher
  DESTINATION lib/${PROJECT_NAME})

add_executable(tml src/move_to_goal.cpp)
ament_target_dependencies(tml rclcpp std_msgs geometry_msgs)
install(TARGETS
  tml
  DESTINATION lib/${PROJECT_NAME})

add_executable(hovering src/hovering.cpp)
ament_target_dependencies(hovering rclcpp crazyflie_interfaces)
install(TARGETS
  hovering
  DESTINATION lib/${PROJECT_NAME})


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
