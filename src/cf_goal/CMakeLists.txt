cmake_minimum_required(VERSION 3.8)
project(cf_goal)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(crazyflie_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

#Add C++ executable
add_executable(cf_goal src/goal.cpp)
ament_target_dependencies(cf_goal rclcpp std_msgs geometry_msgs)

add_executable(cf_trigo src/trigo.cpp)
ament_target_dependencies(cf_trigo rclcpp geometry_msgs)

add_executable(cmd_position src/cmd_position.cpp)
target_include_directories(cmd_position PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(cmd_position PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(cmd_position
  rclcpp
  crazyflie_interfaces
  Eigen3
  )

install(TARGETS
  cf_goal
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  cf_trigo
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  cmd_position
  DESTINATION lib/${PROJECT_NAME}
)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


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