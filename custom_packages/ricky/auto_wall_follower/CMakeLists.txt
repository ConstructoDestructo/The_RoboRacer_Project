cmake_minimum_required(VERSION 3.8)
project(auto_wall_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(auto_wall_follower src/wall_node.cpp)

ament_target_dependencies(auto_wall_follower rclcpp)
ament_target_dependencies(auto_wall_follower sensor_msgs)
ament_target_dependencies(auto_wall_follower nav_msgs)
ament_target_dependencies(auto_wall_follower ackermann_msgs)
ament_target_dependencies(auto_wall_follower visualization_msgs)

install(TARGETS auto_wall_follower DESTINATION lib/${PROJECT_NAME})

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
