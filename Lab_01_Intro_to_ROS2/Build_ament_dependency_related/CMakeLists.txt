cmake_minimum_required(VERSION 3.8)
project(lab1_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker_cpp_node src/talker.cpp)
add_executable(relay_cpp_node src/relay.cpp)
add_executable(test_params_rclcpp src/param.cpp)

ament_target_dependencies(talker_cpp_node rclcpp)
ament_target_dependencies(relay_cpp_node rclcpp)
ament_target_dependencies(test_params_rclcpp rclcpp)

ament_target_dependencies(talker_cpp_node ackermann_msgs)
ament_target_dependencies(relay_cpp_node ackermann_msgs)

ament_target_dependencies(talker_cpp_node std_msgs)
ament_target_dependencies(relay_cpp_node std_msgs)


install(TARGETS talker_cpp_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS relay_cpp_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_params_rclcpp DESTINATION lib/${PROJECT_NAME})

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
