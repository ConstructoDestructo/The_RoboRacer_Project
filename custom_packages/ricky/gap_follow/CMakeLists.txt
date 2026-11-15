cmake_minimum_required(VERSION 3.8)
project(gap_follow)

# Build optimized unless told otherwise: the ghost-car checks are a lot of math.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# the node: src/reactive_node.cpp -> executable "reactive_node"
add_executable(reactive_node src/reactive_node.cpp)
target_compile_features(reactive_node PUBLIC cxx_std_17)
ament_target_dependencies(reactive_node
  rclcpp
  sensor_msgs
  ackermann_msgs
  geometry_msgs
  visualization_msgs
)

install(TARGETS reactive_node
  DESTINATION lib/${PROJECT_NAME})

# launch files (required by the lab's autograder)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
