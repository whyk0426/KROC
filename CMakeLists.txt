cmake_minimum_required(VERSION 3.8)
project(ros2_turtlebot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


set(ROS2_PACKAGES
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  visualization_msgs)

include_directories(${PROJECT_SOURCE_DIR}/include)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(cmd_publisher_node
        src/cmd_publisher_node.cpp
        src/cmd_publisher.cpp)
ament_target_dependencies(cmd_publisher_node ${ROS2_PACKAGES})
install(TARGETS cmd_publisher_node
  DESTINATION lib/${PROJECT_NAME})

#install launch file
install(DIRECTORY launch mesh config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
