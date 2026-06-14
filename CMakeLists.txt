cmake_minimum_required(VERSION 3.5)
project(turtlebot3_nav)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FindWall.srv"
  DEPENDENCIES std_msgs
)

# Add executables
add_executable(wall_follower src/wall_follower.cpp)
ament_target_dependencies(wall_follower
 rclcpp
 geometry_msgs
 sensor_msgs
)

target_link_libraries(wall_follower
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

add_executable(find_wall_server src/find_wall_server.cpp)
ament_target_dependencies(find_wall_server
 rclcpp
 geometry_msgs
 sensor_msgs
)

target_link_libraries(find_wall_server
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Install the executable so 'ros2 run' can find it
install(TARGETS
  wall_follower
  find_wall_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
