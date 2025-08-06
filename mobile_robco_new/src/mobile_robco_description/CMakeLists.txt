cmake_minimum_required(VERSION 3.5)
project(mobile_robco_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_localization REQUIRED)

# Declare the executable target
add_executable(rgl_cloud_configurator_node src/rgl_cloud_configurator.cpp)
add_executable(track_odom_publisher_node src/track_odom_publisher.cpp)

# Specify libraries and include directories
ament_target_dependencies(rgl_cloud_configurator_node rclcpp sensor_msgs)
ament_target_dependencies(track_odom_publisher_node rclcpp nav_msgs)

# Install the executable
install(TARGETS
  rgl_cloud_configurator_node
  track_odom_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install URDF, meshes, and other folders into the share directory.
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY RGL_Gazebo/
  DESTINATION share/${PROJECT_NAME}/RGL_Gazebo
)

ament_package()
