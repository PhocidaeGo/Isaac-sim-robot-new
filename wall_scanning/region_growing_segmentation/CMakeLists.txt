cmake_minimum_required(VERSION 3.5)
project(region_growing_segmentation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io segmentation filters search features)
find_package(OpenMP)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(region_growing_segmentation_node src/region_growing_point.cpp)

ament_target_dependencies(region_growing_segmentation_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(region_growing_segmentation_node ${PCL_LIBRARIES})

install(TARGETS
  region_growing_segmentation_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
