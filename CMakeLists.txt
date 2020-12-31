cmake_minimum_required(VERSION 2.8.3)
project(pcd-extractor)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs visualization_msgs geometry_msgs pcl_conversions pcl_ros)

find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()

catkin_package(
        INCLUDE_DIRS include
)

add_executable(pcd_extractor_node src/pcd_extractor_node.cpp)
target_link_libraries(pcd_extractor_node ${catkin_LIBRARIES})
add_dependencies(pcd_extractor_node ${catkin_EXPORTED_TARGETS})