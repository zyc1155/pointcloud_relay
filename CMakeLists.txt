cmake_minimum_required(VERSION 3.10)
project(pointcloud_relay)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  std_srvs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pointcloud_relay
 CATKIN_DEPENDS roscpp sensor_msgs std_srvs pcl_ros pcl_conversions)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/pointcloud_relay_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

