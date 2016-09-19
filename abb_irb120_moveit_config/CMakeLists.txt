cmake_minimum_required(VERSION 2.8.3)
project(abb_irb120_moveit_config)

find_package(catkin REQUIRED)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

SET(SRCS
	
	/home/carles/tfm_ws/src/perception/src/perception_node.cpp
)

SET(SRCS2

	/home/carles/tfm_ws/src/position/src/position_node.cpp
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(perception_node_ ${SRCS})
add_executable(position_node_ ${SRCS2})

target_link_libraries(perception_node_ ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(position_node_ ${catkin_LIBRARIES})

