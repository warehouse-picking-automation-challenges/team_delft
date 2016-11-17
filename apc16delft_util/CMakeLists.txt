cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_util)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	dr_eigen
	dr_param
	tf
	tf2
	apc16delft_msgs
	pcl_ros
	pcl_conversions
	std_srvs
	moveit_ros_planning
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	roscpp
	roslib
	dr_eigen
	dr_param
	pcl_ros
	pcl_conversions
	std_srvs
)

include_directories(
	include/${PROJECT_NAME}
	SYSTEM
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/managed_node.cpp
	src/lifecycle_state_machine.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(example_node src/example_node.cpp)

add_executable(static_pointcloud_publisher src/static_pointcloud_publisher.cpp)

target_link_libraries(example_node
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

target_link_libraries(static_pointcloud_publisher
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

