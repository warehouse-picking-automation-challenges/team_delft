cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_vision)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	sensor_msgs
	pcl_conversions
	pcl_ros
	apc16delft_msgs
	apc16delft_util
	dr_param
)

catkin_package()

include_directories(
	SYSTEM
	${catkin_INCLUDE_DIRS}
)

add_executable(point_cloud_crop src/point_cloud_crop.cpp)
add_executable(occlusion_detection src/occlusion_detection.cpp)
target_link_libraries(point_cloud_crop ${catkin_LIBRARIES})
target_link_libraries(occlusion_detection ${catkin_LIBRARIES})
