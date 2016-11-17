cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_camera)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	apc16delft_msgs
	apc16delft_util
	dr_ensenso
	dr_eigen
	cv_bridge
	pcl_conversions
	std_srvs
	image_transport
)

catkin_package()

include_directories(
	SYSTEM
	${catkin_INCLUDE_DIRS}
)

add_executable(ensenso_camera src/ensenso_camera.cpp)
target_link_libraries(ensenso_camera ${catkin_LIBRARIES})
