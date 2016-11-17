cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_calibration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	dr_eigen
	dr_param
	roscpp
	geometry_msgs
	tf
	moveit_ros_planning_interface
	apc16delft_msgs
)

catkin_package()

include_directories(
	SYSTEM
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/calibration_coordinator.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
