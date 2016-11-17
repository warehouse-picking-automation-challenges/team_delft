cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_place_synthesizer)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
	roscpp
	apc16delft_msgs
	apc16delft_util
	dr_eigen
	dr_param
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	roscpp
	apc16delft_msgs
	apc16delft_util
	dr_eigen
	dr_param
)

include_directories(
	include/${PROJECT_NAME}
	SYSTEM
	${catkin_INCLUDE_DIRS}
)

add_executable(place_synthesizer src/place_synthesizer.cpp)

target_link_libraries(place_synthesizer
	${catkin_LIBRARIES}
)
