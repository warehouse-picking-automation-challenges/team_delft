cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_gripper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	apc16delft_msgs
	apc16delft_util
	motoman_msgs
)

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	roscpp
	apc16delft_msgs
	apc16delft_util
	motoman_msgs
)

include_directories(
	SYSTEM
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/gripper_driver.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(gripper_driver src/gripper_driver.cpp)

target_link_libraries(gripper_driver ${catkin_LIBRARIES})
