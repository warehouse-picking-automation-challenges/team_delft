cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_motion)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
	roscpp
	apc16delft_msgs
	moveit_core
	geometric_shapes
	moveit_ros_planning_interface
	moveit_msgs
	shape_msgs
	std_msgs
	apc16delft_grasp_synthesizer
	dr_eigen
	dr_param
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	roscpp
	moveit_ros_planning_interface
	moveit_msgs
	apc16delft_msgs
	apc16delft_grasp_synthesizer
	dr_eigen
	dr_param
)

include_directories(
	include/${PROJECT_NAME}
	SYSTEM
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/master_pose.cpp
	src/motion_executor.cpp
	src/trajectory_tracker.cpp
	src/trajectory_cache.cpp
	src/yaml.cpp
)

add_library(manipulation_planner_lib
	src/master_pose.cpp
	src/manipulation_planner.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	yaml-cpp
)

add_executable(motion_executor src/motion_executor.cpp)
add_executable(manipulation_planner src/manipulation_planner.cpp)
add_executable(pose_generator src/pose_generator.cpp)
add_executable(scene_updater src/scene_updater.cpp)
add_executable(trajectory_generator src/trajectory_generator.cpp)
add_executable(trajectory_visualizer src/trajectory_visualizer.cpp)

target_link_libraries(motion_executor
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

target_link_libraries(manipulation_planner
	manipulation_planner_lib
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

target_link_libraries(pose_generator
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

target_link_libraries(scene_updater
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

target_link_libraries(trajectory_generator
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

target_link_libraries(trajectory_visualizer
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)
