cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_pose_estimation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	apc16delft_msgs
	apc16delft_util
	cv_bridge
	eigen_conversions
	pcl_conversions
	pcl_ros
	dr_eigen
	dr_param
)

catkin_package()

include_directories(
	SYSTEM
	${catkin_INCLUDE_DIRS}
)

add_executable(pose_estimation src/pose_estimation.cpp)
add_executable(pose_estimation_test src/pose_estimation_test.cpp)
add_executable(bin_pose_estimation src/bin_pose_estimation.cpp)

target_link_libraries(pose_estimation ${catkin_LIBRARIES})
target_link_libraries(pose_estimation_test ${catkin_LIBRARIES})
target_link_libraries(bin_pose_estimation ${catkin_LIBRARIES})
