cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_grasp_synthesizer)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	apc16delft_msgs
	apc16delft_util
	dr_eigen
	dr_param
	pcl_conversions
	pcl_ros
	cmake_modules
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
	INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
	LIBRARIES ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES}
	CATKIN_DEPENDS
	roscpp
	apc16delft_msgs
	apc16delft_util
	dr_eigen
	dr_param
	pcl_conversions
	pcl_ros
	cmake_modules
)

include_directories(SYSTEM
	include/${PROJECT_NAME}
	SYSTEM
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_executable(grasp_synthesizer src/grasp_synthesizer.cpp src/deformable_grasps.cpp)
add_dependencies(grasp_synthesizer ${catkin_EXPORTED_TARGETS})

target_link_libraries(grasp_synthesizer
	${catkin_LIBRARIES}
	${Eigen_LIBRARIES}
	${Boost_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(deformable_grasps_test src/deformable_grasps_test.cpp src/deformable_grasps.cpp)
target_link_libraries(deformable_grasps_test
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
