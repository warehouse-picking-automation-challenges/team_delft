cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_bin_density)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  apc16delft_msgs
  eigen_conversions
  pcl_conversions
  roscpp
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS apc16delft_msgs eigen_conversions pcl_conversions roscpp
)

include_directories(
	SYSTEM
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_executable(bin_density src/bin_density.cpp)
target_link_libraries(bin_density ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(bin_density_test src/bin_density_test.cpp)
target_link_libraries(bin_density_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
