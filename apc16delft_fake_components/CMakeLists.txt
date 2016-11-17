cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_fake_components)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	apc16delft_msgs
	apc16delft_util
	dr_eigen
	dr_param
	cv_bridge
	pcl_conversions
	roscpp
	sensor_msgs
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
	include/${PROJECT_NAME}
	SYSTEM
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_camera                         src/camera.cpp)
add_executable(${PROJECT_NAME}_vision                         src/vision.cpp)
add_executable(${PROJECT_NAME}_vision_standalone              src/vision_standalone.cpp)
add_executable(${PROJECT_NAME}_gripper                        src/gripper.cpp)
add_executable(${PROJECT_NAME}_robot_io                       src/robot_io.cpp)
add_executable(${PROJECT_NAME}_grasp_synthesizer              src/grasp_synthesizer.cpp)
add_executable(${PROJECT_NAME}_grasp_synthesizer_standalone   src/grasp_synthesizer_standalone.cpp)
add_executable(${PROJECT_NAME}_place_synthesizer              src/place_synthesizer.cpp)
add_executable(${PROJECT_NAME}_pose_estimation                src/pose_estimation.cpp)
add_executable(${PROJECT_NAME}_pose_estimation_standalone     src/pose_estimation_standalone.cpp)
add_executable(${PROJECT_NAME}_bin_pose_estimation            src/bin_pose_estimation.cpp)
add_executable(${PROJECT_NAME}_bin_pose_estimation_standalone src/bin_pose_estimation_standalone.cpp)
add_executable(${PROJECT_NAME}_full_system                    src/full_system.cpp)

target_link_libraries(${PROJECT_NAME}_camera                         ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_vision                         ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_vision_standalone              ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_gripper                        ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_robot_io                       ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_synthesizer              ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_synthesizer_standalone   ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_place_synthesizer              ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_pose_estimation                ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_pose_estimation_standalone     ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_bin_pose_estimation            ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_bin_pose_estimation_standalone ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_full_system                    ${catkin_LIBRARIES})
