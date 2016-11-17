cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_simulation)

find_package(catkin REQUIRED COMPONENTS
	apc16delft_moveit_config
	industrial_robot_simulator
	robot_state_publisher
)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)
