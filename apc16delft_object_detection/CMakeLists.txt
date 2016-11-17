cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_object_detection)

find_package(catkin REQUIRED COMPONENTS
	rospy
	tf
	visualization_msgs
	apc16delft_msgs
)

catkin_package()

catkin_python_setup()
