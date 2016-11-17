cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_description)

find_package(catkin REQUIRED)

catkin_package()

install(
	DIRECTORY urdf
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(
	DIRECTORY srdf
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
)

install(
	DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
