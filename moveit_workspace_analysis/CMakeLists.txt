cmake_minimum_required(VERSION 2.8.3)
project(moveit_workspace_analysis)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    moveit_workspace_analysis
  CATKIN_DEPENDS 
    moveit_msgs
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    Boost
    Eigen
)

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS}
                           ${Boost_INCLUDE_DIR}
                           )

include_directories(include ${catkin_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(moveit_workspace_analysis
   src/workspace_analysis.cpp
)

add_dependencies(moveit_workspace_analysis ${catkin_EXPORTED_TARGETS})

add_executable(workspace_analysis_node src/main.cpp)
add_executable(workspace_reader_node src/reader.cpp)

target_link_libraries(moveit_workspace_analysis
  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(workspace_analysis_node
  moveit_workspace_analysis ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(workspace_reader_node
  moveit_workspace_analysis ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(
  TARGETS
    moveit_workspace_analysis
    workspace_analysis_node
    workspace_reader_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


