cmake_minimum_required(VERSION 2.8.3)
project(apc16delft_msgs)

set(required_message_packages
	trajectory_msgs
	geometry_msgs
	octomap_msgs
	sensor_msgs
	shape_msgs
	std_msgs
)

find_package(catkin REQUIRED COMPONENTS
	dr_base
	message_generation
	${required_message_packages}
)

add_message_files(DIRECTORY msg FILES
	BoundingBox2D.msg
	CollisionObject.msg
	CollisionScene.msg
	CropBox2D.msg
	Error.msg
	GraspCandidate.msg
	GripperState.msg
	GripperStateStamped.msg
	LifecycleState.msg
	MasterPoseDescriptor.msg
	Milestone.msg
	Object.msg
	ObjectDetection.msg
	Occlusion.msg
	PlacementCandidate.msg
	RotatedBox.msg
)

add_service_files(DIRECTORY srv FILES
	AddCollisionObject.srv
	CropPointCloud.srv
	DetectObjects.srv
	DetectOcclusion.srv
	PlanManipulation.srv
	EstimateObjectPose.srv
	EstimateBinDensity.srv
	EstimateBinPose.srv
	ExecuteCalibrationMotion.srv
	ExecuteCoarseMotion.srv
	ExecuteFineMotion.srv
	ExecuteStitchedMotion.srv
	FilterObject.srv
	FinalizeCalibration.srv
	FindFreeSpace.srv
	GetCameraData.srv
	GetCoarseMotion.srv
	GetPlaceInTote.srv
	GetPose.srv
	InitializeCalibration.srv
	MoveToHome.srv
	MoveToCalibrateShelf.srv
	MoveToCalibrateTote.srv
	PlanCameraMotion.srv
	PlanStowMotion.srv
	PublishStaticPointCloud.srv
	PruneGraspCandidates.srv
	RecordCalibration.srv
	RequestTransition.srv
	SetGripperState.srv
	SynthesizeDeformable.srv
	SynthesizeGrasp.srv
	SynthesizePlacement.srv
	SynthesizeStow.srv
)

include(CMakeParseArguments)
function(generate_enum_string_conversions)
	cmake_parse_arguments("ARG" "" "PREFIX;BASENAME" "FILES" ${ARGN})

	set(files           "${ARG_FILES}")
	set(cxx_include_dir "${CATKIN_DEVEL_PREFIX}/include")
	set(cxx_header_dir  "${cxx_include_dir}/${PROJECT_NAME}")
	set(cxx_source_dir  "${CATKIN_DEVEL_PREFIX}/src")
	set(cxx_header      "${cxx_header_dir}/${ARG_BASENAME}.hpp")
	set(cxx_source      "${cxx_source_dir}/${ARG_BASENAME}.cpp")
	set(cxx_library     "${PROJECT_NAME}_${ARG_BASENAME}")
	set(python_file     "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/${ARG_BASENAME}.py")
	set(script          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen-enums.py")

	# Set variables for parent scope.
	set("${ARG_PREFIX}_INCLUDE_DIRS" "${cxx_include_dir}" PARENT_SCOPE)
	set("${ARG_PREFIX}_LIBRARIES"    "${cxx_library}"     PARENT_SCOPE)

	# Add rule to generate header and source file.
	add_custom_command(
		OUTPUT ${cxx_header} ${cxx_source}
		DEPENDS ${files} ${script}
		COMMAND ${CMAKE_COMMAND} -E make_directory "${cxx_header_dir}"
		COMMAND ${CMAKE_COMMAND} -E make_directory "${cxx_source_dir}"
		COMMAND ${script}
			"--cxx-namespace" ${PROJECT_NAME}
			"--cxx-header"    ${cxx_header}
			"--cxx-source"    ${cxx_source}
			"--python"        ${python_file}
			${files}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
	)

	# Build and install C++ library.
	add_library(${cxx_library} ${cxx_source})
	install(FILES ${cxx_header} DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
	install(TARGETS ${cxx_library}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

	# Install python library.
	install(FILES ${python_file} DESTINATION "${CATKIN_PACKAGE_PYTHON_DESTINATION}")
endfunction()

generate_messages(DEPENDENCIES ${required_message_packages})

generate_enum_string_conversions(
	PREFIX "objects_enum"
	BASENAME "objects"
	FILES "msg/Object.msg"
)

catkin_package(
	CATKIN_DEPENDS message_runtime
	LIBRARIES ${objects_enum_LIBRARIES}
	INCLUDE_DIRS "${objects_enum_INCLUDE_DIRS}"
)

add_dependencies(${objects_enum_LIBRARIES} ${PROJECT_NAME}_gencpp)
