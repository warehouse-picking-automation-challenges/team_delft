#include "manipulation_planner.hpp"
#include "master_pose.hpp"

#include <apc16delft_util/util.hpp>
#include <apc16delft_msgs/objects.hpp>
#include <apc16delft_msgs/Object.h>
#include <apc16delft_msgs/Milestone.h>

#include <moveit_msgs/MoveItErrorCodes.h>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <dr_eigen/yaml.hpp>

namespace apc16delft {

bool ManipulationPlanner::load3DModels(const std::map<std::string, GraspItem> & item_map) {
		model_list_.resize(number_of_items_);
		for (uint8_t i = 1; i <= number_of_items_; i++) {
			if (isDeformable(i))
				continue;
			try {
				GraspItem item         = item_map.at(apc16delft_msgs::objectTypeToString(i));
				std::string model_path = resolvePackagePath(item.model_path);
				// read model
				pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);
				if (pcl::io::loadPCDFile(model_path, *model) == -1) {
					ROS_ERROR_STREAM("No model found. Model path: " << model_path);
					return false;
				}
				// check for empty model
				if (model->empty()) {
					ROS_ERROR_STREAM("Model point cloud is empty.");
					return false;
				}
				model_list_.at(i-1) = model;
			} catch (const std::out_of_range & e) {
				ROS_ERROR_STREAM(e.what());
				return false;
			}
		}
		return true;
}

int ManipulationPlanner::loadParams() {
	try {
		visualise_                                = dr::getParam<bool>(node_handle, "visualise", visualise_                                                            );
		stow_y_offset_                            = dr::getParam<double>(node_handle, "stow_y_offset", stow_y_offset_                                                  );
		dumbbell_inwards_pre_                     = dr::getParam<double>(node_handle, "dumbbell_inwards_pre", dumbbell_inwards_pre_                                    );
		gripper_top_bin_edge_clearing_            = dr::getParam<double>(node_handle, "/grasping/params/gripper_top_bin_edge_clearing", gripper_top_bin_edge_clearing_ );
		stow_gripper_top_bin_edge_clearing_       = dr::getParam<double>(node_handle, "/grasping/params/stow_gripper_top_bin_edge_clearing", stow_gripper_top_bin_edge_clearing_ );
		top_bin_edge_clearing_                    = dr::getParam<double>(node_handle, "/grasping/params/top_bin_edge_clearing", top_bin_edge_clearing_                 );
		bottom_bin_edge_clearing_                 = dr::getParam<double>(node_handle, "/grasping/params/bottom_bin_edge_clearing", bottom_bin_edge_clearing_           );
		side_bin_edge_clearing_                   = dr::getParam<double>(node_handle, "/grasping/params/side_bin_edge_clearing", side_bin_edge_clearing_               );
		back_bin_edge_clearing_                   = dr::getParam<double>(node_handle, "/grasping/params/back_bin_edge_clearing", back_bin_edge_clearing_               );
		trajectory_velocity_scaling_              = dr::getParam<double>(node_handle, "trajectory_velocity_scaling", trajectory_velocity_scaling_                      );
		std::map<std::string, GraspItem> item_map = dr::getParamMap<std::string, GraspItem>(node_handle, "/item" , std::map<std::string, GraspItem>{}                  );
		number_of_items_                          = dr::getParam<int>(node_handle, "/grasping/number_of_items"  , number_of_items_                                     );
		approach_angle_                           = deg2rad(dr::getParam<double>(node_handle, "/grasping/params/approach_angle", approach_angle_                      ));
		side_approach_angle_                      = deg2rad(dr::getParam<double>(node_handle, "/grasping/params/side_approach_angle", side_approach_angle_            ));

		// Load in all available 3D models.
		if(!load3DModels(item_map))
			return 1;

		// Load in all bin dimensions.
		if (!loadBinDimensions(bin_dimensions_, node_handle))
			return 1;

		tote_dimensions_ = dr::getParam<Eigen::Vector3d>(node_handle, "/tote/dimensions");

		return 0;
	} catch (const std::exception & e) {
		ROS_ERROR_STREAM(e.what());
		return 1;
	}

}

int ManipulationPlanner::onConfigure() {

	tool0_group_.setGoalTolerance(0.001);
	tool0_group_.setPlannerId("RRTConnectkConfigDefault");
	tool0_group_.allowReplanning(true);
	tool0_group_.setNumPlanningAttempts(5);

	tool1_group_.setGoalTolerance(0.001);
	tool1_group_.setPlannerId("RRTConnectkConfigDefault");
	tool1_group_.allowReplanning(true);
	tool1_group_.setNumPlanningAttempts(5);

	robot_state0_ = tool0_group_.getCurrentState();
	robot_state1_ = tool1_group_.getCurrentState();
	robot_state2_ = tool2_group_.getCurrentState();
	robot_state_  = robot_state0_;

	jmg0_ = robot_state0_->getJointModelGroup(tool0_group_.getName());
	jmg1_ = robot_state1_->getJointModelGroup(tool1_group_.getName());
	jmg2_ = robot_state2_->getJointModelGroup(tool2_group_.getName());
	jmg_  = jmg0_;

	return loadParams();
}

int ManipulationPlanner::onActivate() {
	plan_manipulation_      = node_handle.advertiseService("plan_manipulation", &ManipulationPlanner::planManipulation, this);
	plan_tote_manipulation_ = node_handle.advertiseService("plan_tote_manipulation", &ManipulationPlanner::planToteManipulation, this);
	update_reachability_    = node_handle.advertiseService("update_reachability", &ManipulationPlanner::updateReachability, this);
	plan_camera_motion_     = node_handle.advertiseService("plan_camera_motion", &ManipulationPlanner::planCameraMotion, this);
	plan_stow_motion_       = node_handle.advertiseService("plan_stow_motion", &ManipulationPlanner::planStowMotion, this);
	vacuum_visualiser_      = node_handle.advertise<geometry_msgs::PoseArray>("visualise_vacuum", 1, true);
	waypoint_visualiser_    = node_handle.advertise<geometry_msgs::PoseArray>("visualise_waypoints", 1, true);

	add_collision_object_= node_handle.serviceClient<apc16delft_msgs::AddCollisionObject>("/scene_updater/add_collision_object");
	ik_service_          = node_handle.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");

	return 0;
}

std::array<double, 5> ManipulationPlanner::calculateClearances(apc16delft_msgs::PlanManipulation::Request & req) {
	Eigen::Isometry3d bin_pose                           = dr::toEigen(req.bin_pose.pose);
	Eigen::Isometry3d object_pose                        = dr::toEigen(req.object_pose.pose);
	Eigen::Vector3d bin_sizes;
	if(req.bin_index == -1) { // Tote cheating.
		bin_sizes = tote_dimensions_;
	}
	else {
		bin_sizes = bin_dimensions_.at(req.bin_index);
	}

	std::array<Eigen::Hyperplane<double, 3>, 5> bin_lips = getBinBoundaries(bin_pose, bin_sizes);

	pcl::PointCloud<pcl::PointXYZ> point_cloud           = *model_list_.at(req.object_type - 1);
	pcl::PointCloud<pcl::PointXYZ> transformed_point_cloud;

	Eigen::Affine3d affine_object_pose{object_pose.matrix()};
	pcl::transformPointCloud(point_cloud, transformed_point_cloud, affine_object_pose);

	Eigen::Vector4f homogenous_object_min_corner, homogenous_object_max_corner;
	pcl::getMinMax3D(transformed_point_cloud, homogenous_object_min_corner, homogenous_object_max_corner);
	Eigen::Vector3d object_min_corner = homogenous_object_min_corner.head<3>().cast<double>();
	Eigen::Vector3d object_max_corner = homogenous_object_max_corner.head<3>().cast<double>();

	double top_clearance   = std::min(bin_lips.at(top_lip).signedDistance(object_min_corner)  , bin_lips.at(top_lip).signedDistance(object_max_corner));
	double left_clearance  = std::min(bin_lips.at(left_lip).signedDistance(object_min_corner) , bin_lips.at(left_lip).signedDistance(object_max_corner));
	double right_clearance = std::min(bin_lips.at(right_lip).signedDistance(object_min_corner), bin_lips.at(right_lip).signedDistance(object_max_corner));
	double back_clearance  = std::min(bin_lips.at(back_lip).signedDistance(object_min_corner) , bin_lips.at(back_lip).signedDistance(object_max_corner));

	// Lip order is clockwise starting from top, with back wall as last element.
	// Bottom lip is ommited as the object is always on the bottom.
	return std::array<double, 5>{{top_clearance, right_clearance, 0.0, left_clearance, back_clearance}};
}

void appendLinearWaypoints(std::vector<Eigen::Isometry3d> & output, Eigen::Isometry3d const & start, Eigen::Isometry3d const & target, int samples) {
	Eigen::Isometry3d difference = start.inverse() * target;
	Eigen::Vector3d translation = difference.translation();
	Eigen::AngleAxisd rotation{difference.rotation()};


	for (int i = 0; i < samples; ++i) {
		double factor = double(i + 1) / samples;
		Eigen::Isometry3d waypoint = start * Eigen::Translation3d{factor * translation} * Eigen::AngleAxisd{factor * rotation.angle(), rotation.axis()};
		output.push_back(waypoint);
	}
}

Eigen::Quaterniond aimingFrame(int strategy, const Eigen::Isometry3d & waypoint, const Eigen::Isometry3d & bin_pose) {
	Eigen::Vector3d candidate_z = waypoint.rotation() * dr::axes::z();
	switch (strategy) {
		case apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_H:
		case apc16delft_msgs::GraspCandidate::VACUUM_H_BIG:
			if (facingDown(candidate_z, bin_pose)) {
				return Eigen::Quaterniond::Identity() * dr::rotateX(M_PI);
			} else if (facingLeft(candidate_z, bin_pose)) {
				return Eigen::Quaterniond::Identity() * dr::rotateX(M_PI) * dr::rotateY(-M_PI_2);
			} else if (facingRight(candidate_z, bin_pose)) {
				return Eigen::Quaterniond::Identity() * dr::rotateX(M_PI) * dr::rotateY(M_PI_2);
			}
		case apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_V:
		case apc16delft_msgs::GraspCandidate::VACUUM_V_BIG:
			return Eigen::Quaterniond::Identity() * dr::rotateX(-M_PI_2) * dr::rotateZ(-M_PI_2);
		default:
			return Eigen::Quaterniond::Identity() * dr::rotateX(M_PI);
	}
}

std::array<std::vector<Eigen::Isometry3d>, 4> ManipulationPlanner::planCartesianPath(uint8_t object_type, int bin_index, const Eigen::Isometry3d & bin_pose, const Eigen::Isometry3d & current_pose, apc16delft_msgs::GraspCandidate & grasp, const std::array<double, 5> & clearances, bool angled_approach, bool side_angled_approach) {
	Eigen::Isometry3d grasp_pose = dr::toEigen(grasp.stamped_pose.pose);
	Eigen::Vector3d grasp_pose_z = grasp_pose.rotation() * dr::axes::z();

	Eigen::Vector3d bin_center   = getBinCenter(bin_pose, bin_dimensions_.at(bin_index));

	double offset = grasp.pre_grasp_offset;
	if (grasp.strategy != apc16delft_msgs::GraspCandidate::VACUUM_H_BIG && grasp.strategy != apc16delft_msgs::GraspCandidate::VACUUM_V_BIG) {
		if (facingDown(grasp_pose_z, bin_pose)) {
			offset = clearances.at(top_lip) - gripper_top_bin_edge_clearing_;
		} else if (facingLeft(grasp_pose_z, bin_pose)) {
			offset = clearances.at(right_lip) - side_bin_edge_clearing_;
		} else if (facingRight(grasp_pose_z, bin_pose)) {
			offset = clearances.at(left_lip) - side_bin_edge_clearing_;
		}
		if (object_type == apc16delft_msgs::Object::KLEENEX_PAPER_TOWELS) {
			offset = std::max(0.035, offset);
		} else {
			offset = std::max(0.03, offset);
		}
	}

	// Pre-grasp with certain offset from surface.
	// Max offset is the available clearance.
	offset = std::min(offset, grasp.pre_grasp_offset);

	Eigen::Isometry3d pre_grasp_pose = grasp_pose * dr::translate(0.0, 0.0, -offset);
	std::vector<Eigen::Isometry3d> approach_waypoints, contact_waypoints, lift_waypoints, retreat_waypoints;
	//approach_waypoints.push_back(current_pose);

	// Move into the bin.
	Eigen::Isometry3d waypoint_2     = pre_grasp_pose;
	approach_waypoints.push_back(waypoint_2);

	// Move towards the grasp pose.
	Eigen::Isometry3d waypoint_3     = grasp_pose;
	contact_waypoints.push_back(waypoint_3);

	// Compensate for all angles.
	Eigen::Isometry3d waypoint_4 = dr::translate(waypoint_3.translation()) * aimingFrame(grasp.strategy, waypoint_3, bin_pose);
	Eigen::Vector3d waypoint_4_z = waypoint_4.rotation() * dr::axes::z();
	Eigen::Vector3d waypoint_4_x = waypoint_4.rotation() * dr::axes::x();

	double up_offset = (facingDown(waypoint_4_z, bin_pose) && facingRight(waypoint_4_x, bin_pose)) ?
		clearances.at(top_lip) - gripper_top_bin_edge_clearing_ : clearances.at(top_lip) - top_bin_edge_clearing_;

	if (up_offset < 0) {
		switch (grasp.strategy) {
			case apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_H:
				grasp.strategy = apc16delft_msgs::GraspCandidate::VACUUM_H_BIG;
			case apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_V:
				grasp.strategy = apc16delft_msgs::GraspCandidate::VACUUM_V_BIG;
		}
	}

	if (grasp.strategy == apc16delft_msgs::GraspCandidate::VACUUM_H_BIG || grasp.strategy == apc16delft_msgs::GraspCandidate::VACUUM_V_BIG) {
		Eigen::Vector3d candidate_3_z = waypoint_3.rotation() * dr::axes::z();
		Eigen::Vector3d candidate_3_x = waypoint_3.rotation() * dr::axes::x();

		// If big item, retreat diagonally.
		if (grasp.strategy == apc16delft_msgs::GraspCandidate::VACUUM_H_BIG) {
			// Move to center of bin and lift.
			waypoint_4.translation().x() = bin_center.x();
			waypoint_4 = dr::translate(0, 0, grasp.pre_grasp_offset) * waypoint_4;

			if (facingDown(candidate_3_z, bin_pose) && facingRight(candidate_3_x, bin_pose)) {
				waypoint_4.translation().z() = bin_center.z();
				waypoint_4 = waypoint_4 * dr::rotateY(M_PI_4);
			} else if (facingLeft(candidate_3_z, bin_pose)) {
				if (isCornerBin(bin_index)) {
					waypoint_4 = waypoint_4 * dr::rotateY(M_PI_4);
				} else {
					waypoint_4 = waypoint_4 * dr::rotateY(-M_PI_4);
				}
			} else if (facingRight(candidate_3_z, bin_pose)) {
				if (isCornerBin(bin_index)) {
					waypoint_4 = waypoint_4 * dr::rotateY(-M_PI_4);
				} else {
					waypoint_4 = waypoint_4 * dr::rotateY(M_PI_4);
				}
			}
		} else if (grasp.strategy == apc16delft_msgs::GraspCandidate::VACUUM_V_BIG) {
			waypoint_4 = waypoint_3;
			waypoint_4.translation().x() = bin_center.x();
			waypoint_4 = dr::translate(0, 0, grasp.pre_grasp_offset) * waypoint_4;
		}
	} else {
		if (up_offset < 0)
			throw std::runtime_error("Negative offset");

		// Move up as much as possible.
		waypoint_4 = dr::translate(0, 0, up_offset) * waypoint_4;
		// Move to center of bin.
		waypoint_4.translation().x() = bin_center.x();
	}

	if (object_type == apc16delft_msgs::Object::HANES_TUBE_SOCKS) {
		waypoint_4 = dr::translate(0.0, 0.0, 0.02) * waypoint_4;
	}

	if (object_type == apc16delft_msgs::Object::KLEENEX_PAPER_TOWELS) {
		waypoint_4 = dr::translate(0.0, 0.0, -0.005) * waypoint_4;
	}

	appendLinearWaypoints(lift_waypoints, waypoint_3, waypoint_4, 20);

	/* ------------IMPORTANT!!!--------------*/
	/*           DON'T TOUCH THIS!           */
	// Move out of the bin.
	Eigen::Isometry3d waypoint_5     = waypoint_4;
	waypoint_5.translation().y()     = current_pose.translation().y();
	retreat_waypoints.push_back(waypoint_5);

	geometry_msgs::PoseArray visualization_poses;
	visualization_poses.header.stamp = ros::Time::now();
	visualization_poses.header.frame_id = "world";
//	visualization_poses.poses.push_back(dr::toRosPose(current_pose));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_2));
//	visualization_poses.poses.push_back(dr::toRosPose(waypoint_3));
//	for (Eigen::Isometry3d const & pose : lift_waypoints) {
//		visualization_poses.poses.push_back(dr::toRosPose(pose));
//	}
	//visualization_poses.poses.push_back(dr::toRosPose(waypoint_4));
//	visualization_poses.poses.push_back(dr::toRosPose(waypoint_5));
	waypoint_visualiser_.publish(visualization_poses);

	return std::array<std::vector<Eigen::Isometry3d>, 4>{{approach_waypoints, contact_waypoints, lift_waypoints, retreat_waypoints}};
}

std::array<std::vector<Eigen::Isometry3d>, 4> ManipulationPlanner::planDeformablePath(int bin_index, const Eigen::Isometry3d & bin_pose, const Eigen::Isometry3d & current_pose, const apc16delft_msgs::GraspCandidate & grasp, bool angled_approach, bool side_angled_approach) {
	Eigen::Vector3d bin_sizes                            = bin_dimensions_.at(bin_index);
	std::array<Eigen::Hyperplane<double, 3>, 5> bin_lips = getBinBoundaries(bin_pose, bin_sizes);
	Eigen::Vector3d bin_center = getBinCenter(bin_pose, bin_sizes);
	Eigen::Isometry3d grasp_pose = dr::toEigen(grasp.stamped_pose.pose);
	Eigen::Vector3d grasp_pose_z = grasp_pose.rotation() * dr::axes::z();

	double offset = grasp.pre_grasp_offset;
	Eigen::Isometry3d pre_grasp_pose = grasp_pose * dr::translate(0.0, 0.0, -offset);
	std::vector<Eigen::Isometry3d> approach_waypoints, contact_waypoints, lift_waypoints, retreat_waypoints;
	approach_waypoints.push_back(current_pose);

	// Move into the bin.
	Eigen::Isometry3d waypoint_2     = pre_grasp_pose;
	approach_waypoints.push_back(waypoint_2);

	// Move towards the grasp pose.
	Eigen::Isometry3d waypoint_3     = grasp_pose;
	contact_waypoints.push_back(waypoint_3);

	// Compensate for all angles.
	Eigen::Isometry3d waypoint_4 = dr::translate(waypoint_3.translation()) * aimingFrame(grasp.strategy, waypoint_3, bin_pose);

	// Move to center of bin.
	waypoint_4.translation().x() = bin_center.x();
	waypoint_4.translation().z() = bin_center.z();
	appendLinearWaypoints(lift_waypoints, waypoint_3, waypoint_4, 20);

	/* ------------IMPORTANT!!!--------------*/
	/*           DON'T TOUCH THIS!           */
	// Move out of the bin.
	Eigen::Isometry3d waypoint_5     = waypoint_4;
	waypoint_5.translation().y()     = current_pose.translation().y();
	retreat_waypoints.push_back(waypoint_5);

	geometry_msgs::PoseArray visualization_poses;
	visualization_poses.header.stamp = ros::Time::now();
	visualization_poses.header.frame_id = "world";
	visualization_poses.poses.push_back(dr::toRosPose(current_pose));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_2));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_3));
	for (Eigen::Isometry3d const & pose : lift_waypoints) {
		visualization_poses.poses.push_back(dr::toRosPose(pose));
	}

	//visualization_poses.poses.push_back(dr::toRosPose(waypoint_4));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_5));
	waypoint_visualiser_.publish(visualization_poses);

	return std::array<std::vector<Eigen::Isometry3d>, 4>{{approach_waypoints, contact_waypoints, lift_waypoints, retreat_waypoints}};
}

bool ManipulationPlanner::stitchTrajectories(moveit_msgs::RobotTrajectory & motion_trajectory, std::vector<double> master_joints) {
	move_group_interface::MoveGroup::Plan append_plan;
	moveit_msgs::RobotTrajectory append_trajectory;
	append_trajectory.joint_trajectory.points.clear();

	// Plan from current position to actual master pose with RRT planning.
	robot_state_->setVariablePositions(motion_trajectory.joint_trajectory.joint_names, motion_trajectory.joint_trajectory.points.back().positions);
	current_group_->setStartState(*robot_state_);
	current_group_->setJointValueTarget(master_joints);
	if (current_group_->plan(append_plan) != 1) {
		ROS_INFO_STREAM("Plan to retreat master pose failed.");
		return false;
	}

	// Clear all velocities and accelerations in the planned trajectory. They will be reparameterized.
	for(size_t idx = 0; idx < append_plan.trajectory_.joint_trajectory.points.size(); idx++) {
		append_plan.trajectory_.joint_trajectory.points[idx].velocities.clear();
		append_plan.trajectory_.joint_trajectory.points[idx].accelerations.clear();
	}
	append_trajectory.joint_trajectory.points = append_plan.trajectory_.joint_trajectory.points;
	
	motion_trajectory.joint_trajectory.points.insert(motion_trajectory.joint_trajectory.points.end(),
		append_trajectory.joint_trajectory.points.begin(), append_trajectory.joint_trajectory.points.end());

	return true;
}

bool ManipulationPlanner::getExecutableTrajectory (moveit_msgs::RobotTrajectory & motion_trajectory, const robot_state::RobotState &robot_state, boost::optional<MasterPoseDescriptor> master_pose) {
	robot_trajectory::RobotTrajectory rt(robot_state_->getRobotModel(), current_group_->getName());

	if (master_pose) {
		std::vector<double> master_joint_values;
		getMasterJoints(master_joint_values, *master_pose);
		//If a valid bin index is passed, include that in the trajectory stitching.
		if(!stitchTrajectories(motion_trajectory, master_joint_values)) {
			ROS_ERROR_STREAM("Collision detected during planning to master pose.");
			return false;
		}
	}

	// Fill robot trajectory with computed cartesian path
	rt.setRobotTrajectoryMsg(robot_state, motion_trajectory);

	// Add velocities to the trajectory using iterative parabolic time parameterization
	trajectory_processing::IterativeParabolicTimeParameterization iptp;
	iptp.computeTimeStamps(rt, trajectory_velocity_scaling_);

	rt.getRobotTrajectoryMsg(motion_trajectory);
	// ROS_INFO_STREAM(motion_trajectory.joint_trajectory);
	// //check for too large jumps
	// int inserted_waypoints = 0;
	// for (size_t i = 0; i < rt.getWayPointCount(); i++){
	// 	double duration = rt.getWayPointDurationFromPrevious(i);
	// 	if (duration > 5.0){
	// 		int num_inserts = duration/5;
	// 		for (int ii = 0; ii < num_inserts; ii++){

	// 			robot_state::RobotState rs1 = rt.getWayPoint(i+inserted_waypoints);
	// 			robot_state::RobotState rs2 = rt.getWayPoint(i+inserted_waypoints+1);

	// 			std::vector<double> jv1 = *rs1.getVariablePositions();
	// 			std::vector<double> jv2 = *rs2.getVariablePositions();

	// 			for (int joint = 0; joint < jv1.size(); joint++){
	// 				jv1[joint] += jv2[joint]*((float)ii/num_inserts);
	// 			}
	// 			// Inserting extra waypoints in the trajectory
	// 			rt.insertWayPoint(i,jv1,1.0);
	// 		}
	// 		inserted_waypoints += num_inserts;
	// 	}
	// }
	// if (inserted_waypoints > 0){
	// 	iptp.computeTimeStamps(rt, trajectory_velocity_scaling_);
	// }

	// rt.getRobotTrajectoryMsg(motion_trajectory);
	// ROS_INFO_STREAM(motion_trajectory.joint_trajectory);

	rt.getRobotTrajectoryMsg(motion_trajectory);
	return true;
}

bool ManipulationPlanner::getTrajectoryFromWaypoints(
	const robot_state::RobotState &robot_state,
	std::vector<geometry_msgs::Pose> const & waypoints,
	move_group_interface::MoveGroup::Plan & executable_plan,
	bool enable_collision_check,
	boost::optional<MasterPoseDescriptor> master_pose
) {
	moveit_msgs::RobotTrajectory motion_trajectory;
	if(!enable_collision_check) {
		ROS_WARN_STREAM ("ATTENTION: Received fine motion request without collision check. Watch out!!!");
	}

	// Setting start state for computing Cartesian Path
	current_group_->setStartState(robot_state);
	double frac = current_group_->computeCartesianPath (waypoints, 0.01, 4.0, motion_trajectory, enable_collision_check);
	if (frac < 1.0) {
		//Collisions. Nothing further to do.
		return false;
	} else {
		//Modify the trajectory to include velocities.
		getExecutableTrajectory(motion_trajectory, robot_state, master_pose);
		executable_plan.trajectory_ = motion_trajectory;
		motion_visualizer_.displayTrajectory(executable_plan);
		return true;
	}
}

bool ManipulationPlanner::getMasterJoints(std::vector<double> & joint_values, apc16delft_msgs::MasterPoseDescriptor master_pose_descriptor) {
	std::string param = "/apc16delft/master_poses/" + masterPoseToString(master_pose_descriptor);
	if (!node_handle.getParam(param, joint_values)) {
		return false;
	}
	return true;
}

bool ManipulationPlanner::getMasterPose(geometry_msgs::Pose & pose, apc16delft_msgs::MasterPoseDescriptor master_pose_descriptor) {
	std::vector<double> joint_states;

	if (!getMasterJoints(joint_states, master_pose_descriptor)) return false;

	//Dummy gripper states to update the full kinematic state.
	joint_states.push_back(0.00);
	joint_states.push_back(0.00);
	joint_states.push_back(0.00);

	robot_state_->setVariablePositions(joint_states);

	const Eigen::Affine3d &master_pose = robot_state_->getGlobalLinkTransform(current_group_->getEndEffectorLink());
	pose.position.x = master_pose.translation().x();
	pose.position.y = master_pose.translation().y() ;
	pose.position.z = master_pose.translation().z();

	Eigen::Quaterniond quat(master_pose.rotation());

	pose.orientation.x = quat.x();
	pose.orientation.y = quat.y();
	pose.orientation.z = quat.z();
	pose.orientation.w = quat.w();
	ROS_INFO_STREAM(pose);

	return true;
}

robot_state::RobotState ManipulationPlanner::robotStateFromJoints(std::vector<double> const & joint_values) {
	robot_state::RobotState result = *robot_state_;
	result.setJointGroupPositions(jmg_, joint_values);
	return result;
}

bool ManipulationPlanner::checkStateCollision (geometry_msgs::Pose & pose) {
	moveit_msgs::GetPositionIK::Request ik_req;
	ik_req.ik_request.group_name = current_group_->getName();

	moveit_msgs::RobotState moveit_rs;
	ik_req.ik_request.robot_state = moveit_rs;

	ik_req.ik_request.avoid_collisions = true;

	geometry_msgs::PoseStamped pose_s;
	pose_s.header.stamp = ros::Time::now();
	pose_s.header.frame_id = "world";
	pose_s.pose = pose;

	ik_req.ik_request.pose_stamped = pose_s;
	ik_req.ik_request.timeout = ros::Duration(0.01);
	ik_req.ik_request.attempts = 3;

	moveit_msgs::GetPositionIK::Response ik_res;
	if(ik_service_.call(ik_req, ik_res)){
		if (ik_res.error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION){
			return false;
		} else {
			return true;
		}
	} else {
		return false;
	}
}

std::string ManipulationPlanner::getWorkingGroupFromStrategy(int strategy) {
	std::string tool;
	if(strategy == apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_H || strategy == apc16delft_msgs::GraspCandidate::VACUUM_H_BIG) {
		current_group_ = &tool0_group_;
		current_group_->setEndEffectorLink("gripper_tool0");
		tool = "gripper_tool0";
		jmg_ = jmg0_;
		robot_state_ = robot_state0_;
	} else if (strategy == apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_V || strategy == apc16delft_msgs::GraspCandidate::VACUUM_V_BIG) {
		current_group_ = &tool1_group_;
		current_group_->setEndEffectorLink("gripper_tool1");
		tool = "gripper_tool1";
		jmg_ = jmg1_;
		robot_state_ = robot_state1_;
	} else {
		switch (strategy) {
			case apc16delft_msgs::GraspCandidate::DUMBBELL_INWARDS:
			case apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS:
			case apc16delft_msgs::GraspCandidate::PENCIL_CUP_STANDING:
			case apc16delft_msgs::GraspCandidate::PENCIL_CUP_SIDEWAYS:
			case apc16delft_msgs::GraspCandidate::PENCIL_CUP_INWARDS:
				current_group_ = &tool2_group_;
				current_group_->setEndEffectorLink("gripper_tool2");
				tool = "gripper_tool2";
				jmg_ = jmg2_;
				robot_state_ = robot_state2_;
				break;
			default:
				// All those ye who enter here will be damned to hell!
				current_group_ = &tool0_group_;
				current_group_->setEndEffectorLink("gripper_tool0");
				tool = "gripper_tool0";
				jmg_ = jmg0_;
				robot_state_ = robot_state0_;
				unknown_grasp_strategy_ = true;
				ROS_ERROR_STREAM("Unknown grasp strategy! All further requests down the line will be declined!");
				break;
			}
	}
	return tool;
}

bool ManipulationPlanner::planStowMotion(apc16delft_msgs::PlanStowMotion::Request & req, apc16delft_msgs::PlanStowMotion::Response & res) {
	std::string tool = getWorkingGroupFromStrategy(req.grasp_candidate.strategy);

	if(unknown_grasp_strategy_) {
		res.error.code = 1;
		res.error.message = "Unknown grasp strategy. Plan request rejected!";
		unknown_grasp_strategy_ = false;
		return true;
	}

	geometry_msgs::Pose master_pose;
	if (!getMasterPose(master_pose, binMasterPose(req.bin_index, current_group_->getName()))) {
		res.error.code = apc16delft_msgs::PlanManipulationRequest::E_INVALID_BIN;
		res.error.message = "Could not find master pose from index.";
		return true;
	}

	Eigen::Isometry3d bin_pose                           = dr::toEigen(req.bin_pose.pose);
	Eigen::Vector3d bin_sizes                            = bin_dimensions_.at(req.bin_index);
	std::array<Eigen::Hyperplane<double, 3>, 5> bin_lips = getBinBoundaries(bin_pose, bin_sizes);
	Eigen::Vector3d bin_center                           = getBinCenter(dr::toEigen(req.bin_pose.pose), bin_dimensions_.at(req.bin_index));

	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(master_pose);

	std::vector<double> joint_values;
	getMasterJoints(joint_values, containerMasterPose(req.bin_index,current_group_->getName()));

	bool sol_found = false;
	move_group_interface::MoveGroup::Plan plan;
	ROS_INFO_STREAM(req.grasp_candidate.strategy);
	// Vacuum Vertical
	if (req.grasp_candidate.strategy == apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_V ||
		req.grasp_candidate.strategy == apc16delft_msgs::GraspCandidate::VACUUM_V_BIG){

		geometry_msgs::Pose target_pose, align_pose;
		
		target_pose.orientation.x = -0.707;
		target_pose.orientation.y = 0;
		target_pose.orientation.z = 0;
		target_pose.orientation.w = 0.707;
		target_pose.position.x = bin_center.x();
		target_pose.position.y = bin_center.y() + stow_y_offset_;
		target_pose.position.z = bin_center.z();

		align_pose = target_pose;
		align_pose.position.y = master_pose.position.y;
		waypoints.push_back(align_pose);
		waypoints.push_back(target_pose);

	// Vacumm Horizontal
	} else if (req.grasp_candidate.strategy == apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_H){

		geometry_msgs::Pose target_pose, align_pose;

		target_pose.orientation.x = 1;
		target_pose.orientation.y = 0;
		target_pose.orientation.z = 0;
		target_pose.orientation.w = 0;
		target_pose.position.x = bin_center.x();
		target_pose.position.y = bin_center.y() + stow_y_offset_;
		target_pose.position.z = bin_center.z();

		double clearance = bin_lips.at(top_lip).signedDistance(dr::toEigen(target_pose).translation());
		target_pose.position.z = bin_center.z() + clearance - stow_gripper_top_bin_edge_clearing_;

		align_pose = target_pose;
		align_pose.position.y = master_pose.position.y;

		waypoints.push_back(align_pose);
		waypoints.push_back(target_pose);

	} else if (req.grasp_candidate.strategy == apc16delft_msgs::GraspCandidate::VACUUM_H_BIG) {
		geometry_msgs::Pose target_pose, align_pose;

		target_pose.orientation.x = 1;
		target_pose.orientation.y = 0;
		target_pose.orientation.z = 0;
		target_pose.orientation.w = 0;
		target_pose.position.x = bin_center.x();
		target_pose.position.y = bin_center.y() + stow_y_offset_;
		target_pose.position.z = bin_center.z();

		double clearance       = bin_lips.at(top_lip).signedDistance(dr::toEigen(target_pose).translation());
		target_pose.position.z = bin_center.z() + clearance - stow_gripper_top_bin_edge_clearing_ - 0.05;
		ROS_ERROR_STREAM("Before " << target_pose);
		target_pose            = dr::toRosPose(dr::toEigen(target_pose) * dr::rotateY(deg2rad(45.0)));
		ROS_ERROR_STREAM("After " << target_pose);

		align_pose = target_pose;
		align_pose.position.y = master_pose.position.y;

		waypoints.push_back(align_pose);
		waypoints.push_back(target_pose);
	} else if (req.grasp_candidate.strategy == apc16delft_msgs::GraspCandidate::VACUUM_V_BIG) {
		geometry_msgs::Pose target_pose, align_pose;

		target_pose.orientation.x = 1;
		target_pose.orientation.y = 0;
		target_pose.orientation.z = 0;
		target_pose.orientation.w = 0;
		target_pose.position.x = bin_center.x();
		target_pose.position.y = bin_center.y() + stow_y_offset_;
		target_pose.position.z = bin_center.z();

		double clearance = bin_lips.at(top_lip).signedDistance(dr::toEigen(target_pose).translation());
		target_pose.position.z = bin_center.z() + clearance - stow_gripper_top_bin_edge_clearing_;

		align_pose = target_pose;
		align_pose.position.y = master_pose.position.y;

		waypoints.push_back(align_pose);
		waypoints.push_back(target_pose);
	// Pencil cup standing
	} else if (req.grasp_candidate.strategy == apc16delft_msgs::GraspCandidate::PENCIL_CUP_STANDING ||
				req.grasp_candidate.strategy == apc16delft_msgs::GraspCandidate::PENCIL_CUP_SIDEWAYS){

		geometry_msgs::Pose rotate_pose, target_pose, align_pose;
		Eigen::Isometry3d eigen_master, eigen_target;

		eigen_master = dr::toEigen(master_pose);

		eigen_master = eigen_master * dr::rotateY(M_PI_4);
		rotate_pose = dr::toRosPose(eigen_master);
		
		target_pose.orientation.x = rotate_pose.orientation.x;
		target_pose.orientation.y = rotate_pose.orientation.y;
		target_pose.orientation.z = rotate_pose.orientation.z;
		target_pose.orientation.w = rotate_pose.orientation.w;
		target_pose.position.x = bin_center.x();  // adding a x offset for clearing the side - bigger because of where the gripper_tool2 frame is located
		target_pose.position.y = bin_center.y() + stow_y_offset_;
		target_pose.position.z = bin_center.z();  // adding a small z offset for clearing the bottom lip

		double clearance_x = bin_lips.at(right_lip).signedDistance(dr::toEigen(target_pose).translation());
		target_pose.position.x = target_pose.position.x + clearance_x - 3*side_bin_edge_clearing_;

		align_pose = target_pose;
		align_pose.position.y = master_pose.position.y;
		waypoints.push_back(align_pose);
		waypoints.push_back(target_pose);

	////////////////////////////////////////////////
	// Pencil cup inwards, dumbbel sidewards, dumbbel inwards
	} else if (req.grasp_candidate.strategy == apc16delft_msgs::GraspCandidate::PENCIL_CUP_INWARDS ||
				req.grasp_candidate.strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS ||
				req.grasp_candidate.strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_INWARDS){

		geometry_msgs::Pose rotate_pose, target_pose, align_pose;
		Eigen::Isometry3d eigen_master, eigen_target;
		Eigen::Isometry3d neg_rotate, pos_rotate;
		geometry_msgs::Pose neg_target, pos_target;
		geometry_msgs::Pose neg_align, pos_align;

		eigen_master = dr::toEigen(master_pose);

		std::vector<geometry_msgs::Pose> wp_neg, wp_pos;
		wp_neg = waypoints;
		wp_pos = waypoints;

		neg_rotate = eigen_master * dr::rotateY(-0.6*M_PI);
		pos_rotate = eigen_master * dr::rotateY(0.6*M_PI);

		rotate_pose = dr::toRosPose(eigen_master);

		neg_target = dr::toRosPose(neg_rotate);
		pos_target = dr::toRosPose(pos_rotate);

		neg_target.position.x = bin_center.x();
		neg_target.position.y = bin_center.y() + stow_y_offset_;
		neg_target.position.z = bin_center.z();  // adding a small offset for clearing the bottom lip

		double clearance_z = bin_lips.at(top_lip).signedDistance(dr::toEigen(neg_target).translation());
		neg_target.position.z = neg_target.position.z + clearance_z - 1*stow_gripper_top_bin_edge_clearing_;

		pos_target.position = neg_target.position;

		neg_align = neg_target;
		neg_align.position.y = master_pose.position.y;

		pos_align = pos_target;
		pos_align.position.y = master_pose.position.y;

		wp_neg.push_back(neg_align);
		wp_neg.push_back(neg_target);

		wp_pos.push_back(pos_align);
		wp_pos.push_back(pos_target);

		robot_state_->setJointGroupPositions(jmg_, joint_values);
		current_group_->setStartState(*robot_state_);

		for (int i = 0; i < 10; i++){ //Looping because of strict computeCartesian jump value.
			if(getTrajectoryFromWaypoints(*robot_state_, wp_neg, plan, true)){
				sol_found = true;
				break;
			}
		}
		if (!sol_found){
			for (int i = 0; i < 10; i++){ //Looping because of strict computeCartesian jump value.
				if(getTrajectoryFromWaypoints(*robot_state_, wp_pos, plan, true)){
					sol_found = true;
					break;
				}
			}
		}

	} else {
		ROS_ERROR_STREAM("unrecognized strategy!");
		res.error.code    = 1; // TODO
		res.error.message = "Unrecognized trajectory.";
		return true;
	}


	if (!sol_found){ //if sol is not found yet, find it and parameterize!
		robot_state_->setJointGroupPositions(jmg_, joint_values);
		current_group_->setStartState(*robot_state_);
		for (int i = 0; i < 10; i++){ //Looping because of strict computeCartesian jump value.
			if(getTrajectoryFromWaypoints(*robot_state_, waypoints, plan, true)){
				sol_found = true;
				break;
			}
		}
	}

	if (!sol_found) { // if still not found, report error.
		res.error.code    = 1; // TODO
		res.error.message = "Collision while going for stow motion.";
		return true;
	} else {
		res.release_waypoint = plan.trajectory_.joint_trajectory.points.size();
		trajectory_msgs::JointTrajectory reverse_traj = plan.trajectory_.joint_trajectory;
		std::reverse(reverse_traj.points.begin(), reverse_traj.points.end());

		// insert the retreat
		plan.trajectory_.joint_trajectory.points.insert(plan.trajectory_.joint_trajectory.points.end(),
			reverse_traj.points.begin()+1,
			reverse_traj.points.end());

		getExecutableTrajectory(plan.trajectory_, *robot_state_);
		res.error.code = apc16delft_msgs::Error::SUCCESS;
		res.trajectory = plan.trajectory_.joint_trajectory;
		return true;
	}
	return true;
}

bool ManipulationPlanner::getJointPathFromWaypoints(
	const robot_state::RobotState &robot_state,
	std::vector<geometry_msgs::Pose> &waypoints,
	move_group_interface::MoveGroup::Plan &executable_plan,
	bool enable_collision_check
) {
	moveit_msgs::RobotTrajectory motion_trajectory;
	if(!enable_collision_check) {
		ROS_WARN_STREAM ("ATTENTION: Received fine motion request without collision check. Watch out!!!");
	}

	// Setting start state for computing Cartesian Path
	current_group_->setStartState(robot_state);
	double frac = current_group_->computeCartesianPath (waypoints, 0.01, 4.0, motion_trajectory, enable_collision_check);
	if (frac < 1.0) {
		//Collisions. Nothing further to do.
		return false;
	} else {
		executable_plan.trajectory_ = motion_trajectory;
		return true;
	}
}

bool ManipulationPlanner::appendCartesianToJointTrajectory(
	trajectory_msgs::JointTrajectory & output,
	std::vector<geometry_msgs::Pose> const & waypoints,
	boost::optional<MasterPoseDescriptor> master_pose
) {
	move_group_interface::MoveGroup::Plan plan;
	if (!getTrajectoryFromWaypoints(*robot_state_, waypoints, plan, true, master_pose)) {
		return false;
	}

	if (output.joint_names.empty()) {
		output.joint_names = plan.trajectory_.joint_trajectory.joint_names;
	}

	std::vector<trajectory_msgs::JointTrajectoryPoint> const & points = plan.trajectory_.joint_trajectory.points;
	output.points.insert(output.points.end(), points.begin(), points.end());
	return true;
}

bool ManipulationPlanner::testWaypoints(int bin_index, int strategy, std::array<std::vector<Eigen::Isometry3d>, 4> & waypoints, apc16delft_msgs::PlanManipulation::Response & res) {
	std::vector<geometry_msgs::Pose> approach_waypoints = eigenPosesToRos(waypoints.at(0));
	std::vector<geometry_msgs::Pose> contact_waypoints  = eigenPosesToRos(waypoints.at(1));
	std::vector<geometry_msgs::Pose> lift_waypoints     = eigenPosesToRos(waypoints.at(2));
	std::vector<geometry_msgs::Pose> retreat_waypoints  = eigenPosesToRos(waypoints.at(3));

	//XZ motion and pre-grasp.
	move_group_interface::MoveGroup::Plan complete_plan;
	trajectory_msgs::JointTrajectory & trajectory = complete_plan.trajectory_.joint_trajectory;

	// Set the start state of the trajectory to be the master pose.
	std::vector<double> joint_values;
	getMasterJoints(joint_values, containerMasterPose(bin_index));

	if (!checkStateCollision(approach_waypoints.back())){
		ROS_WARN_STREAM("Grasp pose in collision");
		res.error.code = apc16delft_msgs::PlanManipulationRequest::E_COLLISION;
		res.error.message = "Grasp pose is in Collision!";
		return true;
	} else if (!checkStateCollision(contact_waypoints.back())){
		ROS_WARN_STREAM("Contact pose in collision");
		res.error.code = apc16delft_msgs::PlanManipulationRequest::E_COLLISION;
		res.error.message = "Contact pose is in Collision!";
		return true;
	} else if (!checkStateCollision(lift_waypoints.back())){
		ROS_WARN_STREAM("Lift pose in collision");
		res.error.code = apc16delft_msgs::PlanManipulationRequest::E_COLLISION;
		res.error.message = "Lift pose is in Collision!";
		return true;
	} else if (!checkStateCollision(retreat_waypoints.back())){
		ROS_WARN_STREAM("Retreat pose in collision");
		res.error.code = apc16delft_msgs::PlanManipulationRequest::E_COLLISION;
		res.error.message = "Retreat pose is in Collision!";
		return true;
	}

	robot_state_->setJointGroupPositions(jmg_, joint_values);
	current_group_->setStartState(*robot_state_);
	std::vector<apc16delft_msgs::Milestone> trajectory_milestones;
	apc16delft_msgs::Milestone trajectory_milestone;

	for (size_t i = 0; i < approach_waypoints.size(); i++) {
		ROS_INFO_STREAM("Approach waypoint " << i << ": " << approach_waypoints[i]);
	}

	ROS_INFO_STREAM("Approach waypoints: " << approach_waypoints.size());
	ROS_INFO_STREAM("Contact waypoints: " << contact_waypoints.size());
	ROS_INFO_STREAM("Lift waypoints: " << lift_waypoints.size());
	ROS_INFO_STREAM("Retreat waypoints: " << retreat_waypoints.size());

	trajectory_msgs::JointTrajectory approach, contact, lift, retreat;

	if (!appendCartesianToJointTrajectory(trajectory, approach_waypoints)) {
		res.error.code = apc16delft_msgs::PlanManipulationRequest::E_COLLISION;
		res.error.message = "Collision while going for pre-grasp.";
		return true;
	}

	ROS_WARN_STREAM("Pre-grasp trajectory computed to be collision-free.");
	// Add approach to the final trajectory.
	switch (strategy) {
		case apc16delft_msgs::GraspCandidate::VACUUM_H_BIG:
		case apc16delft_msgs::GraspCandidate::VACUUM_V_BIG:
		case apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_H:
		case apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_V: {
			// Add Vacuum Relay ON milestone.
			trajectory_milestone.waypoint = trajectory.points.size() / 2;
			trajectory_milestone.event    = apc16delft_msgs::Milestone::VACUUM_POWER_ON;

			// Add Suction ON milestone.
			trajectory_milestones.push_back(trajectory_milestone);
			trajectory_milestone.waypoint = trajectory.points.size();
			trajectory_milestone.event    = apc16delft_msgs::Milestone::SUCTION_ON;
			trajectory_milestones.push_back(trajectory_milestone);
			break;
		}
		case apc16delft_msgs::GraspCandidate::DUMBBELL_INWARDS:
		case apc16delft_msgs::GraspCandidate::PENCIL_CUP_SIDEWAYS:
		case apc16delft_msgs::GraspCandidate::PENCIL_CUP_STANDING:
		case apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS:
		case apc16delft_msgs::GraspCandidate::PENCIL_CUP_INWARDS:
			// Nothing to do because coordinator has done the initial IO.
			break;
		default:
			res.error.code = apc16delft_msgs::PlanManipulationRequest::E_INVALID_STRATEGY;
			res.error.message = "Wrong grasp strategy!";
			return true;
			break;
	}

	for (size_t i = 0; i < contact_waypoints.size(); i++) {
		ROS_INFO_STREAM("Contact waypoint " << i << ": " << contact_waypoints[i]);
	}
	//Grasp the object.
	robot_state_->setVariablePositions(trajectory.joint_names, trajectory.points.back().positions);
	current_group_->setStartState(*robot_state_);
	ROS_WARN_STREAM("Size of contact waypoints: " << contact_waypoints.size());
	if (!appendCartesianToJointTrajectory(trajectory, contact_waypoints)) {
		res.error.code = apc16delft_msgs::PlanManipulationRequest::E_COLLISION;
		res.error.message = "Collision while going for grasp.";
		return true;
	}
	ROS_WARN_STREAM("Contact trajectory computed to be collision-free.");
	ROS_WARN_STREAM("Size of approach waypoints: " << contact_waypoints.size());
	// Add contact to the final trajectory.
	switch (strategy) {
		case apc16delft_msgs::GraspCandidate::VACUUM_H_BIG:
		case apc16delft_msgs::GraspCandidate::VACUUM_V_BIG:
		case apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_H:
		case apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_V: {
			// Add Contact milestone.
			trajectory_milestone.waypoint = trajectory.points.size();
			trajectory_milestone.event    = apc16delft_msgs::Milestone::CONTACT;
			trajectory_milestones.push_back(trajectory_milestone);
			break;
		}
		case apc16delft_msgs::GraspCandidate::DUMBBELL_INWARDS: {
			// Nothing to do because coordinator has done the initial IO.
			trajectory_milestone.waypoint = trajectory.points.size();
			trajectory_milestone.event    = apc16delft_msgs::Milestone::THUMB_EXTENDED;
			trajectory_milestones.push_back(trajectory_milestone);
			break;
		}
		case apc16delft_msgs::GraspCandidate::PENCIL_CUP_SIDEWAYS:
		case apc16delft_msgs::GraspCandidate::PENCIL_CUP_STANDING:
		case apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS:
		case apc16delft_msgs::GraspCandidate::PENCIL_CUP_INWARDS: {
			// Nothing to do because coordinator has done the initial IO.
			trajectory_milestone.waypoint = trajectory.points.size();
			trajectory_milestone.event    = apc16delft_msgs::Milestone::SUCTION_ROTATED;
			trajectory_milestones.push_back(trajectory_milestone);
			break;
		}
		default:
			res.error.code = apc16delft_msgs::PlanManipulationRequest::E_INVALID_STRATEGY;
			res.error.message = "Wrong grasp strategy!";
			return true;
			break;
	}

	//Post-grasp.
	robot_state_->setVariablePositions(trajectory.joint_names, complete_plan.trajectory_.joint_trajectory.points.back().positions);
	current_group_->setStartState(*robot_state_);
	if (!appendCartesianToJointTrajectory(trajectory, lift_waypoints)) {
		res.error.code = apc16delft_msgs::PlanManipulationRequest::E_COLLISION;
		res.error.message = "Collision while going for post-grasp.";
		return true;
	}
	ROS_WARN_STREAM("Lift trajectory computed to be collision-free.");

	//Add Lift milestone.
	trajectory_milestone.waypoint = trajectory.points.size();
	trajectory_milestone.event    = apc16delft_msgs::Milestone::LIFT;
	trajectory_milestones.push_back(trajectory_milestone);

	//Retreat to bin master pose.
	robot_state_->setVariablePositions(trajectory.joint_names, complete_plan.trajectory_.joint_trajectory.points.back().positions);
	current_group_->setStartState(*robot_state_);
	if (!appendCartesianToJointTrajectory(trajectory, retreat_waypoints, containerMasterPose(bin_index))) {
		res.error.code = apc16delft_msgs::PlanManipulationRequest::E_COLLISION;
		res.error.message = "Collision while retreating to master pose.";
		return true;
	}

	ROS_WARN_STREAM("Retreat trajectory computed to be collision-free.");

	res.trajectory = complete_plan.trajectory_.joint_trajectory;
	res.milestones = trajectory_milestones;
	res.error.code = apc16delft_msgs::Error::SUCCESS;
	res.error.message = "All is Well! Every fine motion is Happy!";

	return true;
}

std::array<std::vector<Eigen::Isometry3d>, 4> ManipulationPlanner::planTotePick(const Eigen::Isometry3d & current_pose, apc16delft_msgs::GraspCandidate & grasp) {
	Eigen::Isometry3d grasp_pose = dr::toEigen(grasp.stamped_pose.pose);
	Eigen::Vector3d grasp_pose_z = grasp_pose.rotation() * dr::axes::z();

	double offset = grasp.pre_grasp_offset;
	Eigen::Isometry3d pre_grasp_pose = grasp_pose * dr::translate(0.0, 0.0, -offset);
	std::vector<Eigen::Isometry3d> approach_waypoints, contact_waypoints, lift_waypoints, retreat_waypoints;
	approach_waypoints.push_back(current_pose);

	// Align with target.
	Eigen::Isometry3d waypoint_2     = pre_grasp_pose;
	waypoint_2.translation().z()     = current_pose.translation().z();
	approach_waypoints.push_back(waypoint_2);

	// Move into the tote,
	Eigen::Isometry3d waypoint_3     = pre_grasp_pose;
	approach_waypoints.push_back(waypoint_3);

	// Move towards the grasp pose.
	Eigen::Isometry3d waypoint_4     = grasp_pose;
	contact_waypoints.push_back(waypoint_4);

	// Move out of the tote.
	Eigen::Isometry3d waypoint_5     = waypoint_4;
	waypoint_5.translation().z()     = current_pose.translation().z();
	lift_waypoints.push_back(waypoint_5);
	retreat_waypoints.push_back(waypoint_5);

	geometry_msgs::PoseArray visualization_poses;
	visualization_poses.header.stamp = ros::Time::now();
	visualization_poses.header.frame_id = "world";
	visualization_poses.poses.push_back(dr::toRosPose(current_pose));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_2));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_3));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_4));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_5));
	waypoint_visualiser_.publish(visualization_poses);

	return std::array<std::vector<Eigen::Isometry3d>, 4>{{approach_waypoints, contact_waypoints, lift_waypoints, retreat_waypoints}};
}

std::array<std::vector<Eigen::Isometry3d>, 4> ManipulationPlanner::planPinchPath(int bin_index, const Eigen::Isometry3d & bin_pose, const Eigen::Isometry3d & current_pose, apc16delft_msgs::GraspCandidate & grasp) {
	Eigen::Isometry3d grasp_pose = dr::toEigen(grasp.stamped_pose.pose);
	Eigen::Vector3d grasp_pose_z = grasp_pose.rotation() * dr::axes::z();
	Eigen::Vector3d bin_center   = getBinCenter(bin_pose, bin_dimensions_.at(bin_index));
	double offset                = grasp.pre_grasp_offset;

	Eigen::Isometry3d pre_grasp_pose;
	if (grasp.strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS) {
		pre_grasp_pose = grasp_pose * dr::translate(0.0, 8.0*offset, -offset);
	} else if (grasp.strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_INWARDS) {
		pre_grasp_pose = dr::translate(0.0, 0.0, dumbbell_inwards_pre_) * grasp_pose;
	} else if (grasp.strategy == apc16delft_msgs::GraspCandidate::PENCIL_CUP_INWARDS) {
		pre_grasp_pose = grasp_pose * dr::translate(0.0, 8.0*offset, -offset);
	} else {
		pre_grasp_pose = grasp_pose * dr::translate(0.0, 0.0, -offset);
	}

	std::vector<Eigen::Isometry3d> approach_waypoints, contact_waypoints, lift_waypoints, retreat_waypoints;
	approach_waypoints.push_back(current_pose);

	// Align with target.
	Eigen::Isometry3d waypoint_1     = pre_grasp_pose;
	waypoint_1.translation().y()     = current_pose.translation().y();
	approach_waypoints.push_back(waypoint_1);

	// Move into the bin.
	Eigen::Isometry3d waypoint_2     = pre_grasp_pose;
	approach_waypoints.push_back(waypoint_2);

	// Move towards the grasp pose.
	Eigen::Isometry3d waypoint_3     = grasp_pose;
	contact_waypoints.push_back(waypoint_3);

	// Lift the item.
	Eigen::Isometry3d waypoint_4     = dr::translate(0.0, 0.0, offset) * grasp_pose;
	waypoint_4.translation().x()     = bin_center.x();
	lift_waypoints.push_back(waypoint_4);

	// Move out of bin.
	Eigen::Isometry3d waypoint_5     = waypoint_4;
	waypoint_5.translation().y()     = current_pose.translation().y();
	retreat_waypoints.push_back(waypoint_5);

	geometry_msgs::PoseArray visualization_poses;
	visualization_poses.header.stamp = ros::Time::now();
	visualization_poses.header.frame_id = "world";
	visualization_poses.poses.push_back(dr::toRosPose(current_pose));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_1));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_2));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_3));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_4));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_5));
	waypoint_visualiser_.publish(visualization_poses);

	return std::array<std::vector<Eigen::Isometry3d>, 4>{{approach_waypoints, contact_waypoints, lift_waypoints, retreat_waypoints}};
}

std::array<std::vector<Eigen::Isometry3d>, 4> ManipulationPlanner::planTotePinch(const Eigen::Isometry3d & current_pose, apc16delft_msgs::GraspCandidate & grasp) {
	Eigen::Isometry3d grasp_pose = dr::toEigen(grasp.stamped_pose.pose);
	Eigen::Vector3d grasp_pose_z = grasp_pose.rotation() * dr::axes::z();
	double offset                = grasp.pre_grasp_offset;

	Eigen::Isometry3d pre_grasp_pose;
	if (grasp.strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS) {
		pre_grasp_pose = grasp_pose * dr::translate(0.0, 3.0*offset, -offset);
	} else if (grasp.strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_INWARDS) {
		pre_grasp_pose = grasp_pose * dr::translate(0.0, 0.0, -dumbbell_inwards_pre_);
	} else if (grasp.strategy == apc16delft_msgs::GraspCandidate::PENCIL_CUP_INWARDS) {
		pre_grasp_pose = grasp_pose;
	} else {
		pre_grasp_pose = grasp_pose * dr::translate(0.0, 0.0, -offset);
	}

	std::vector<Eigen::Isometry3d> approach_waypoints, contact_waypoints, lift_waypoints, retreat_waypoints;
	approach_waypoints.push_back(current_pose);

	// Align with target.
	Eigen::Isometry3d waypoint_2 = pre_grasp_pose;

	if (grasp.strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS) {
		waypoint_2 = current_pose;
		waypoint_2.translation().x() = pre_grasp_pose.translation().x();
		waypoint_2.translation().y() = pre_grasp_pose.translation().y();
	} else {
		waypoint_2.translation().z() = current_pose.translation().z();
	}

	approach_waypoints.push_back(waypoint_2);

	// Move into the tote,
	Eigen::Isometry3d waypoint_3     = pre_grasp_pose;
	approach_waypoints.push_back(waypoint_3);

	// Move towards the grasp pose.
	Eigen::Isometry3d waypoint_4     = grasp_pose;
	contact_waypoints.push_back(waypoint_4);

	// Move out of the tote.
	Eigen::Isometry3d waypoint_5     = waypoint_4;
	waypoint_5.translation().z()       = current_pose.translation().z();
	lift_waypoints.push_back(waypoint_5);
	retreat_waypoints.push_back(waypoint_5);

	geometry_msgs::PoseArray visualization_poses;
	visualization_poses.header.stamp = ros::Time::now();
	visualization_poses.header.frame_id = "world";
	visualization_poses.poses.push_back(dr::toRosPose(current_pose));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_2));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_3));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_4));
	visualization_poses.poses.push_back(dr::toRosPose(waypoint_5));
	waypoint_visualiser_.publish(visualization_poses);

	return std::array<std::vector<Eigen::Isometry3d>, 4>{{approach_waypoints, contact_waypoints, lift_waypoints, retreat_waypoints}};
}

bool ManipulationPlanner::planToteManipulation(apc16delft_msgs::PlanManipulation::Request & req, apc16delft_msgs::PlanManipulation::Response & res) {
	std::string tool = getWorkingGroupFromStrategy(req.grasp.strategy);
	if(unknown_grasp_strategy_) {
		res.error.code = 1;
		res.error.message = "Unknown grasp strategy. Plan request rejected!";
		unknown_grasp_strategy_ = false;
		return true;
	}
	current_group_->clearPoseTargets ();

	std::array<std::vector<Eigen::Isometry3d>, 4> result;
	apc16delft_msgs::GraspCandidate grasp = req.grasp;

	geometry_msgs::Pose gripper_master_pose;
	if (!getMasterPose(gripper_master_pose, containerMasterPose(req.bin_index, current_group_->getName()))) {
		res.error.code = apc16delft_msgs::PlanManipulationRequest::E_INVALID_BIN;
		res.error.message = "Could not find master pose from index.";
		return true;
	}

	switch (req.grasp.strategy) {
		case apc16delft_msgs::GraspCandidate::VACUUM_H_BIG:
		case apc16delft_msgs::GraspCandidate::VACUUM_V_BIG:
		case apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_H:
		case apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_V:
			result = planTotePick(dr::toEigen(gripper_master_pose), grasp);
			break;
		case apc16delft_msgs::GraspCandidate::DUMBBELL_INWARDS:
		case apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS:
		case apc16delft_msgs::GraspCandidate::PENCIL_CUP_STANDING:
		case apc16delft_msgs::GraspCandidate::PENCIL_CUP_SIDEWAYS:
		case apc16delft_msgs::GraspCandidate::PENCIL_CUP_INWARDS:
			result = planTotePinch(dr::toEigen(gripper_master_pose), grasp);
		default:
			res.error.code = apc16delft_msgs::PlanManipulationRequest::E_INVALID_STRATEGY;
			res.error.message = "Wrong grasp strategy!";
	}

//	result.at(1).insert(result.at(1).begin(), result.at(0).back());
//	result.at(2).insert(result.at(2).begin(), result.at(1).back());
//	result.at(3).insert(result.at(3).begin(), result.at(2).back());

	return testWaypoints(req.bin_index, req.grasp.strategy, result, res);
}

bool ManipulationPlanner::planManipulation(apc16delft_msgs::PlanManipulation::Request & req, apc16delft_msgs::PlanManipulation::Response & res) {
	std::string tool = getWorkingGroupFromStrategy(req.grasp.strategy);
	if(unknown_grasp_strategy_) {

		res.error.code = apc16delft_msgs::PlanManipulationRequest::E_INVALID_STRATEGY;
		res.error.message = "Wrong grasp strategy!";
		return true;
	}
	current_group_->clearPoseTargets ();

	std::array<std::vector<Eigen::Isometry3d>, 4> result;
	apc16delft_msgs::GraspCandidate grasp = req.grasp;
	Eigen::Vector3d bin_sizes;
	bin_sizes = bin_dimensions_.at(req.bin_index);

	geometry_msgs::Pose gripper_master_pose;
	if (!getMasterPose(gripper_master_pose, containerMasterPose(req.bin_index, current_group_->getName()))) {
		res.error.code = apc16delft_msgs::PlanManipulationRequest::E_INVALID_BIN;
		res.error.message = "Could not find master pose from index.";
		return true;
	}

	if (isDeformable(req.object_type)) {
		result = planDeformablePath(req.bin_index, dr::toEigen(req.bin_pose.pose), dr::toEigen(gripper_master_pose), grasp, req.grasp.angled_approach, req.grasp.side_angled_approach);
	} else {
		std::array<double, 5> clearances = calculateClearances(req);
		switch (req.grasp.strategy) {
			case apc16delft_msgs::GraspCandidate::VACUUM_H_BIG:
			case apc16delft_msgs::GraspCandidate::VACUUM_V_BIG:
			case apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_H:
			case apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_V:
				try {
					result = planCartesianPath(req.object_type, req.bin_index, dr::toEigen(req.bin_pose.pose), dr::toEigen(gripper_master_pose), grasp, clearances, req.grasp.angled_approach, req.grasp.side_angled_approach);
				} catch(const std::runtime_error & e) {
						res.error.code = apc16delft_msgs::PlanManipulationRequest::E_OBJECT_OUTSIDE_BIN;
						res.error.message = e.what();
						return true;
				}
				break;
			case apc16delft_msgs::GraspCandidate::DUMBBELL_INWARDS:
			case apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS:
			case apc16delft_msgs::GraspCandidate::PENCIL_CUP_STANDING:
			case apc16delft_msgs::GraspCandidate::PENCIL_CUP_SIDEWAYS:
			case apc16delft_msgs::GraspCandidate::PENCIL_CUP_INWARDS:
				result = planPinchPath(req.bin_index, dr::toEigen(req.bin_pose.pose), dr::toEigen(gripper_master_pose), grasp);
				break;
			default:
				res.error.code = apc16delft_msgs::PlanManipulationRequest::E_INVALID_STRATEGY;
				res.error.message = "Wrong grasp strategy!";
		}
	}

	result.at(1).insert(result.at(1).begin(), result.at(0).back());
	result.at(2).insert(result.at(2).begin(), result.at(1).back());
	result.at(3).insert(result.at(3).begin(), result.at(2).back());

	//Publish bin collision object to the planning scene.
	apc16delft_msgs::AddCollisionObject add_co_service;
	add_co_service.request.pose_collision_object = req.bin_pose;
	add_co_service.request.bin_index = req.bin_index;
	add_collision_object_.call(add_co_service);

	return testWaypoints(req.bin_index, req.grasp.strategy, result, res);
}

bool ManipulationPlanner::planCameraMotion(apc16delft_msgs::PlanCameraMotion::Request & req, apc16delft_msgs::PlanCameraMotion::Response & res) {
	//Set robot state container to point to manipulator_tool0 state.
	std::vector<double> joint_values, current_values;
	if (!getMasterJoints(joint_values, camMasterPose(req.bin_index))) {
		res.error.code = 1;
		res.error.message = "Camera master pose not found!";
		return true;
	}

	robot_state::RobotState state = robotStateFromJoints(joint_values);
	ROS_INFO_STREAM("Got robot state from joints..");
	tool0_group_.setStartState(state);

	Eigen::Affine3d master_pose = state.getGlobalLinkTransform(tool0_group_.getEndEffectorLink());

	std::string const & frame = req.relative_transform.header.frame_id;
	Eigen::Isometry3d relative_transform = dr::toEigen(req.relative_transform.pose);
	const Eigen::Affine3d link_transform = state.getGlobalLinkTransform(frame);
	
	Eigen::Affine3d target_pose = link_transform * relative_transform;
	state.updateStateWithLinkAt(frame, target_pose);
	target_pose = state.getGlobalLinkTransform(tool0_group_.getEndEffectorLink());

	geometry_msgs::Pose target_ros_pose = dr::toRosPose(affineToIsometry(target_pose));
	geometry_msgs::Pose master_ros_pose = dr::toRosPose(affineToIsometry(master_pose));

	moveit::planning_interface::MoveGroup::Plan plan;
	std::vector<geometry_msgs::Pose> waypoints;

	waypoints.push_back(master_ros_pose);
	waypoints.push_back(target_ros_pose);

	if (target_ros_pose.position.y - waypoints.back().position.y <= 0.1) {
		if (!getTrajectoryFromWaypoints(state, waypoints, plan, true)) {
			res.error.code = apc16delft_msgs::PlanManipulationRequest::E_COLLISION;
			res.error.message = "Collision detected while attempting camera motion.";
			return true;
		} else {
			// found first trajectory, now calculate second
			res.camera_trajectory = plan.trajectory_.joint_trajectory;
			std::reverse(plan.trajectory_.joint_trajectory.points.begin(), plan.trajectory_.joint_trajectory.points.end());

			// clearing time param
			for(size_t idx = 0; idx < plan.trajectory_.joint_trajectory.points.size(); idx++) {
				plan.trajectory_.joint_trajectory.points[idx].velocities.clear();
				plan.trajectory_.joint_trajectory.points[idx].accelerations.clear();
			}

			getExecutableTrajectory(plan.trajectory_, state);
			res.error.code = apc16delft_msgs::Error::SUCCESS;
			res.error.message = "Successfully planned camera adjusemtent motion";
			res.reverse_trajectory = plan.trajectory_.joint_trajectory;
			return true;
		}
	} else {
		res.error.code = 1;
		res.error.message = "Rejecting relative pose due to safety reasons.";
		return true;
	}
}

void ManipulationPlanner::resetVisualisation(const std::string & frame_id) {
	pinch_pose_array_.poses.clear();
	vacuum_pose_array_.poses.clear();

	pinch_pose_array_.header.frame_id  = frame_id;
	vacuum_pose_array_.header.frame_id = frame_id;
	pinch_pose_array_.header.stamp     = ros::Time::now();
	vacuum_pose_array_.header.stamp    = ros::Time::now();
}

bool ManipulationPlanner::updateReachability(apc16delft_msgs::PruneGraspCandidates::Request & req, apc16delft_msgs::PruneGraspCandidates::Response & res) {
	std::vector<apc16delft_msgs::GraspCandidate> grasp_candidates_in = req.candidates;
	resetVisualisation(req.candidates.at(0).stamped_pose.header.frame_id);
	ROS_INFO_STREAM("Requesting to update reachability for bin index " << (int) req.bin_index);

	if (grasp_candidates_in.empty()) return true;

	// Remove the rejected one, which is the first one since the vector is sorted.
	apc16delft_msgs::GraspCandidate rejected_candidate = grasp_candidates_in.at(0);
	Eigen::Vector3d rejected_candidate_translation = dr::toEigen(rejected_candidate.stamped_pose.pose.position);
	Eigen::Vector3d rejected_candidate_z = dr::toEigen(rejected_candidate.stamped_pose.pose).rotation() * dr::axes::z();
	grasp_candidates_in.erase(grasp_candidates_in.begin());

	std::string model_name = apc16delft_msgs::objectTypeToString(req.object_type);
	std::string param_path = "/item/" + model_name + "/reachability_prune_distance";
	double reachability_prune_distance = dr::getParam<double>(node_handle, param_path, 0.015);

	// Skip the vectors with the same direction as the rejected one.
	for (const apc16delft_msgs::GraspCandidate & candidate : grasp_candidates_in) {
		Eigen::Vector3d candidate_z = dr::toEigen(candidate.stamped_pose.pose).rotation() * dr::axes::z();
		Eigen::Vector3d candidate_translation = dr::toEigen(candidate.stamped_pose.pose.position);
		double distance = (candidate_translation - rejected_candidate_translation).norm();
		if (sameDirection(rejected_candidate_z, candidate_z) && distance < reachability_prune_distance)
			continue;

		vacuum_pose_array_.poses.push_back(candidate.stamped_pose.pose);
		res.pruned_candidates.push_back(candidate);
	}

	vacuum_visualiser_.publish(vacuum_pose_array_);

	return true;
}

} // namespace

int main(int argc, char ** argv) {
	ros::init(argc, argv, "manipulation_planner");
	apc16delft::ManipulationPlanner manipulation_planner;
	ros::Rate loop_rate(1000);
	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::waitForShutdown();
	return 0;

}
