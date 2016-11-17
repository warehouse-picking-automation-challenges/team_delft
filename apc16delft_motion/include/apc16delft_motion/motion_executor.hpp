#pragma once
#include "trajectory_cache.hpp"
#include "motion_sanity_checker.hpp"
#include "trajectory_tracker.hpp"

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <apc16delft_msgs/Error.h>
#include <apc16delft_msgs/ExecuteCoarseMotion.h>
#include <apc16delft_msgs/GetCoarseMotion.h>
#include <apc16delft_msgs/ExecuteFineMotion.h>
#include <apc16delft_msgs/ExecuteStitchedMotion.h>
#include <apc16delft_msgs/ExecuteCalibrationMotion.h>
#include <apc16delft_msgs/MoveToHome.h>
#include <apc16delft_msgs/MoveToCalibrateShelf.h>
#include <apc16delft_msgs/MoveToCalibrateTote.h>
#include <std_msgs/Int32.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <Eigen/Geometry>


namespace apc16delft {

class MotionExecutor {
public:
	/// Construct a motion executor node.
	MotionExecutor();
	enum {
		MASTERPOSE_NOT_AVAILABLE = 1,
		TARGET_POSE_NOT_REACHABLE,
		MOTION_SAFETY_VIOLATION,
		STOW_PIPELINE_NOT_AVAIALABLE
	};

protected:
	ros::AsyncSpinner spinner;

private:
	ros::NodeHandle node_handle_;
	TrajectoryCache trajectory_cache_;
	MotionSanityChecker motion_sanity_checker_;

	std::string yaml_path_;
	int calibration_trajectory_length_;
	double trajectory_velocity_scaling_;
	double calibration_trajectory_tolerance_;

	ros::ServiceServer execute_coarse_motion_;
	ros::ServiceServer get_coarse_motion_;
	ros::ServiceServer execute_fine_motion_;
	ros::ServiceServer execute_stitched_motion_;
	ros::ServiceServer execute_calibration_motion_;
	ros::ServiceServer move_to_home_;
	ros::ServiceServer move_to_calibrate_shelf_;
	ros::ServiceServer move_to_calibrate_tote_;
	ros::Publisher grasp_pose_visualizer_;

	moveit::planning_interface::MoveGroup tool0_group_;
	moveit::planning_interface::MoveGroup tool1_group_;
	moveit::planning_interface::MoveGroup * current_group_;
	moveit::planning_interface::MoveGroup::Plan plan_;

	TrajectoryTracker trajectory_tracker_;

	robot_state::RobotStatePtr rs_;

protected:
	bool executeCoarseMotion(apc16delft_msgs::ExecuteCoarseMotion::Request & req, apc16delft_msgs::ExecuteCoarseMotion::Response & res);
	bool getCoarseMotion(apc16delft_msgs::GetCoarseMotion::Request & req, apc16delft_msgs::GetCoarseMotion::Response & res);
	bool executeStitchedMotion(apc16delft_msgs::ExecuteStitchedMotion::Request & req, apc16delft_msgs::ExecuteStitchedMotion::Response & res);
	bool executeFineMotion(apc16delft_msgs::ExecuteFineMotion::Request & req, apc16delft_msgs::ExecuteFineMotion::Response & res);
	bool executeCalibrationMotion(apc16delft_msgs::ExecuteCalibrationMotion::Request & req, apc16delft_msgs::ExecuteCalibrationMotion::Response & res);
	bool moveToHome(apc16delft_msgs::MoveToHome::Request &, apc16delft_msgs::MoveToHome::Response & res);
	bool moveToCalibrateShelf(apc16delft_msgs::MoveToCalibrateShelf::Request &, apc16delft_msgs::MoveToCalibrateShelf::Response & res);
	bool moveToCalibrateTote(apc16delft_msgs::MoveToCalibrateTote::Request &, apc16delft_msgs::MoveToCalibrateTote::Response & res);
	bool checkCalibrationTrajectorySanity(trajectory_msgs::JointTrajectory & motion_trajectory, moveit::planning_interface::MoveGroup* current_group);
};

} // namespace

