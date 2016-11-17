#pragma once
#include "motion_visualizer.hpp"
#include "master_pose.hpp"
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <apc16delft_grasp_synthesizer/param.hpp>
#include <apc16delft_msgs/PlanManipulation.h>
#include <apc16delft_msgs/PruneGraspCandidates.h>
#include <apc16delft_util/managed_node.hpp>
#include <apc16delft_msgs/GraspCandidate.h>
#include <apc16delft_msgs/PlanCameraMotion.h>
#include <apc16delft_msgs/PlanStowMotion.h>
#include <apc16delft_msgs/AddCollisionObject.h>

#include <moveit_msgs/GetPositionIK.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <boost/optional.hpp>

namespace apc16delft {

class ManipulationPlanner : public ManagedNode {

using MasterPoseDescriptor = apc16delft_msgs::MasterPoseDescriptor;

public:
	ManipulationPlanner() :
		spinner(1),
		motion_visualizer_(node_handle),
		tool0_group_("manipulator_tool0"),
		tool1_group_("manipulator_tool1"),
		tool2_group_("manipulator_tool2"),
		current_group_(&tool0_group_)
	{
		spinner.start();
	};

protected:
	ros::AsyncSpinner spinner;
	ros::ServiceServer update_reachability_;
	ros::ServiceServer plan_manipulation_;
	ros::ServiceServer plan_tote_manipulation_;
	ros::ServiceServer plan_camera_motion_;
	ros::ServiceServer plan_stow_motion_;

	ros::ServiceClient add_collision_object_;
	ros::ServiceClient ik_service_;

	MotionVisualizer motion_visualizer_;

	/// Publisher for visualising vacuum candidates.
	ros::Publisher vacuum_visualiser_;

	/// Publisher for visualising waypoints.
	ros::Publisher waypoint_visualiser_;

	/// List of all item models.
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> model_list_;

	/// PoseArrays for visualisation.
	geometry_msgs::PoseArray pinch_pose_array_;
	geometry_msgs::PoseArray vacuum_pose_array_;

	robot_state::RobotStatePtr robot_state0_;
	robot_state::RobotStatePtr robot_state1_;
	robot_state::RobotStatePtr robot_state2_;
	robot_state::RobotStatePtr robot_state_;

	const robot_state::JointModelGroup *jmg_;
	const robot_state::JointModelGroup *jmg0_;
	const robot_state::JointModelGroup *jmg1_;
	const robot_state::JointModelGroup *jmg2_;

private:
	bool visualise_                       = true;
	int number_of_items_                  = 39;
	double stow_y_offset_                 = 0.08;
	double dumbbell_inwards_pre_          = 0.05;
	double gripper_top_bin_edge_clearing_ = 0.09;
	double stow_gripper_top_bin_edge_clearing_ = 0.09;
	double top_bin_edge_clearing_         = 0.06;
	double bottom_bin_edge_clearing_      = 0.05;
	double side_bin_edge_clearing_        = 0.05;
	double back_bin_edge_clearing_        = 0.02;
	double approach_angle_                = 20.0;
	double side_approach_angle_           = 20.0;
	double trajectory_velocity_scaling_   = 0.025;
	double unknown_grasp_strategy_         = false;

	std::vector<Eigen::Vector3d> bin_dimensions_;
	Eigen::Vector3d tote_dimensions_;
	void resetVisualisation(const std::string & frame_id);

protected:
	virtual int onConfigure() override;
	virtual int onActivate() override;

	bool testWaypoints(int bin_index, int strategy, std::array<std::vector<Eigen::Isometry3d>, 4> & waypoints, apc16delft_msgs::PlanManipulation::Response & res);
	bool updateReachability(apc16delft_msgs::PruneGraspCandidates::Request & req, apc16delft_msgs::PruneGraspCandidates::Response & res);
	std::string getWorkingGroupFromStrategy(int strategy);
	bool planManipulation(apc16delft_msgs::PlanManipulation::Request & req, apc16delft_msgs::PlanManipulation::Response & res);
	bool planToteManipulation(apc16delft_msgs::PlanManipulation::Request & req, apc16delft_msgs::PlanManipulation::Response & res);
	std::array<std::vector<Eigen::Isometry3d>, 4> planTotePick(const Eigen::Isometry3d & current_pose, apc16delft_msgs::GraspCandidate & grasp);
	std::array<std::vector<Eigen::Isometry3d>, 4> planTotePinch(const Eigen::Isometry3d & current_pose, apc16delft_msgs::GraspCandidate & grasp);
	std::array<std::vector<Eigen::Isometry3d>, 4> planCartesianPath(uint8_t object_type, int bin_index, const Eigen::Isometry3d & bin_pose, const Eigen::Isometry3d & current_pose, apc16delft_msgs::GraspCandidate & grasp, const std::array<double, 5> & clearances, bool angled_approach, bool side_angled_approach);
	std::array<std::vector<Eigen::Isometry3d>, 4> planPinchPath(int bin_index, const Eigen::Isometry3d & bin_pose, const Eigen::Isometry3d & current_pose, apc16delft_msgs::GraspCandidate & grasp);
	std::array<std::vector<Eigen::Isometry3d>, 4> planDeformablePath(int bin_index, const Eigen::Isometry3d & bin_pose,  const Eigen::Isometry3d & current_pose, const apc16delft_msgs::GraspCandidate & grasp, bool angled_approach, bool side_angled_approach);

	std::array<double, 5> calculateClearances(apc16delft_msgs::PlanManipulation::Request & req);

	bool load3DModels(const std::map<std::string, GraspItem> & item_map);
	int loadParams();

private:
	moveit::planning_interface::MoveGroup tool0_group_;
	moveit::planning_interface::MoveGroup tool1_group_;
	moveit::planning_interface::MoveGroup tool2_group_;
	moveit::planning_interface::MoveGroup * current_group_;
protected:

	bool planStowMotion(apc16delft_msgs::PlanStowMotion::Request & req, apc16delft_msgs::PlanStowMotion::Response & res);
	bool appendCartesianToJointTrajectory(
		trajectory_msgs::JointTrajectory & output,
		std::vector<geometry_msgs::Pose> const & waypoints,
		boost::optional<MasterPoseDescriptor> master_pose = boost::none
	);
	bool getTrajectoryFromWaypoints(
		const robot_state::RobotState &robot_state,
		std::vector<geometry_msgs::Pose> const & waypoints,
		move_group_interface::MoveGroup::Plan & executable_plan,
		bool enable_collision_check,
		boost::optional<MasterPoseDescriptor> master_pose = boost::none
	);
	bool getJointPathFromWaypoints(
		const robot_state::RobotState &robot_state,
		std::vector<geometry_msgs::Pose> & waypoints,
		move_group_interface::MoveGroup::Plan & executable_plan,
		bool enable_collision_check
	);
	bool getMasterJoints(std::vector<double> & joint_values, apc16delft_msgs::MasterPoseDescriptor master_pose_descriptor);
	bool getMasterPose(geometry_msgs::Pose & pose, apc16delft_msgs::MasterPoseDescriptor master_pose_descriptor);
	robot_state::RobotState robotStateFromJoints(std::vector<double> const & joint_values);
	bool planCameraMotion(apc16delft_msgs::PlanCameraMotion::Request & req, apc16delft_msgs::PlanCameraMotion::Response & res);
	bool stitchTrajectories(moveit_msgs::RobotTrajectory & motion_trajectory, std::vector<double> joint_states);
	bool getExecutableTrajectory (moveit_msgs::RobotTrajectory & motion_trajectory, const robot_state::RobotState & robot_state, boost::optional<MasterPoseDescriptor> master_pose = boost::none);
	bool checkStateCollision(geometry_msgs::Pose & pose);
};

} // namespace

