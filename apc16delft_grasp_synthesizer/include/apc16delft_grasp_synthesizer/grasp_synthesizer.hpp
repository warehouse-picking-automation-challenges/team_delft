#pragma once

#include <ros/ros.h>
#include <ros/param.h>

#include <geometry_msgs/PoseArray.h>
#include <apc16delft_msgs/SynthesizeGrasp.h>
#include <apc16delft_msgs/SynthesizeDeformable.h>
#include <apc16delft_util/managed_node.hpp>

#include "local_pose_processor.hpp"
#include "global_pose_processor.hpp"
#include "param.hpp"

namespace apc16delft {

using LocalPoseParams = struct LocalPoseParams;
using Weights         = struct Weights;
using Extremes        = struct Extremes;

class GraspSynthesizer : public ManagedNode {

private:
	/// Struct of Limits for LocalPoseProcessor.
	LocalPoseParams local_pose_params_;

	/// Struct of weights for CostFunction.
	Weights weights_;

	/// Struct of Limits for CostFunction.
	Extremes extremes_;

	/// Service server for synthesizing grasps.
	ros::ServiceServer synthesize_grasp_;

	/// Service server for synthesizing grasps on deformable objects.
	ros::ServiceServer synthesize_deformable_;

	/// PointCloud publisher.
	ros::Publisher pcl_pub_;

	/// Publisher for visualising vacuum candidates.
	ros::Publisher vacuum_visualiser_;

	/// Publisher for visualising pinch candidates.
	ros::Publisher pinch_visualiser_;

	/// List of all grasp items.
	std::vector<GraspItem> item_list_;

	/// PoseArrays for visualisation.
	geometry_msgs::PoseArray pinch_pose_array_;
	geometry_msgs::PoseArray vacuum_pose_array_;

	/// Vector of all bin dimensions.
	std::vector<Eigen::Vector3d> bin_dimensions_;

	/// Tote dimensions
	Eigen::Vector3d tote_dimensions_;

	/// Total number of items.
	int number_of_items_ = 39;

	/// Determines whether we want to visualise our candidates or not.
	bool visualise_ = true;

	double deformable_normal_radius_ = 0.01;
	double deformable_leaf_size_     = 0.01;

protected:
	/// Callback for synthesize grasp service.
	bool synthesizeGrasp(apc16delft_msgs::SynthesizeGrasp::Request & req, apc16delft_msgs::SynthesizeGrasp::Response & res);

	/// Callback for synthesize deformable service.
	bool synthesizeDeformable(apc16delft_msgs::SynthesizeDeformable::Request & req, apc16delft_msgs::SynthesizeDeformable::Response & res);

	/// Apply boost visitors to obtain the grasp poses.
	std::vector<Candidates> applyVisitors(apc16delft_msgs::SynthesizeGrasp::Request & req);

	virtual int onConfigure() override;
	virtual int onActivate() override;

public:
	GraspSynthesizer() {};

	/// Loads parameters from parameter server.
	bool loadParams();
};

}
