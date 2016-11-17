#include <apc16delft_util/managed_node.hpp>
#include <apc16delft_msgs/EstimateObjectPose.h>

#include <ros/init.h>

namespace apc16delft {

class PoseEstimationNode : public ManagedNode {
using PoseEstReq     = apc16delft_msgs::EstimateObjectPose::Request;
using PoseEstRes     = apc16delft_msgs::EstimateObjectPose::Response;

protected:

	/// Read necessary parameters from the param server.
	int onConfigure() override {
		return 0;
	}

	int onActivate() override {
		// service servers
		estimate_pose_server = node_handle.advertiseService("estimate_pose", &PoseEstimationNode::estimatePose, this);
		return 0;
	}

	int onDeactivate() override {
		// deactivate services
		estimate_pose_server.shutdown();
		return 0;
	}
	
	/// Service call for estimating the pose of an object.
	bool estimatePose(PoseEstReq & req, PoseEstRes & res) {
		// copy transform to result
		res.pose.header.frame_id    = req.object.point_cloud.header.frame_id;
		res.pose.pose.position.x    = 0;
		res.pose.pose.position.y    = 0;
		res.pose.pose.position.z    = 0;
		res.pose.pose.orientation.x = 0;
		res.pose.pose.orientation.y = 0;
		res.pose.pose.orientation.z = 0;
		res.pose.pose.orientation.w = 1;
		res.error.code              = 0;
		res.error.message           = "";

		return true;
	}

	/// Service server for estimating the pose of an object.
	ros::ServiceServer estimate_pose_server;

	/// Publisher for viewing the current particles in the particle filter tracker.
	ros::Publisher particle_publisher;

	/// Publisher for viewing the transformed model after particle filter pose estimation.
	ros::Publisher transformed_model_pf_publisher;

	/// Publisher for viewing the transformed model after the ICP pose estimation refinement.
	ros::Publisher transformed_model_icp_publisher;

	/// Publisher for viewing the original input point cloud.
	ros::Publisher object_cloud_publisher;
};

}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "pose_estimation");
	apc16delft::PoseEstimationNode node;
	ros::spin();
}

