#include <ros/ros.h>
#include <apc16delft_util/managed_node.hpp>
#include <apc16delft_msgs/SynthesizeGrasp.h>
#include <apc16delft_util/managed_node.hpp>

namespace apc16delft {
namespace grasping {

class GraspSynthesizer : public ManagedNode {

	ros::ServiceServer synthesize_grasp;
	geometry_msgs::Pose pose;

	bool synthesizeGrasp(apc16delft_msgs::SynthesizeGrasp::Request & req, apc16delft_msgs::SynthesizeGrasp::Response & res) {
		apc16delft_msgs::GraspCandidate candidate;
		res.error       = 0;
		res.message     = "Fake result.";
		candidate.score = 1.0;
		candidate.stamped_pose.header.frame_id  = req.object_pose.header.frame_id;
		candidate.stamped_pose.pose             = pose;

		res.candidates.push_back(candidate);
		return true;
	}

	virtual int onConfigure() override {
		node_handle.getParam("grasp_position/x"   , pose.position.x   );
		node_handle.getParam("grasp_position/y"   , pose.position.y   );
		node_handle.getParam("grasp_position/z"   , pose.position.z   );
		node_handle.getParam("grasp_orientation/w", pose.orientation.w);
		node_handle.getParam("grasp_orientation/x", pose.orientation.x);
		node_handle.getParam("grasp_orientation/y", pose.orientation.y);
		node_handle.getParam("grasp_orientation/z", pose.orientation.z);

		return 0;
	}

	virtual int onActivate() override {
		ROS_INFO_STREAM("Activated!");
		synthesize_grasp = node_handle.advertiseService("synthesize_grasp", &GraspSynthesizer::synthesizeGrasp, this);
		return 0;
	}

	virtual int onDeactivate() override {
		synthesize_grasp.shutdown();
		return 0;
	}
};

} // namespace
}

int main(int argc, char * * argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME "_grasp_synthesizer");
	apc16delft::grasping::GraspSynthesizer node;
	node.configure();
	node.activate();
	ros::spin();
}

