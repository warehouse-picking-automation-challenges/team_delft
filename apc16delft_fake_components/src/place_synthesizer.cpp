#include <apc16delft_util/managed_node.hpp>
#include <apc16delft_msgs/SynthesizePlacement.h>

namespace apc16delft {

class PlaceSynthesizerNode : public ManagedNode {
protected:


	/// Read necessary parameters from the param server.
	int onConfigure() override {
		return 0;
	}

	int onActivate() override {
		// service servers
		placement_server = node_handle.advertiseService("synthesize_place", &PlaceSynthesizerNode::synthesizePlace, this);
		return 0;
	}

	int onDeactivate() override {
		// deactivate services
		placement_server.shutdown();
		return 0;
	}

	bool synthesizePlace(apc16delft_msgs::SynthesizePlacement::Request & req, apc16delft_msgs::SynthesizePlacement::Response & res) {
		apc16delft_msgs::PlacementCandidate place_candidate;
	
		/* create fixed fake placement candidate pose: 20cm above the tote base center */
		// NOW: the pose is refered to the world frame
		// TODO: USE TF!!! the reference frame for this pose should be the center of the tote base, the 'tote' frame is expected to be there
		//       request that the PlacementCandidate be a pose stamped to have a reference frame
		place_candidate.score              = 1.0;
		place_candidate.pose.position.x    = 0.669197262866;
		place_candidate.pose.position.y    = -0.42478218777;
		place_candidate.pose.position.z    = 0.840426003826;
		place_candidate.pose.orientation.x = -0.735385807296;
		place_candidate.pose.orientation.y = 0.67759509458;
		place_candidate.pose.orientation.z = -0.00804032796635;
		place_candidate.pose.orientation.w = 0.00282052396264;
		/**********************************************/
	
		res.error = 0;
		res.message = "fake candidate";
		res.candidates.push_back(place_candidate);
		return true;
	}

	/// Service server for estimating the pose of an object.
	ros::ServiceServer placement_server;

};

}

int main(int argc, char ** argv) {
	ROS_INFO("ROS NODE place_synthesizer started");
	ros::init(argc, argv, "place_synthesizer");
	apc16delft::PlaceSynthesizerNode place_synthesizer;
	ros::spin();
	return 0;
}
