
#include "place_synthesizer.hpp"
#include <dr_param/param.hpp>
#include <dr_eigen/ros.hpp>

using namespace apc16delft;
using namespace placement;

PlaceSynthesizer::PlaceSynthesizer() : node_handle_("~") {

	length_x_  = dr::getParam<double>(node_handle_, "dimensions/x");
	length_y_  = dr::getParam<double>(node_handle_, "dimensions/y");
	volumes_x_ = dr::getParam<int>(node_handle_,    "volumes/x");
	volumes_y_ = dr::getParam<int>(node_handle_,    "volumes/y");

	double volume_dim_x = length_x_ / volumes_x_;
	double min_x = -length_x_/2.0 + volume_dim_x/2.0;
	double volume_dim_y = length_y_ / volumes_y_;
	double min_y = -length_y_/2.0 + volume_dim_y/2.0;
	for(int x = 0; x < volumes_x_; ++x) {
		for(int y = 0; y < volumes_y_; ++y) {
			double x_coord = min_x + x * volume_dim_x;
			double y_coord = min_y + y * volume_dim_y;
			Eigen::Isometry3d place = Eigen::Isometry3d::Identity() * Eigen::Translation3d(x_coord, y_coord, 0);
			places_.push_back(place);
		}
	}

	next_place_ = places_.begin();

	synthesize_placement_ = node_handle_.advertiseService("synthesize_place", &PlaceSynthesizer::synthesizePlace, this);
};

bool PlaceSynthesizer::synthesizePlace(apc16delft_msgs::SynthesizePlacement::Request & req, apc16delft_msgs::SynthesizePlacement::Response & res) {
	apc16delft_msgs::PlacementCandidate place_candidate;

	Eigen::Isometry3d container = dr::toEigen(req.container);

	place_candidate.score = 1.0;
	place_candidate.pose = dr::toRosPose(container * (*next_place_));
	/**********************************************/

	res.error = 0;
	res.message = "hugely heuristic candidate";
	res.candidates.push_back(place_candidate);

	++next_place_;
	if(next_place_ == places_.end())
		next_place_ = places_.begin();

	return true;
}

int main(int argc, char ** argv) {
	ROS_INFO("ROS NODE place_synthesizer started");
	ros::init(argc, argv, "place_synthesizer");
	apc16delft::placement::PlaceSynthesizer place_synthesizer;
	ros::spin();
	return 0;
}
