#include <ros/ros.h>
#include <apc16delft_msgs/SynthesizePlacement.h>
#include <apc16delft_msgs/PlacementCandidate.h>
#include <apc16delft_util/managed_node.hpp>
#include <dr_eigen/eigen.hpp>

namespace apc16delft {
	namespace placement {

		class PlaceSynthesizer {
			private:
				ros::ServiceServer synthesize_placement_;
				ros::NodeHandle node_handle_;

				std::vector<Eigen::Isometry3d> places_;
				std::vector<Eigen::Isometry3d>::const_iterator next_place_;
				double length_x_;
				double length_y_;
				int volumes_x_;
				int volumes_y_;

			public:
				PlaceSynthesizer();

			protected:
				bool synthesizePlace(apc16delft_msgs::SynthesizePlacement::Request & req, apc16delft_msgs::SynthesizePlacement::Response & res);
		};

	} // namespace
}
