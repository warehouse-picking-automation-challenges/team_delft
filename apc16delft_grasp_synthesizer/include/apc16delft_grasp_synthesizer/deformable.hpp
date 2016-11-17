#pragma once

#include <apc16delft_util/util.hpp>

namespace apc16delft {

class Deformable {
public:
	/// Vector of all pinch grasp candidates.
	std::vector<apc16delft_msgs::GraspCandidate> grasp_candidates;

	/// Threshold to check for intersections with other shapes.
	/// For deformable items just set this really low to not eliminate other shapes.
	double threshold;

	/// Origin of this shape.
	Eigen::Isometry3d origin;

	/// Determines whether we use vacuum for this item or not.
	bool vacuum;

	/// Calculates distance of a point to this shape.
	double distanceTo(const Eigen::Vector3d & point) const {
		return point.norm();
	}

	/// Default constructor.
	Deformable() { origin = Eigen::Isometry3d::Identity(); vacuum = true; threshold = 1e-10;};

};

}
