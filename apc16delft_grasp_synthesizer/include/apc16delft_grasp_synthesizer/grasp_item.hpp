#pragma once

#include "bar.hpp"
#include "cone.hpp"
#include "sphere.hpp"
#include "cylinder.hpp"
#include "ring.hpp"
#include "deformable.hpp"
#include "manual.hpp"

#include <boost/variant/variant.hpp>

namespace apc16delft {

using Shape = boost::variant<Bar, Cone, Sphere, Cylinder, Ring, Circle, Deformable, Manual>;

struct Candidates {
	Shape shape;
	bool vacuum;
	std::vector<apc16delft_msgs::GraspCandidate> grasp_candidates;

	Candidates(const Shape & shape) : shape{shape} {};

	Candidate(Candidates & other) {
		shape            = other.shape;
		vacuum           = other.vacuum;
		grasp_candidates = other.grasp_candidates;
	}
};

using Candidates = struct Candidates;

class GraspItem {
public:
	/// Pre grasp offset.
	double pre_grasp_offset;

	/// Center of Mass.
	Eigen::Vector3d center_of_mass;

	/// Path to the 3D model.
	std::string model_path;

	/// List of grasp sample spaces in object frame.
	std::vector<Shape> sample_spaces;
};

}
