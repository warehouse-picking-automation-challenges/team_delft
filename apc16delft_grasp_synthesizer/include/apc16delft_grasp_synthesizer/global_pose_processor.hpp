#pragma once

#include <boost/variant.hpp>
#include <dr_eigen/ros.hpp>
#include <dr_eigen/yaml.hpp>

#include <apc16delft_util/util.hpp>

#include "grasp_item.hpp"

namespace apc16delft {

/// Visitor to return the correct candidates between sample spaces.
class GlobalPoseProcessor : public boost::static_visitor<void> {
private:
	Candidates *candidates_;
	Eigen::Isometry3d object_pose_;
	std::string frame_id;

	template<typename S>
	std::vector<apc16delft_msgs::GraspCandidate> comparePoints(const S & s, const std::vector<apc16delft_msgs::GraspCandidate> & grasp_candidates) {
		std::vector<apc16delft_msgs::GraspCandidate> new_grasp_candidates;
		for (const apc16delft_msgs::GraspCandidate & candidate : grasp_candidates) {
			const Eigen::Vector3d position = s.origin.inverse() * object_pose_.inverse() * dr::toEigen(candidate.stamped_pose.pose).translation();
			if (s.distanceTo(position) > s.threshold) {
				new_grasp_candidates.push_back(candidate);
			}
		}

		return new_grasp_candidates;
	}

	template<typename S>
	void eliminateByDistance(const S & s) {
		std::vector<apc16delft_msgs::GraspCandidate> new_candidates;

		candidates_->grasp_candidates = comparePoints(s, candidates_->grasp_candidates);
	}

public:
	GlobalPoseProcessor(Candidates* candidates, const Eigen::Isometry3d & object_pose) :
	candidates_{candidates},
	object_pose_{object_pose}
	{};

	void operator()(const Cone & cone) {
		eliminateByDistance<Cone>(cone);
	}

	void operator()(const Sphere & sphere) {
		eliminateByDistance<Sphere>(sphere);
	}

	void operator()(const Bar & bar) {
		eliminateByDistance<Bar>(bar);
	}

	void operator()(const Cylinder & cylinder) {
		eliminateByDistance<Cylinder>(cylinder);
	}

	void operator()(const Deformable & deformable) {
		eliminateByDistance<Deformable>(deformable);
	}

	void operator()(const Ring & ring) {
		eliminateByDistance<Ring>(ring);
	}

	void operator()(const Circle & circle) {
		eliminateByDistance<Circle>(circle);
	}

	void operator()(const Manual & manual) {
		eliminateByDistance<Manual>(manual);
	}

};

}
