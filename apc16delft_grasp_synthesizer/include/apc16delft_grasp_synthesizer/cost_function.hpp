#pragma once

#include "local_pose_processor.hpp"
#include <apc16delft_util/util.hpp>

using Weights  = struct Weights;
using Extremes = struct Extremes;

namespace apc16delft {

struct Weights {
	double after_snap_weight;
	double before_snap_weight;
	double distance_weight;
	double length_weight;
	double knn_weight;

	Weights(const Weights & weights) {
		after_snap_weight  = weights.after_snap_weight;
		before_snap_weight = weights.before_snap_weight;
		distance_weight    = weights.distance_weight;
		length_weight      = weights.length_weight;
		knn_weight         = weights.knn_weight;
	}

	Weights() {
		after_snap_weight  = 0.25;
		before_snap_weight = 0.75;
		distance_weight    = 0.75;
		length_weight      = 0.25;
		knn_weight         = 1.25;
	}
};

struct Extremes {
	double distance_epsilon;
	double distance_tau;
	double length_epsilon;
	double length_tau;

	Extremes(const Extremes & extremes) {
		distance_epsilon  = extremes.distance_epsilon;
		distance_tau      = extremes.distance_tau;
		length_epsilon    = extremes.length_epsilon;
		length_tau        = extremes.length_tau;
	}

	Extremes() {
		distance_epsilon  = 5e-3;
		distance_tau      = 5e-2;
		length_tau        = 10e-2;
		length_epsilon    = 3e-2;
	}
};

class CostFunction {
friend class LocalPoseProcessor;
public:
	Extremes extremes;
	Weights weights;
	CostFunction(const Extremes & extremes, const Weights & weights) : extremes{extremes}, weights{weights} {}
	CostFunction(const CostFunction & other) { extremes = other.extremes; weights = other.weights; };

private:
	/// Converts distance between grasp z axis and CoM to a score.
	double distanceToScore(double distance) {
		return interpolateLinearly(distance, extremes.distance_epsilon, extremes.distance_tau);
	}

	/// Converts distance between grasp pose and CoM to a score.
	double pendulumLengthToScore(double pendulum_length) {
		return interpolateLinearly(pendulum_length, extremes.length_epsilon, extremes.length_tau);
	}

	double calculateScore(double knn_weight, const Eigen::Isometry3d & candidate, const Eigen::Vector3d & center_of_mass) {
		Eigen::Vector3d z_axis = candidate.rotation() * dr::axes::z();
		Eigen::ParametrizedLine<double, 3> line{candidate.translation(), z_axis};
		double distance_to_com = (candidate.translation() - center_of_mass).norm();

		return knn_weight * (weights.distance_weight * distanceToScore(line.distance(center_of_mass)) +
			   weights.length_weight * pendulumLengthToScore(distance_to_com));
	}

	double calculateDeformableScore(const Eigen::Isometry3d & candidate, const Eigen::Vector3d & center_of_mass) {
		double distance_to_com = (candidate.translation() - center_of_mass).norm();
		return distanceToScore(distance_to_com);
	}
};

};
