#pragma once

#include <apc16delft_msgs/GraspCandidate.h>
#include <dr_eigen/eigen.hpp>
#include <dr_eigen/ros.hpp>

namespace apc16delft {

class Sphere {
private:
	/// Generate grasp points.
	void generateGraspPoints(double angular_distance) {
		int number_of_points = std::floor((2*M_PI) / angular_distance);

		apc16delft_msgs::GraspCandidate candidate;
		for (int i = 0; i < number_of_points; i++) {
			for (int j = 0; j < number_of_points; j++) {
				candidate.stamped_pose.pose = dr::toRosPose(origin * dr::rotateX(angular_distance* j) * dr::rotateZ(angular_distance*i) * Eigen::Translation3d{radius, 0.0, 0.0} * dr::rotateY(-0.5*M_PI));
				grasp_candidates.push_back(candidate);
			}
		}
	};

public:
	/// Calculates distance of a point to this shape.
	double distanceTo(const Eigen::Vector3d & point) const {
		return point.norm() - radius;
	}

	/// Vector of all pinch grasp candidates.
	std::vector<apc16delft_msgs::GraspCandidate> grasp_candidates;

	/// Threshold to check for intersections with other shapes.
	double threshold;

	/// Radius of the sphere.
	double radius;

	/// Radius of the sphere.
	bool vacuum;

	/// Pose of the origin in object frame.
	Eigen::Isometry3d origin;

	/// Default constructor.
	Sphere(double radius, double distance, double threshold, bool vacuum, bool generate, const Eigen::Isometry3d & origin) :
		threshold{threshold},
		radius{radius},
		vacuum{vacuum},
		origin{origin}
	{
		if (generate) {
			double angular_distance = distance / (2*M_PI*radius);
			generateGraspPoints(angular_distance);
		}
	};
};

}
