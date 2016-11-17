#pragma once

#include <apc16delft_msgs/GraspCandidate.h>
#include <apc16delft_util/util.hpp>
#include <dr_eigen/eigen.hpp>
#include <dr_eigen/ros.hpp>

namespace apc16delft {

class Circle {
private:
	/// Generate grasp points.
	void generateGraspPoints(double angular_distance) {
		int points_per_circle = std::floor((2*M_PI) / angular_distance);

		Eigen::AngleAxisd rotation_y = dr::rotateY(direction * 0.5*M_PI) ;
		apc16delft_msgs::GraspCandidate candidate;
		for (int i = 0; i < points_per_circle; i++) {
			candidate.stamped_pose.pose = dr::toRosPose(origin * dr::rotateZ(angular_distance * i) * Eigen::Translation3d{radius, 0.0, 0.0} * rotation_y);

			if (strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_INWARDS    ||
				strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS) {
				candidate.strategy = strategy;
			}

			grasp_candidates.push_back(candidate);
		}
	}

public:
	/// Distance to this shape.
	double distanceTo(const Eigen::Vector3d & point) const {
		if (std::abs(point.z()) < 1e-5) {
			return point.norm() - radius;
		} else {
			CylindricalCoordinates cylinder_coordinates = cart2cylinder(point);
			Eigen::Vector2d point_cylindric{cylinder_coordinates.rho, std::abs(cylinder_coordinates.z)};
			Eigen::Vector2d circle_corner{radius, 0.0};

			return (point_cylindric - circle_corner).norm();
		}
	}

	/// Vector of grasp_candidates.
	std::vector<apc16delft_msgs::GraspCandidate> grasp_candidates;

	/// Radius of the circle.
	double radius;

	/// Approach direction of the circle. true = direction, false = parallel.
	int direction;

	/// Determines wheter we are using vacuum or pinch.
	bool vacuum;

	/// Threshold to check for intersections with other shapes.
	double threshold;

	/// Origin of this shape.
	Eigen::Isometry3d origin;

	/// The strategy for this shape.
	int strategy;

	/// Default constructor.
	Circle(double radius, double distance, double threshold, int direction, bool vacuum, bool generate, const Eigen::Isometry3d & origin, int strategy = 100) :
		radius{radius},
		direction{direction},
		vacuum{vacuum},
		threshold{threshold},
		origin{origin},
		strategy{strategy}
	{
		if (
			strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_INWARDS    ||
			strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS  ||
			strategy == apc16delft_msgs::GraspCandidate::PENCIL_CUP_INWARDS  ||
			strategy == apc16delft_msgs::GraspCandidate::PENCIL_CUP_SIDEWAYS ||
			strategy == apc16delft_msgs::GraspCandidate::PENCIL_CUP_STANDING
		    )
		{
			vacuum = false;
		}

		if (generate) {
			double angular_distance = distance / (2*M_PI*radius);
			generateGraspPoints(angular_distance);
		}
	}
};

}
