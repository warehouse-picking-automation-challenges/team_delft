#pragma once

#include "circle.hpp"
#include <apc16delft_util/util.hpp>

namespace apc16delft {

class Cylinder {
private:
	/// Generate grasp points.
	void generateGraspPoints(double distance, double edge_clearing) {
		Eigen::Isometry3d circle_origin = Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0} * Eigen::Translation3d{0, 0, -0.5*height + distance};

		int number_of_steps = std::floor((height - edge_clearing) / distance);

		// Generate circle in center.
		if (number_of_steps > 0) {
			Circle circle(radius, distance, 10.0, -1, vacuum, true, Eigen::Isometry3d::Identity());
			grasp_candidates.insert(std::end(grasp_candidates), std::begin(circle.grasp_candidates), std::end(circle.grasp_candidates));
		}

		for (int k = 0; k < number_of_steps; k++) {
			Circle circle(radius, distance, 10.0, -1, vacuum, true, circle_origin);

			grasp_candidates.insert(std::end(grasp_candidates), std::begin(circle.grasp_candidates), std::end(circle.grasp_candidates));
			circle_origin.translate(Eigen::Vector3d{0.0, 0.0, distance});
		}
	};

public:
	/// Vector of grasp_candidates.
	std::vector<apc16delft_msgs::GraspCandidate> grasp_candidates;

	/// Radius of the cylinder.
	double radius;

	/// Height of the cylinder.
	double height;

	/// Threshold to check for intersections with other shapes.
	double threshold;

	/// Determines wheter we are using vacuum or pinch.
	bool vacuum;

	/// Position and orientation of the origin in object frame.
	Eigen::Isometry3d origin;

	/// Calculates distance of a point to this shape.
	double distanceTo(const Eigen::Vector3d & point) const {

		CylindricalCoordinates cylinder_coordinates = cart2cylinder(point);

		if(std::abs(cylinder_coordinates.z) > height/2.0) {
			if(cylinder_coordinates.rho > radius) {
				Eigen::Vector2d cylinder_corner{radius, height/2.0};
				Eigen::Vector2d point_cylindric{cylinder_coordinates.rho, std::abs(cylinder_coordinates.z)};
				return (point_cylindric - cylinder_corner).norm();
			} else {
				return std::abs(cylinder_coordinates.z - height/2.0);
			}
		} else {
			return cylinder_coordinates.rho - radius;
		}
	}

	/// Default constructor.
	Cylinder(double radius, double height, double distance, double threshold, double edge_clearing, bool vacuum, bool generate, const Eigen::Isometry3d & origin) :
		radius{radius},
		height{height},
		threshold{threshold},
		vacuum{vacuum},
		origin{origin}
	{
		if (generate) {
			generateGraspPoints(distance, edge_clearing);
		}

	};
};

}
