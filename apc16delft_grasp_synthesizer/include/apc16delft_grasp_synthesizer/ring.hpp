#pragma once

#include "circle.hpp"
#include <apc16delft_util/util.hpp>

namespace apc16delft {

class Ring {
private:
	/// Generate grasp points.
	void generateGraspPoints(double distance, double edge_clearing) {
		Eigen::Isometry3d circle_origin = Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0} * Eigen::Translation3d{0, 0, -0.5*height + distance};

		int number_of_steps = std::floor((height - edge_clearing) / distance);

		// Generate circle in center.
		if (number_of_steps > 0) {
			Circle circle(outer_radius, distance, 10.0, -1, vacuum, true, Eigen::Isometry3d::Identity());
			grasp_candidates.insert(std::end(grasp_candidates), std::begin(circle.grasp_candidates), std::end(circle.grasp_candidates));
		}

		for (int k = 0; k < number_of_steps; k++) {
			Circle outer_circle(outer_radius, distance, 10.0, -1, vacuum, true, circle_origin);

			grasp_candidates.insert(std::end(grasp_candidates), std::begin(outer_circle.grasp_candidates), std::end(outer_circle.grasp_candidates));
			circle_origin.translate(Eigen::Vector3d{0.0, 0.0, distance});
		}

		double ring_width = outer_radius - inner_radius;
		double mid_radius = inner_radius + 0.5*ring_width;

		Eigen::Isometry3d top_circle_origin    = Eigen::Translation3d{0, 0, 0.5*height} * Eigen::Quaterniond{0.0, 1.0, 0.0, 0.0};
		Eigen::Isometry3d bottom_circle_origin = Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0} * Eigen::Translation3d{0, 0, -0.5*height};

		Circle top_mid_circle(mid_radius, distance, 10.0, 0, vacuum, true, top_circle_origin);
		Circle bottom_mid_circle(mid_radius, distance, 10.0, 0, vacuum, true, bottom_circle_origin);

		grasp_candidates.insert(std::end(grasp_candidates), std::begin(top_mid_circle.grasp_candidates), std::end(top_mid_circle.grasp_candidates));
		grasp_candidates.insert(std::end(grasp_candidates), std::begin(bottom_mid_circle.grasp_candidates), std::end(bottom_mid_circle.grasp_candidates));
	};

public:
	/// Vector of grasp_candidates.
	std::vector<apc16delft_msgs::GraspCandidate> grasp_candidates;

	/// Radius of the inner circle.
	double inner_radius;

	/// Radius of the outer circle.
	double outer_radius;

	/// Height of the ring.
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
		Eigen::Vector2d point_cylindric{cylinder_coordinates.rho, std::abs(cylinder_coordinates.z)};
		Eigen::Vector2d ring_outer_corner{outer_radius, height/2.0};
		Eigen::Vector2d ring_inner_corner{inner_radius, height/2.0};

		if(std::abs(cylinder_coordinates.z) > height/2.0) {
			if(cylinder_coordinates.rho > outer_radius) {
				return (point_cylindric - ring_outer_corner).norm();
			} else {
				if (cylinder_coordinates.rho > inner_radius) {
					return std::abs(cylinder_coordinates.z - height/2.0);
				} else {
					return (point_cylindric - ring_inner_corner).norm();
				}
			}
		} else {
			if(cylinder_coordinates.rho > outer_radius) {
				return cylinder_coordinates.rho - outer_radius;
			} else {
				return std::min(std::abs(cylinder_coordinates.rho - inner_radius), outer_radius - cylinder_coordinates.rho);
			}
		}
	}

	/// Default constructor.
	Ring(double inner_radius, double outer_radius, double height, double distance, double threshold, double edge_clearing, bool vacuum, bool generate, const Eigen::Isometry3d & origin) :
		inner_radius{inner_radius},
		outer_radius{outer_radius},
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
