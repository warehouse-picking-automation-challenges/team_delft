#pragma once

#include "circle.hpp"
#include <apc16delft_util/util.hpp>

namespace apc16delft {

class Cone {
private:
	/// Generate grasp points.
	void generateGraspPoints(double distance) {
		Eigen::Isometry3d circle_origin = Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0} * Eigen::Translation3d{0, 0, -0.5*height};
		Eigen::Isometry3d peak          = Eigen::Translation3d{0, 0, 0.5*height} * dr::rotateY(M_PI);

		Circle circle(radius, distance, 10.0, -1, vacuum, true, circle_origin);
		grasp_candidates.insert(std::end(grasp_candidates), std::begin(circle.grasp_candidates), std::end(circle.grasp_candidates));

		apc16delft_msgs::GraspCandidate candidate;
		candidate.stamped_pose.pose = dr::toRosPose(peak);
		grasp_candidates.push_back(candidate);
	};

public:
	/// Vector of grasp_candidates.
	std::vector<apc16delft_msgs::GraspCandidate> grasp_candidates;

	/// Radius of the cone.
	double radius;

	/// Height of the cone.
	double height;

	/// Threshold to check for intersections with other shapes.
	double threshold;

	/// Determines wheter we are using vacuum or pinch.
	bool vacuum;

	/// Origin of this shape.
	Eigen::Isometry3d origin;

	/// Calculates distance of a point to this shape.
	double distanceTo(const Eigen::Vector3d & point) const {
		CylindricalCoordinates cylinder_coordinates = cart2cylinder(point);

		if (cylinder_coordinates.rho < (cylinder_coordinates.z / height) * (0.5*height - cylinder_coordinates.z) &&
			std::fabs(cylinder_coordinates.z) < 0.5*height) {
			return -1.0;
		} else {
			Eigen::Vector2d point_cylindric{cylinder_coordinates.rho, cylinder_coordinates.z};
			Eigen::Vector2d triangle_upper_corner{0.0, 0.5*height};
			Eigen::Vector2d triangle_lower_right_corner{radius, -0.5*height};
			Eigen::Vector2d triangle_lower_left_corner{0.0, -0.5*height};

			Eigen::ParametrizedLine<double, 2> lower_cathetus = Eigen::ParametrizedLine<double, 2>::Through(triangle_lower_left_corner, triangle_lower_right_corner);
			Eigen::ParametrizedLine<double, 2> hypothenuse    = Eigen::ParametrizedLine<double, 2>::Through(triangle_lower_right_corner, triangle_upper_corner);

			double distance_lower_right_corner     = (point_cylindric - triangle_lower_right_corner).norm();
			double distance_upper_corner           = (point_cylindric - triangle_upper_corner).norm();
			double distance_lower_cathetus         = lower_cathetus.distance(point_cylindric);
			double distance_hypothenuse            = hypothenuse.distance(point_cylindric);
			Eigen::Vector2d projection_hypothenuse = hypothenuse.projection(point_cylindric);

			if (cylinder_coordinates.z < -0.5*height && cylinder_coordinates.rho <= radius) {
				return distance_lower_cathetus;
			} else if (projection_hypothenuse.x() <= radius && projection_hypothenuse.x() >= 0.0) {
				return distance_hypothenuse;
			} else if (projection_hypothenuse.x() < 0) {
				return distance_upper_corner;
			} else {
				return distance_lower_right_corner;
			}
		}
	}

	/// Default constructor.
	Cone(double radius, double height, double distance, double threshold, bool vacuum, bool generate, const Eigen::Isometry3d & origin) :
		radius{radius},
		height{height},
		threshold{threshold},
		vacuum{vacuum},
		origin{origin}
	{
		if (generate) {
			generateGraspPoints(distance);
		}
	};
};

}
