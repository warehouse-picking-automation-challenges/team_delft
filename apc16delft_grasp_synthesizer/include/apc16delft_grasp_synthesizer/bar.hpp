#pragma once

#include <apc16delft_msgs/GraspCandidate.h>
#include <dr_eigen/eigen.hpp>
#include <dr_eigen/ros.hpp>

namespace apc16delft {

class Bar {
private:
	std::vector<apc16delft_msgs::GraspCandidate> generatePlane(const Eigen::VectorXd & x_coordinates, const Eigen::VectorXd & y_coordinates, const Eigen::Isometry3d & plane_origin) {
		Eigen::Isometry3d grasp_pose;

		std::vector<apc16delft_msgs::GraspCandidate> result;
		for (int i = 0; i < x_coordinates.size(); i++) {
			bool is_x_edge = x_coordinates.size() > 1 && (i == 0 || i == (x_coordinates.size() - 1));
			for (int j = 0; j < y_coordinates.size(); j++) {
				bool is_y_edge = y_coordinates.size() > 1 && (j == 0 || j == (y_coordinates.size() - 1));
				if(is_x_edge && is_y_edge)
					continue;
				Eigen::Isometry3d grasp_pose = plane_origin * Eigen::Translation3d{x_coordinates[i], y_coordinates[j], 0.0};
				apc16delft_msgs::GraspCandidate candidate;
				candidate.stamped_pose.pose = dr::toRosPose(grasp_pose);
				result.push_back(candidate);
			}
		}

		return result;
	}

	/// Generate grasp points.
	void generateGraspPoints(const Eigen::VectorXd & x_coordinates, const Eigen::VectorXd & y_coordinates, const Eigen::VectorXd & z_coordinates) {
		Eigen::Isometry3d bottom_origin = origin * Eigen::Translation3d{0.0, 0.0, -0.5*dimensions.z()};
		Eigen::Isometry3d top_origin    = origin * Eigen::Translation3d{0.0, 0.0,  0.5*dimensions.z()} * dr::rotateX(M_PI);
		Eigen::Isometry3d right_origin  = origin * Eigen::Translation3d{0.5*dimensions.x(), 0.0, 0.0 } * dr::rotateX(M_PI_2) * dr::rotateY(-M_PI_2);
		Eigen::Isometry3d left_origin   = origin * Eigen::Translation3d{-0.5*dimensions.x(), 0.0, 0.0} * dr::rotateX(M_PI_2) * dr::rotateY(M_PI_2);
		Eigen::Isometry3d front_origin  = origin * Eigen::Translation3d{0.0, -0.5*dimensions.y(), 0.0} * dr::rotateX(-M_PI_2);
		Eigen::Isometry3d back_origin   = origin * Eigen::Translation3d{0.0, 0.5*dimensions.y(), 0.0 } * dr::rotateX(M_PI_2);

		std::vector<apc16delft_msgs::GraspCandidate> top_plane    = generatePlane(x_coordinates, y_coordinates, top_origin);
		std::vector<apc16delft_msgs::GraspCandidate> bottom_plane = generatePlane(x_coordinates, y_coordinates, bottom_origin);
		std::vector<apc16delft_msgs::GraspCandidate> left_plane   = generatePlane(y_coordinates, z_coordinates, left_origin);
		std::vector<apc16delft_msgs::GraspCandidate> right_plane  = generatePlane(y_coordinates, z_coordinates, right_origin);
		std::vector<apc16delft_msgs::GraspCandidate> front_plane  = generatePlane(x_coordinates, z_coordinates, front_origin);
		std::vector<apc16delft_msgs::GraspCandidate> back_plane   = generatePlane(x_coordinates, z_coordinates, back_origin);

		grasp_candidates.insert(grasp_candidates.end(), top_plane.begin()   , top_plane.end()   );
		grasp_candidates.insert(grasp_candidates.end(), bottom_plane.begin(), bottom_plane.end());
		grasp_candidates.insert(grasp_candidates.end(), left_plane.begin()  , left_plane.end()  );
		grasp_candidates.insert(grasp_candidates.end(), right_plane.begin() , right_plane.end() );
		grasp_candidates.insert(grasp_candidates.end(), front_plane.begin() , front_plane.end() );
		grasp_candidates.insert(grasp_candidates.end(), back_plane.begin(), back_plane.end());
	}

public:
	/// Vector of grasp_candidates.
	std::vector<apc16delft_msgs::GraspCandidate> grasp_candidates;

	/// Dimensions of the bar.
	Eigen::Vector3d dimensions;

	/// Origin of this shape.
	Eigen::Isometry3d origin;

	/// Determines whether we are using vacuum or pinch.
	bool vacuum;

	/// Threshold to check for intersections with other shapes.
	double threshold;

	/// Calculates distance of a point to this shape.
	double distanceTo(const Eigen::Vector3d & point) const {
		double dx = std::max(-0.5*dimensions.x() - point.x(), std::max(0.0, point.x() - 0.5*dimensions.x()));
		double dy = std::max(-0.5*dimensions.y() - point.y(), std::max(0.0, point.y() - 0.5*dimensions.y()));
		double dz = std::max(-0.5*dimensions.z() - point.z(), std::max(0.0, point.z() - 0.5*dimensions.z()));

		return sqrt(dx*dx + dy*dy + dz*dz);
	}

	/// Default constructor.
	Bar(const Eigen::Vector3d & dimensions, const Eigen::Isometry3d & origin, double distance, double threshold, double edge_clearing, bool vacuum, bool generate) :
		dimensions{dimensions},
		origin{origin},
		vacuum{vacuum},
		threshold{threshold}
	{
		if (generate) {
			int number_of_points_x = std::floor((dimensions.x() - 2*edge_clearing) / distance);
			int number_of_points_y = std::floor((dimensions.y() - 2*edge_clearing) / distance);
			int number_of_points_z = std::floor((dimensions.z() - 2*edge_clearing) / distance);
			Eigen::VectorXd x_coordinates, y_coordinates, z_coordinates;

			if (number_of_points_x > 0) {
				x_coordinates = (number_of_points_x >= 3)
					? (Eigen::VectorXd) Eigen::VectorXd::LinSpaced(number_of_points_x - (number_of_points_x % 2 - 1), -dimensions.x() * 0.5 + edge_clearing, dimensions.x() * 0.5 - edge_clearing)
					: (Eigen::VectorXd) Eigen::VectorXd::Zero(1);
					;
			}

			if (number_of_points_y > 0) {
				y_coordinates = (number_of_points_y >= 3)
					? (Eigen::VectorXd) Eigen::VectorXd::LinSpaced(number_of_points_y - (number_of_points_y % 2 - 1), -dimensions.y() * 0.5 + edge_clearing, dimensions.y() * 0.5 - edge_clearing)
					: (Eigen::VectorXd) Eigen::VectorXd::Zero(1);
					;
			}

			if (number_of_points_z > 0) {
				z_coordinates = (number_of_points_z >= 3)
					? (Eigen::VectorXd) Eigen::VectorXd::LinSpaced(number_of_points_z - (number_of_points_z % 2 - 1), -dimensions.z() * 0.5 + edge_clearing, dimensions.z() * 0.5 - edge_clearing)
					: (Eigen::VectorXd) Eigen::VectorXd::Zero(1);
					;
			}

			generateGraspPoints(x_coordinates, y_coordinates, z_coordinates);
		}

	};
};

}

