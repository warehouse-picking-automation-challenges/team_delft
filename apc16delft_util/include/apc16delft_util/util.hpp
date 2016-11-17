#pragma once

#include <ros/package.h>

#include <apc16delft_msgs/Object.h>
#include <dr_eigen/eigen.hpp>
#include <dr_eigen/ros.hpp>
#include <dr_eigen/param.hpp>
#include <dr_param/param.hpp>
#include <iostream>
#include <cmath>

#include <boost/bind.hpp>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#include <apc16delft_msgs/objects.hpp>

namespace apc16delft {
// Lip order is clockwise starting from top, with back wall as last element.
constexpr uint8_t top_lip    = 0;
constexpr uint8_t right_lip  = 1;
constexpr uint8_t bottom_lip = 2;
constexpr uint8_t left_lip   = 3;
constexpr uint8_t back_lip   = 4;

enum class BinIndex {
	BIN_A = 0,
	BIN_B = 1,
	BIN_C = 2,
	BIN_D = 3,
	BIN_E = 4,
	BIN_F = 5,
	BIN_G = 6,
	BIN_H = 7,
	BIN_I = 8,
	BIN_J = 9,
	BIN_K = 10,
	BIN_L = 11,
	NUMBER_OF_BINS = 12,
};

bool isDeformable(uint8_t object_type) {
	switch (object_type) {
		case apc16delft_msgs::Object::WOMENS_KNIT_GLOVES:
		case apc16delft_msgs::Object::CHEROKEE_EASY_TEE_SHIRT:
		case apc16delft_msgs::Object::CLOUD_B_PLUSH_BEAR:
		case apc16delft_msgs::Object::COOL_SHOT_GLUE_STICKS:
		return true;

		default:
		return false;
	}
}

bool isBig(uint8_t object_type) {
	switch (object_type) {
		case apc16delft_msgs::Object::I_AM_A_BUNNY_BOOK:
		case apc16delft_msgs::Object::SCOTCH_BUBBLE_MAILER:
		return true;

		default:
		return false;
	}
}

bool isCornerBin(int bin_index) {
	if (
		bin_index == (int) BinIndex::BIN_A ||
		bin_index == (int) BinIndex::BIN_B ||
		bin_index == (int) BinIndex::BIN_C ||
		bin_index == (int) BinIndex::BIN_D) {
		return true;
	}

	return false;
}

bool loadToteDimensions(Eigen::Vector3d & tote_dimensions, const ros::NodeHandle & node_handle) {
	try {
		tote_dimensions = dr::getParam<Eigen::Vector3d>(node_handle, "/tote/dimensions");
	} catch (const std::exception & e) {
		ROS_ERROR_STREAM(e.what());
		return false;
	}

	return true;
}

bool loadBinDimensions(std::vector<Eigen::Vector3d> & bin_dimensions, const ros::NodeHandle & node_handle) {
	std::stringstream param;
	bin_dimensions.resize((int) BinIndex::NUMBER_OF_BINS);
	for (int i = 0; i < (int) BinIndex::NUMBER_OF_BINS; i++) {
		param.str(std::string());
		param << "/bin/" << i << "/dimensions";
		try {
			bin_dimensions.at(i) = dr::getParam<Eigen::Vector3d>(node_handle, param.str());
		} catch (const std::exception & e) {
			ROS_ERROR_STREAM(e.what());
			return false;
		}
	}

	return true;
}

/// Interpolates linearly between epsilon and tau.
double interpolateLinearly(double value, double epsilon, double tau) {
	if (value < epsilon) {
		return 1.0;
	} else if (value > tau) {
		return 0.0;
	} else {
		return 1.0 + (epsilon - value) / (tau - epsilon);
	}
}

/// Extract Isometry transformation from Affine transformation.
Eigen::Isometry3d affineToIsometry(const Eigen::Affine3d & transformation) {
	Eigen::Isometry3d result;
	result.translation() = transformation.translation();
	result.linear()      = transformation.rotation();

	return result;
}

/// Construct Affine3d from Isometry3d
Eigen::Affine3d isometryToAffine(const Eigen::Isometry3d & transformation) {
	return Eigen::Affine3d{transformation.matrix()};
}

std::array<Eigen::Hyperplane<double,3>, 5> getBinBoundaries(const Eigen::Isometry3d & bin_pose, const Eigen::Vector3d & bin_sizes) {
	Eigen::Isometry3d ceiling_pose = bin_pose * dr::translate(bin_sizes.x() * 0.5, bin_sizes.y() * 0.5, bin_sizes.z()) * dr::rotateY(M_PI);
	Eigen::Isometry3d bottom_pose  = bin_pose * dr::translate(bin_sizes.x() * 0.5, bin_sizes.y() * 0.5, 0);

	double rotation                          = M_PI_2;
	double x_translation                     = bin_sizes.x() * 0.5;
	Eigen::Isometry3d right_wall_pose        = bottom_pose * dr::translate(x_translation, 0, bin_sizes.z() * 0.5) * dr::rotateY(-rotation);
	Eigen::Isometry3d left_wall_pose         = bottom_pose * dr::translate(-x_translation, 0, bin_sizes.z() * 0.5) * dr::rotateY(rotation);
	Eigen::Isometry3d back_wall_pose         = bottom_pose * dr::translate(0, bin_sizes.y() * 0.5, bin_sizes.z() * 0.5) * dr::rotateX(rotation);
	Eigen::Hyperplane<double, 3> bin_bottom  = dr::makeXyPlane(bottom_pose);
	Eigen::Hyperplane<double, 3> bin_ceiling = dr::makeXyPlane(ceiling_pose);
	Eigen::Hyperplane<double, 3> right_wall  = dr::makeXyPlane(right_wall_pose);
	Eigen::Hyperplane<double, 3> left_wall   = dr::makeXyPlane(left_wall_pose);
	Eigen::Hyperplane<double, 3> back_wall   = dr::makeXyPlane(back_wall_pose);

	// Lip order is clockwise starting from top, with back wall as last element.
	return std::array<Eigen::Hyperplane<double, 3>, 5>{{bin_ceiling, right_wall, bin_bottom, left_wall, back_wall}};
}

Eigen::Vector3d getBinCenter(const Eigen::Isometry3d & bin_pose, const Eigen::Vector3d & bin_sizes) {
	Eigen::Isometry3d bin_center_pose = bin_pose * dr::translate(bin_sizes.x() * 0.5, bin_sizes.y() * 0.5, bin_sizes.z() * 0.5);
	return bin_center_pose.translation();
}

bool withinBoundaries(const Eigen::Isometry3d & grasp_pose, const std::array<Eigen::Hyperplane<double, 3>, 5> & boundaries) {
	for (const auto & lip : boundaries) {
		if(lip.signedDistance(grasp_pose.translation()) < 0) {
			return false;
		}
	}
	return true;
}

bool withinBoundaries(const Eigen::Isometry3d & grasp_pose, const Eigen::Isometry3d & bin_pose, const Eigen::Vector3d & bin_sizes) {
	std::array<Eigen::Hyperplane<double, 3>, 5> boundaries = getBinBoundaries(bin_pose, bin_sizes);
	return withinBoundaries(grasp_pose, boundaries);
}

// TODO: Should never make vector of stamped poses, should use PoseArray.
std::vector<geometry_msgs::PoseStamped> eigen2Geometry(const std::vector<Eigen::Isometry3d> & poses, std::string frame_id) {
	std::vector<geometry_msgs::PoseStamped> result;
	result.reserve(poses.size());
	for (size_t i = 0; i < poses.size(); i++) {
		geometry_msgs::PoseStamped waypoint;
		waypoint.header.frame_id = frame_id;
		waypoint.header.stamp    = ros::Time::now();
		waypoint.pose            = dr::toRosPose(poses.at(i));
		result.push_back(waypoint);
	}

	return result;
}

std::vector<geometry_msgs::Pose> eigenPosesToRos(const std::vector<Eigen::Isometry3d> & poses) {
	std::vector<geometry_msgs::Pose> result;
	result.reserve(poses.size());
	for (auto const & pose : poses) {
		result.push_back(dr::toRosPose(pose));
	}
	return result;
}

/// Clip a value between min and max.
double clipValue(double value, double min, double max) {
	if (value > max) return max;
	if (value < min) return min;
	return value;
}

/// Round double to specified number of decimals.
double setSignificance(double value, int significance) {
	return std::roundf((value * pow(10, significance))) / pow(10, significance);
}

/// Returns the angle between two vectors.
double angularDistance(const Eigen::Vector3d & a, const Eigen::Vector3d & b, int significance = 6) {
	double dot_product = a.dot(b);
	return setSignificance(acos((clipValue(dot_product / (a.norm() * b.norm()), -1.0, 1.0))), significance);
}

/// Check whether two vectors are facing in the same direction.
bool sameDirection(const Eigen::Vector3d & a, const Eigen::Vector3d & b, double epsilon = -1e-6) {
	return a.dot(b) >= -epsilon;
}

/// Checks whether a vector is facing to the front.
/// Front is within 0.5*pi of positive y axis.
bool facingFront(const Eigen::Vector3d & direction, const Eigen::Isometry3d & bin_pose = Eigen::Isometry3d::Identity(), double epsilon = -1e-3) {
	return sameDirection(bin_pose.rotation() * dr::axes::y(), direction, epsilon);
}

/// Checks whether a vector is facing to the back.
/// Back is within 0.5*pi of negative y axis.
bool facingBack(const Eigen::Vector3d & direction, const Eigen::Isometry3d & bin_pose = Eigen::Isometry3d::Identity(),double epsilon = -1e-3) {
	return sameDirection(bin_pose.rotation() * -dr::axes::y(), direction, epsilon);
}

/// Checks whether a vector is facing down.
/// Down is within 0.5*pi of negative z axis.
bool facingDown(const Eigen::Vector3d & direction, const Eigen::Isometry3d & bin_pose = Eigen::Isometry3d::Identity(),double epsilon = -1e-3) {
	return sameDirection(bin_pose.rotation() * -dr::axes::z(), direction, epsilon);
}

/// Checks whether a vector is facing up.
/// Up is within 0.5*pi of positive z axis.
bool facingUp(const Eigen::Vector3d & direction, const Eigen::Isometry3d & bin_pose = Eigen::Isometry3d::Identity(),double epsilon = -1e-3) {
	return sameDirection(bin_pose.rotation() * dr::axes::z(), direction, epsilon);
}

/// Checks whether a vector is facing right.
/// Right is within 0.5*pi of positive x axis.
bool facingRight(const Eigen::Vector3d & direction, const Eigen::Isometry3d & bin_pose = Eigen::Isometry3d::Identity(),double epsilon = -1e-3) {
	return sameDirection(bin_pose.rotation() * dr::axes::x(), direction, epsilon);
}

/// Checks whether a vector is facing left.
/// Left is within 0.5*pi of negative x axis.
bool facingLeft(const Eigen::Vector3d & direction, const Eigen::Isometry3d & bin_pose = Eigen::Isometry3d::Identity(),double epsilon = -1e-3) {
	return sameDirection(bin_pose.rotation() * -dr::axes::x(), direction, epsilon);
}

/// Conversion function from radians to degrees.
double rad2deg(double radians) {
	return ((radians * 180.0) / M_PI);
}

/// Conversion function from degrees to radians.
double deg2rad(double degrees) {
	return ((degrees * M_PI) / 180.0);
}

/// Struct holding cylindrical coordinate parameters.
struct CylindricalCoordinates {
	double z, rho, phi;
	CylindricalCoordinates(double z, double rho, double phi) : z{z}, rho{rho}, phi{phi} {}
};

using CylindricalCoordinates = struct CylindricalCoordinates;

/// Converts a 3d Cartesian coordinate to cylindrical coordinates.
CylindricalCoordinates cart2cylinder(const Eigen::Vector3d & point) {
	double point_z = point.z();
	Eigen::Vector3d point_xy = point;
	point_xy.z() = 0;
	//point_xy = point_xy.normalized();

	double point_rho = point_xy.norm();
	double point_phi = std::atan2(point_xy.y(), point_xy.x());

	return CylindricalCoordinates{point_z, point_rho, point_phi};
}

/// Find model path from url.
/// Code from http://docs.ros.org/jade/api/resource_retriever/html/retriever_8cpp_source.html
std::string resolvePackagePath(const std::string & url) {
	std::string mod_url = url;
	if (url.find("package://") == 0) {
		mod_url.erase(0, strlen("package://"));
		size_t pos = mod_url.find("/");
		if (pos == std::string::npos) {
			std::cout << "Could not parse package:// format into file:// format\n";
			return url;
		}

		std::string package = mod_url.substr(0, pos);
		mod_url.erase(0, pos);
		std::string package_path = ros::package::getPath(package);

		if (package_path.empty()) {
			std::cout << "Package [" + package + "] does not exist \n";
			return url;
		}

		mod_url = package_path + mod_url;
		return mod_url;
	}

	//TODO: Error handling
	return url;
}

template <typename PointCloud>
bool loadObjectModels(std::vector<typename PointCloud::Ptr> & object_models, ros::NodeHandle const & node_handle) {
	object_models.emplace_back(new PointCloud);

	for (int i = 1; i < apc16delft_msgs::Object::MAX_OBJECT; i++) {
		if (isDeformable(static_cast<uint8_t>(i))) {
			object_models.emplace_back(new PointCloud);
			continue;
		}

		std::string model_path;
		if (!node_handle.getParam("/item/" + apc16delft_msgs::objectTypeToString(i) + "/model_path", model_path)) {
			ROS_WARN_STREAM("Failed to read model path for object: " << apc16delft_msgs::objectTypeToString(i));
			return false;
		}

		model_path = apc16delft::resolvePackagePath(model_path);

		if (!boost::filesystem::exists(model_path)) {
			ROS_WARN_STREAM("Model file for item " << apc16delft_msgs::objectTypeToString(i) << " does not exist. Path: " << model_path);
			return false;
		}

		typename PointCloud::Ptr model(new PointCloud);
		if (pcl::io::loadPCDFile(model_path, *model) == -1) {
			ROS_WARN_STREAM("No model found. Model path: " << model_path);
			return false;
		}

		if (model->empty()) {
			ROS_ERROR_STREAM("Model point cloud is empty.");
			return false;
		}

		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*model, *model, indices);

		ROS_INFO_STREAM("Adding model for " << apc16delft_msgs::objectTypeToString(i) << " to cache.");
		object_models.push_back(model);
	}

	return true;
}

template <typename PointCloud>
bool loadBinModels(std::vector<typename PointCloud::Ptr> & bin_models, ros::NodeHandle const & node_handle) {
	for (int i = int(BinIndex::BIN_A); i <= int(BinIndex::BIN_L); i++) {
		std::string model_path;
		if (!node_handle.getParam("/bin/" + std::to_string(i) + "/model_path", model_path)) {
			ROS_ERROR_STREAM("Failed to read model path for bin: " << i);
			return false;
		}

		model_path = resolvePackagePath(model_path);

		if (!boost::filesystem::exists(model_path)) {
			ROS_ERROR_STREAM("Model file for bin " << i << " does not exist. Path: " << model_path);
			return false;
		}

		typename PointCloud::Ptr model(new PointCloud);
		if (pcl::io::loadPCDFile(model_path, *model) == -1) {
			ROS_ERROR_STREAM("No point cloud data found in file: " << model_path);
			return false;
		}

		if (model->empty()) {
			ROS_ERROR_STREAM("Model point cloud for bin " << i << " is empty");
			return false;
		}

		bin_models.push_back(model);
	}

	return true;
}

};

