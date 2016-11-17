#pragma once

#include "local_pose_params.hpp"
#include <apc16delft_msgs/GraspCandidate.h>
#include <dr_eigen/eigen.hpp>
#include <dr_eigen/ros.hpp>

namespace apc16delft {

class GraspOrientationSynthesizer {
private:
	int bin_index_;
	Eigen::Isometry3d bin_pose_;
	LocalPoseParams params_;
	std::array<Eigen::Hyperplane<double, 3>, 5> bin_lips_;

public:
	GraspOrientationSynthesizer(int bin_index, const LocalPoseParams & params, const Eigen::Isometry3d & bin_pose, const Eigen::Vector3d & bin_dimensions) : bin_index_{bin_index}, bin_pose_{bin_pose}, params_{params} {
		bin_lips_ = getBinBoundaries(bin_pose, bin_dimensions);
	};

	void rotatePinchCandidate(apc16delft_msgs::GraspCandidate & candidate) {
		Eigen::Isometry3d pose         = dr::toEigen(candidate.stamped_pose.pose);
		Eigen::Vector3d candidate_z    = pose.rotation() * dr::axes::z();
		candidate.angled_approach      = false;
		candidate.side_angled_approach = false;

		switch (candidate.strategy) {
			case apc16delft_msgs::GraspCandidate::PENCIL_CUP_INWARDS:
				pose = pose * dr::rotateY(M_PI_2) * dr::rotateZ(-M_PI_2)  * dr::translate(0.0, -0.02, 0.005);
				break;

			case apc16delft_msgs::GraspCandidate::PENCIL_CUP_STANDING:
			case apc16delft_msgs::GraspCandidate::PENCIL_CUP_SIDEWAYS:
				pose = pose * dr::rotateZ(M_PI_2);
				break;
			case apc16delft_msgs::GraspCandidate::DUMBBELL_INWARDS:
			case apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS:
				candidate.angled_approach   = true;
				Eigen::Vector3d candidate_y = pose.rotation() * dr::axes::y();
				Eigen::Vector3d bin_y       = bin_pose_.rotation() * -1* dr::axes::y();
				bin_y.normalize();
				candidate_y.normalize();
				pose.matrix().block<3,1>(0,0) = bin_y.cross(candidate_z).normalized();
				pose.matrix().block<3,1>(0,1) = candidate_z.cross(pose.matrix().block<3,1>(0,0)).normalized();
				pose.matrix().block<3,1>(0,2) = candidate_z;

				if (candidate.strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS) {
					if (bin_index_ == -1) {
						pose = dr::translate(0.0, 0.0, 0.015) * pose * dr::rotateX(deg2rad(20.0)) * dr::translate(0.0, 0.0, 0.01);
					} else {
						pose = dr::rotate(-deg2rad(30.0), bin_pose_.rotation() * dr::axes::x(), pose.translation()) * pose;
					}
				}
				break;
		}

		candidate.stamped_pose.pose = dr::toRosPose(pose);
	}

	void rotateCandidate(apc16delft_msgs::GraspCandidate & candidate) {
		Eigen::Isometry3d pose         = dr::toEigen(candidate.stamped_pose.pose);
		Eigen::Vector3d candidate_z    = pose.rotation() * dr::axes::z();
		candidate.angled_approach      = false;
		candidate.side_angled_approach = false;

		if (bin_lips_.at(bottom_lip).signedDistance(pose.translation()) < params_.bottom_bin_edge_clearing) {
			pose = dr::rotateX(-params_.approach_angle, pose.translation()) * pose;
			candidate.angled_approach = true;
		}

		if (bin_lips_.at(right_lip).signedDistance(pose.translation()) < params_.side_bin_edge_clearing) {
			pose = dr::rotateZ(-params_.side_approach_angle, pose.translation()) * pose;
			candidate.side_angled_approach = true;

			if (facingDown(candidate_z, bin_pose_)) {
				pose = pose * dr::rotateY(params_.side_approach_angle);
			}

		} else if (bin_lips_.at(left_lip).signedDistance(pose.translation()) < params_.side_bin_edge_clearing) {
			pose = dr::rotateZ(params_.side_approach_angle, pose.translation()) * pose;
			candidate.side_angled_approach = true;

			if (facingDown(candidate_z, bin_pose_)) {
				pose = pose * dr::rotateY(-params_.side_approach_angle);
			}
		}
		candidate.stamped_pose.pose = dr::toRosPose(pose);
	}

	Eigen::Isometry3d snapGraspPose(const Eigen::Isometry3d & grasp_pose) {
		Eigen::Isometry3d snapped_grasp_pose = grasp_pose;
		Eigen::Vector3d candidate_z          = grasp_pose.rotation() * dr::axes::z();

		double dot_x = candidate_z.dot(bin_pose_.rotation() * dr::axes::x());
		double dot_y = candidate_z.dot(bin_pose_.rotation() * dr::axes::y());
		double dot_z = candidate_z.dot(bin_pose_.rotation() * dr::axes::z());

		Eigen::Vector3d snap_axis;
		bool align_y_with_y = true;
		if (std::fabs(dot_x) > std::fabs(dot_y)) {
			if (std::fabs(dot_x) > std::fabs(dot_z)) {
				snap_axis = (dot_x < 0 ? -1 : 1 ) * bin_pose_.rotation() * dr::axes::x();
			} else {
				snap_axis = (dot_z < 0 ? -1 : 1 ) * bin_pose_.rotation() * dr::axes::z();
			}
		} else {
			if (std::fabs(dot_y) > std::fabs(dot_z)) {
				snap_axis = (dot_y < 0 ? -1 : 1 ) * bin_pose_.rotation() * dr::axes::y();
				align_y_with_y = false;
			} else {
				snap_axis = (dot_z < 0 ? -1 : 1 ) * bin_pose_.rotation() * dr::axes::z();
			}
		}

		Eigen::Vector3d rotation_axis = candidate_z.cross(snap_axis);
		rotation_axis                 = rotation_axis.norm() == 0 ? rotation_axis : rotation_axis.normalized();
		double angle                  = angularDistance(snap_axis, candidate_z);
		snapped_grasp_pose            = dr::rotate(angle, rotation_axis, grasp_pose.translation()) * grasp_pose;

		if(align_y_with_y)
			snap_axis = -1 * bin_pose_.rotation() * dr::axes::y();
		else
			snap_axis = -1 * bin_pose_.rotation() * dr::axes::z();

		Eigen::Vector3d y_snap_axis_child = snapped_grasp_pose.rotation().inverse() * snap_axis;
		double angle_y_snap_to_x          = std::atan2(y_snap_axis_child.y(), y_snap_axis_child.x());
		double angle_candidate_y_to_x     = 0.5 * M_PI;

		angle = angle_y_snap_to_x - angle_candidate_y_to_x;
		Eigen::Isometry3d y_snapped_grasp_pose = snapped_grasp_pose * dr::rotateZ(angle);

		return y_snapped_grasp_pose;
	}
};

} //namespace
