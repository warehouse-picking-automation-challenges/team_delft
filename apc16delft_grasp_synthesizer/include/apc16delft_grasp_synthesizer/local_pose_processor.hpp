#pragma once

#include "local_pose_params.hpp"
#include "cost_function.hpp"
#include "grasp_item.hpp"
#include "grasp_orientation_synthesizer.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <boost/variant.hpp>
#include <dr_eigen/ros.hpp>
#include <dr_eigen/yaml.hpp>

using LocalPoseParams = struct LocalPoseParams;

namespace apc16delft {
constexpr double dumbbell_offset = 0.02;
constexpr double pencil_cup_offset = 0.02;

/// Visitor to return the correct candidates within a shape.
class LocalPoseProcessor : public boost::static_visitor<Candidates> {

private:
	/// Raw point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;

	/// Pose of the object in bin_frame.
	Eigen::Isometry3d object_pose_;

	/// Pose of the bin.
	Eigen::Isometry3d bin_pose_;

	/// Origin of the sample space in object_frame.
	Eigen::Isometry3d origin_;

	/// CoM of the item.
	Eigen::Vector3d center_of_mass_;

	/// The parameters for this class.
	LocalPoseParams params_;

	/// The orientation synthesizer.
	GraspOrientationSynthesizer grasp_orientation_synthesizer_;

	/// The cost function needed to calculate costs.
	CostFunction cost_function_;

	/// The lips of the current bin.
	std::array<Eigen::Hyperplane<double, 3>, 5> bin_lips_;

	std::vector<apc16delft_msgs::GraspCandidate> deformable_grasp_candidates_;

	uint8_t object_type_;
	int bin_index_;

	/// Use K-Nearest-Neighbour to refine grasp poses w.r.t. a given point_cloud.
	bool refineWithPointCloud(apc16delft_msgs::GraspCandidate & candidate) {
		pcl::PointXYZ search_point;
		Eigen::Isometry3d grasp_pose = dr::toEigen(candidate.stamped_pose.pose);
		search_point.x = grasp_pose.translation().x();
		search_point.y = grasp_pose.translation().y();
		search_point.z = grasp_pose.translation().z();

		std::vector<int> point_indices(1);
		std::vector<float> squared_distances(1);
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud (point_cloud_);

		if (kdtree.nearestKSearch(search_point, 1, point_indices, squared_distances) > 0) {
			for (size_t i = 0; i < point_indices.size (); i++) {
				if (sqrt(squared_distances.at(i)) > params_.knn_offset) {
					return false;
				}

				grasp_pose.translation().x() = point_cloud_->points[point_indices.at(i)].x;
				grasp_pose.translation().y() = point_cloud_->points[point_indices.at(i)].y;
				grasp_pose.translation().z() = point_cloud_->points[point_indices.at(i)].z;
				candidate.stamped_pose.pose  = dr::toRosPose(grasp_pose);

				return true;
			}
		}

		// By default return false if we don't find any nearest neighbours.
		return false;
	}

	bool validDumbbellCandidate(apc16delft_msgs::GraspCandidate & candidate) {
		Eigen::Isometry3d grasp_pose = dr::toEigen(candidate.stamped_pose.pose);
		Eigen::Vector3d candidate_z  = grasp_pose.rotation() * dr::axes::z();
		Eigen::Vector3d object_x     = bin_pose_.rotation().inverse() * object_pose_.rotation() * dr::axes::x();

		double dot_x = std::abs(object_x.dot(dr::axes::x()));
		double dot_y = std::abs(object_x.dot(dr::axes::y()));
		double dot_z = std::abs(object_x.dot(dr::axes::z()));

		bool standing  = (dot_z > dot_x) && (dot_z > dot_y);
		bool inwards   = (dot_y > dot_x) && (dot_y > dot_z);
		bool sidewards = (dot_x > dot_z) && (dot_x > dot_y);

		if (inwards) {
			if (candidate.strategy != apc16delft_msgs::GraspCandidate::DUMBBELL_INWARDS) {
				return false;
			}
		}

		if (standing || sidewards) {
			if (candidate.strategy != apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS) {
				return false;
			}
		}

		double object_pose_y = (bin_pose_.rotation().inverse() * object_pose_.translation()).y();
		double grasp_pose_y  = (bin_pose_.rotation().inverse() * grasp_pose.translation()).y();
		if (grasp_pose_y > object_pose_y + pencil_cup_offset) {
			return false;
		}

		return true;
	}

	/// Check whether a candidate is valid.
	bool isValidCandidate(apc16delft_msgs::GraspCandidate & candidate) {
		Eigen::Isometry3d grasp_pose = dr::toEigen(candidate.stamped_pose.pose);
		Eigen::Vector3d candidate_z  = grasp_pose.rotation() * dr::axes::z();

		if (facingBack(candidate_z, bin_pose_, -params_.facing_back_tolerance)) {
			if (object_type_ != apc16delft_msgs::Object::FITNESS_GEAR_3LB_DUMBBELL &&
				object_type_ != apc16delft_msgs::Object::ROLODEX_JUMBO_PENCIL_CUP)
				return false;
		}

		if (bin_index_ >= 0 && facingUp(candidate_z, bin_pose_, -params_.facing_up_tolerance)) {
			return false;
		}

		if (params_.knn && (facingFront(candidate_z, bin_pose_) || facingDown(candidate_z, bin_pose_))) {
			// If no suitable neighbour is found, reject this pose.
			if (!refineWithPointCloud(candidate)) {
				return false;
			}
		}

		return true;
	}

	void determineStrategy(apc16delft_msgs::GraspCandidate & transformed_candidate, bool vacuum) {
		Eigen::Isometry3d transformed_pose = dr::toEigen(transformed_candidate.stamped_pose.pose);
		Eigen::Vector3d candidate_y = transformed_pose.rotation() * dr::axes::y();
		Eigen::Vector3d candidate_z = transformed_pose.rotation() * dr::axes::z();

		if (vacuum) {
			if (facingDown(candidate_y, bin_pose_)) {
				transformed_candidate.strategy = (uint8_t) (isBig(object_type_) ? apc16delft_msgs::GraspCandidate::VACUUM_V_BIG : apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_V);

				if (bin_index_ >= 0) {
					transformed_candidate.stamped_pose.pose = dr::toRosPose(transformed_pose *  dr::rotateZ(-M_PI_2));
				}
			} else {
				Eigen::Vector3d candidate_z = transformed_pose.rotation() * dr::axes::z();
				transformed_candidate.strategy = (uint8_t) (isBig(object_type_) ? apc16delft_msgs::GraspCandidate::VACUUM_H_BIG : apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_H);

				if (facingLeft(candidate_z, bin_pose_, 0.9) || facingRight(candidate_z, bin_pose_, 0.9)) {
					// Penalise side grasps a bit.
					transformed_candidate.score *= 0.70;
				}
			}
		} else {
			if (object_type_ == apc16delft_msgs::Object::ROLODEX_JUMBO_PENCIL_CUP) {
				if (facingDown(candidate_z, bin_pose_) || facingUp(candidate_z, bin_pose_)) {
					transformed_candidate.strategy = apc16delft_msgs::GraspCandidate::PENCIL_CUP_STANDING;
				} else if (facingFront(candidate_z, bin_pose_) || facingBack(candidate_z, bin_pose_)) {
					transformed_candidate.strategy = apc16delft_msgs::GraspCandidate::PENCIL_CUP_INWARDS;
				} else if (facingLeft(candidate_z, bin_pose_) || facingRight(candidate_z, bin_pose_)) {
					transformed_candidate.strategy = apc16delft_msgs::GraspCandidate::PENCIL_CUP_SIDEWAYS;
				}
			}
		}
		transformed_candidate.pre_grasp_offset = params_.pre_grasp_offset;
	}

	bool closeToWall(const apc16delft_msgs::GraspCandidate & candidate) {
		Eigen::Isometry3d grasp_pose = dr::toEigen(candidate.stamped_pose.pose);
		Eigen::Vector3d candidate_z  = grasp_pose.rotation() * dr::axes::z();

		if (object_type_ == apc16delft_msgs::Object::KLEENEX_PAPER_TOWELS) {
			if (facingLeft(candidate_z, bin_pose_) || facingRight(candidate_z, bin_pose_)) {
				return true;
			}

			if (bin_index_ > 0 && (candidate.strategy != apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_H && candidate.strategy != apc16delft_msgs::GraspCandidate::VACUUM_H_BIG)) {
				return true;
			}
		}

		if (candidate.strategy == candidate.STRATEGY_VACUUM_H || candidate.strategy == candidate.VACUUM_H_BIG) {
			// Suction from side
			if(facingLeft(candidate_z, bin_pose_) || facingRight(candidate_z, bin_pose_)) {
				uint8_t wall_index = facingRight(candidate_z, bin_pose_) ? left_lip : right_lip;
				Eigen::Hyperplane<double, 3> wall = bin_lips_.at(wall_index);

				// If facing right, we check its closeness to the left wall and vice versa.
				if (wall.signedDistance(grasp_pose.translation()) < params_.side_bin_edge_clearing) {
					return true;
				}
			}
		}

		Eigen::Hyperplane<double, 3> back_wall = bin_lips_.at(back_lip);
		double front_distance = 0.35;
		double post_side_clearing = 0.05;
		if((candidate.strategy == candidate.VACUUM_V_BIG || candidate.strategy == candidate.STRATEGY_VACUUM_V) &&
			back_wall.signedDistance(grasp_pose.translation()) > front_distance) { // TODO: Magic number
			if(bin_index_ == 0 ||
			   bin_index_ == 3 ||
			   bin_index_ == 6 ||
			   bin_index_ == 9) {
				// Check the left side
				Eigen::Hyperplane<double, 3> wall = bin_lips_.at(left_lip);
				if(wall.signedDistance(grasp_pose.translation()) < post_side_clearing) {
					return true;
				}
			}

			if(bin_index_ == 2 ||
			   bin_index_ == 5 ||
			   bin_index_ == 8 ||
			   bin_index_ == 11) {
				// Check the right side
				Eigen::Hyperplane<double, 3> wall = bin_lips_.at(right_lip);
				if(wall.signedDistance(grasp_pose.translation()) < post_side_clearing) {
					return true;
				}
			}
		}
		return false;
	}

	apc16delft_msgs::GraspCandidate handlePinch(const std::vector<apc16delft_msgs::GraspCandidate> & candidates, bool min) {
		std::vector<apc16delft_msgs::GraspCandidate> transformed_poses;

		for (const apc16delft_msgs::GraspCandidate & candidate : candidates) {
			apc16delft_msgs::GraspCandidate transformed_candidate = candidate;
			Eigen::Isometry3d transformed_pose = object_pose_ * origin_ * dr::toEigen(candidate.stamped_pose.pose);
			transformed_candidate.stamped_pose.pose = dr::toRosPose(transformed_pose);
			transformed_poses.push_back(transformed_candidate);
		}

		apc16delft_msgs::GraspCandidate first_pose = transformed_poses.at(0);
		Eigen::Vector3d object_x = bin_pose_.rotation().inverse() * (object_pose_.rotation() * dr::axes::x());
		Eigen::Vector3d object_z = bin_pose_.rotation().inverse() * (object_pose_.rotation() * dr::axes::z());
		double dot_x_x = std::abs(object_x.dot(dr::axes::x()));
		double dot_x_z = std::abs(object_x.dot(dr::axes::z()));
		double dot_z_x = std::abs(object_z.dot(dr::axes::x()));
		double dot_z_y = std::abs(object_z.dot(dr::axes::y()));
		double dot_z_z = std::abs(object_z.dot(dr::axes::z()));

		bool standing, inwards, sidewards;
		if (object_type_ == apc16delft_msgs::Object::FITNESS_GEAR_3LB_DUMBBELL) {
			standing = dot_x_z > dot_x_x;
		} else if (object_type_ == apc16delft_msgs::Object::ROLODEX_JUMBO_PENCIL_CUP) {
			standing  = (dot_z_z > dot_z_x) && (dot_z_z > dot_z_y);
			inwards   = (dot_z_y > dot_z_x) && (dot_z_y > dot_z_z);
			sidewards = (dot_z_x > dot_z_z) && (dot_z_x > dot_z_y);
		}

		for (const apc16delft_msgs::GraspCandidate & transformed_candidate : transformed_poses) {
			Eigen::Isometry3d candidate = dr::toEigen(transformed_candidate.stamped_pose.pose);
			Eigen::Vector3d translation = bin_pose_.rotation().inverse() * candidate.translation();
			double to_compare, current_max;

			if (object_type_ == apc16delft_msgs::Object::ROLODEX_JUMBO_PENCIL_CUP) {
				if (standing) {
					to_compare  = translation.y();
					current_max = (bin_pose_.rotation().inverse() * dr::toEigen(first_pose.stamped_pose.pose).translation()).y();
				} else if (inwards) {
					to_compare  = translation.z();
					current_max = (bin_pose_.rotation().inverse() * dr::toEigen(first_pose.stamped_pose.pose).translation()).z();
				} else if (sidewards) {
					to_compare  = translation.y();
					current_max = (bin_pose_.rotation().inverse() * dr::toEigen(first_pose.stamped_pose.pose).translation()).y();
				}
			} else if (object_type_ == apc16delft_msgs::Object::FITNESS_GEAR_3LB_DUMBBELL) {
				if (transformed_candidate.strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_INWARDS) {
					to_compare = translation.z();
					current_max = (bin_pose_.rotation().inverse() * dr::toEigen(first_pose.stamped_pose.pose).translation()).z();
				} else if (transformed_candidate.strategy == apc16delft_msgs::GraspCandidate::DUMBBELL_SIDEWARDS) {
					if (standing) {
						to_compare  = translation.x();
						current_max = (bin_pose_.rotation().inverse() * dr::toEigen(first_pose.stamped_pose.pose).translation()).x();
					} else {
						to_compare  = translation.z();
						current_max = (bin_pose_.rotation().inverse() * dr::toEigen(first_pose.stamped_pose.pose).translation()).z();
					}
				}
			}

			if (min) {
				if (to_compare < current_max) {
					first_pose = transformed_candidate;
				}
			} else {
				if (to_compare > current_max) {
					first_pose = transformed_candidate;
				}
			}
		}

		first_pose.score = 0.1;
		return first_pose;
	}

	apc16delft_msgs::GraspCandidate handlePencilCup(const std::vector<apc16delft_msgs::GraspCandidate> & candidates, bool min) {
		apc16delft_msgs::GraspCandidate transformed_candidate = handlePinch(candidates, min);
		Eigen::Isometry3d pose                                = dr::toEigen(transformed_candidate.stamped_pose.pose);
		Eigen::Isometry3d snapped_pose                        = grasp_orientation_synthesizer_.snapGraspPose(pose);

		apc16delft_msgs::GraspCandidate snapped_candidate = transformed_candidate;
		geometry_msgs::Pose snapped_ros_pose              = dr::toRosPose(snapped_pose);
		snapped_candidate.stamped_pose.pose               = snapped_ros_pose;

		determineStrategy(snapped_candidate, false);

		transformed_candidate.stamped_pose.pose = dr::toRosPose(pose);
		transformed_candidate.strategy          = snapped_candidate.strategy;
		transformed_candidate.pre_grasp_offset  = params_.pre_grasp_offset;
		grasp_orientation_synthesizer_.rotatePinchCandidate(transformed_candidate);
		return transformed_candidate;
	}

	apc16delft_msgs::GraspCandidate handleDumbbell(const std::vector<apc16delft_msgs::GraspCandidate> & candidates, bool min) {
		apc16delft_msgs::GraspCandidate transformed_candidate = handlePinch(candidates, min);

		grasp_orientation_synthesizer_.rotatePinchCandidate(transformed_candidate);
		Eigen::Isometry3d pose                  = dr::toEigen(transformed_candidate.stamped_pose.pose);
		transformed_candidate.pre_grasp_offset  = params_.pre_grasp_offset;
		transformed_candidate.stamped_pose.pose = dr::toRosPose(pose);
		return transformed_candidate;
	}

	/// Calculate the parent frame pose out of the predefined local grasp pose for a single vector of candidates.
	std::vector<apc16delft_msgs::GraspCandidate> calculateParentPose(const std::vector<apc16delft_msgs::GraspCandidate> & candidates, bool vacuum) {
		std::vector<apc16delft_msgs::GraspCandidate> transformed_candidates;
		apc16delft_msgs::GraspCandidate transformed_candidate;

		if (vacuum) {
			for (const apc16delft_msgs::GraspCandidate & candidate : candidates) {
				Eigen::Isometry3d transformed_pose = object_pose_ * origin_ * dr::toEigen(candidate.stamped_pose.pose);

				geometry_msgs::Pose transformed_ros_pose = dr::toRosPose(transformed_pose);
				transformed_candidate.stamped_pose.pose  = transformed_ros_pose;

				if (!isValidCandidate(transformed_candidate)) {
					continue;
				}

				transformed_pose = dr::toEigen(transformed_candidate.stamped_pose.pose);

				double score_before_snap                = cost_function_.calculateScore(1.0, dr::toEigen(candidate.stamped_pose.pose), center_of_mass_);
				transformed_pose                        = grasp_orientation_synthesizer_.snapGraspPose(transformed_pose);
				transformed_ros_pose                    = dr::toRosPose(transformed_pose);
				transformed_candidate.stamped_pose.pose = transformed_ros_pose;

				Eigen::Vector3d z_axis                  = transformed_pose.rotation() * dr::axes::z();
				double knn_weight                       = (params_.knn && (facingFront(z_axis, bin_pose_) || facingDown(z_axis, bin_pose_))) ? cost_function_.weights.knn_weight : 1.0;
				double score_after_snap                 = cost_function_.calculateScore(knn_weight, object_pose_.inverse() * origin_.inverse() * transformed_pose, center_of_mass_);
				transformed_candidate.score             = cost_function_.weights.after_snap_weight * score_after_snap + cost_function_.weights.before_snap_weight * score_before_snap;

				determineStrategy(transformed_candidate, true);

				if(object_type_ == apc16delft_msgs::Object::KLEENEX_TISSUE_BOX &&
					bin_index_ > 3 && bin_index_ < 9 &&
					facingDown(z_axis, bin_pose_) ) {
					// No top grasps for KLEENEX_TISSUE_BOX in the middle bins
					continue;
				}

				if (closeToWall(transformed_candidate)) {
					continue;
				}

				// Use angled approach when needed.
				grasp_orientation_synthesizer_.rotateCandidate(transformed_candidate);
				transformed_candidates.push_back(transformed_candidate);
			}
		} else {
			if (object_type_ == apc16delft_msgs::Object::ROLODEX_JUMBO_PENCIL_CUP) {
				transformed_candidate = handlePencilCup(candidates, false);
				if (isValidCandidate(transformed_candidate)) {
					transformed_candidates.push_back(transformed_candidate);
				}
				apc16delft_msgs::GraspCandidate transformed_candidate2 = handlePencilCup(candidates, true);
				if (isValidCandidate(transformed_candidate2)) {
					transformed_candidates.push_back(transformed_candidate2);
				}
			} else if (object_type_ == apc16delft_msgs::Object::FITNESS_GEAR_3LB_DUMBBELL) {
				transformed_candidate = handleDumbbell(candidates, false);
				if (isValidCandidate(transformed_candidate) && validDumbbellCandidate(transformed_candidate)) {
					transformed_candidates.push_back(transformed_candidate);
				}
				apc16delft_msgs::GraspCandidate transformed_candidate2 = handleDumbbell(candidates, true);
				if (isValidCandidate(transformed_candidate2) && validDumbbellCandidate(transformed_candidate)) {
					transformed_candidates.push_back(transformed_candidate2);
				}
			}
		}

		return transformed_candidates;
	}

	/// Calculate the pose in parent frame out of the predefined local grasp pose for a Candidates struct (both vacuum and pinch candidates).
	Candidates calculateParentPose(const Candidates & candidates) {
		Candidates transformed_candidates{candidates.shape};
		transformed_candidates.grasp_candidates = calculateParentPose(candidates.grasp_candidates, candidates.vacuum);
		return transformed_candidates;
	}

	/// Calculate the parent frame pose out of the predefined local grasp pose for a single vector of candidates.
	std::vector<apc16delft_msgs::GraspCandidate> calculateDeformablePose(const std::vector<apc16delft_msgs::GraspCandidate> & candidates, bool vacuum) {
		std::vector<apc16delft_msgs::GraspCandidate> transformed_candidates;
		apc16delft_msgs::GraspCandidate transformed_candidate;

		for (const apc16delft_msgs::GraspCandidate & candidate : candidates) {
			Eigen::Isometry3d transformed_pose = dr::toEigen(candidate.stamped_pose.pose);
			transformed_candidate.score        = cost_function_.calculateDeformableScore(dr::toEigen(candidate.stamped_pose.pose), center_of_mass_);

			geometry_msgs::Pose transformed_ros_pose = dr::toRosPose(transformed_pose);
			transformed_candidate.stamped_pose.pose  = transformed_ros_pose;

			// If candidate is invalid, skip.
			if (!isValidCandidate(transformed_candidate)) {
				continue;
			}

			transformed_pose                         = grasp_orientation_synthesizer_.snapGraspPose(transformed_pose);
			transformed_ros_pose = dr::toRosPose(transformed_pose);
			transformed_candidate.stamped_pose.pose  = transformed_ros_pose;

			determineStrategy(transformed_candidate, vacuum);

			if (object_type_ == apc16delft_msgs::Object::KYJEN_SQUEAKIN_EGGS_PLUSH_PUPPIES) {
				if (transformed_candidate.strategy == apc16delft_msgs::GraspCandidate::STRATEGY_VACUUM_V)
					continue;
			}

			if (closeToWall(transformed_candidate)) {
				continue;
			}

			// Use angled approach when needed.
			grasp_orientation_synthesizer_.rotateCandidate(transformed_candidate);

			transformed_candidates.push_back(transformed_candidate);
		}

		return transformed_candidates;
	}

	Candidates calculateDeformablePose(const Candidates & candidates) {
		Candidates results{candidates.shape};
		results.grasp_candidates = calculateDeformablePose(candidates.grasp_candidates, candidates.vacuum);

		return results;
	}

public:
	LocalPoseProcessor(const pcl::PointCloud<pcl::PointXYZ>::Ptr & point_cloud, int bin_index, uint8_t object_type, const Eigen::Isometry3d & object_pose, const Eigen::Vector3d & center_of_mass, const Eigen::Isometry3d & bin_pose, const Eigen::Vector3d & bin_dimensions, const CostFunction & cost_function, const LocalPoseParams & params) :
	point_cloud_{point_cloud},
	object_pose_{object_pose},
	bin_pose_{bin_pose},
	center_of_mass_{center_of_mass},
	params_{params},
	grasp_orientation_synthesizer_{bin_index, params_, bin_pose, bin_dimensions},
	cost_function_{cost_function},
	object_type_{object_type},
	bin_index_{bin_index} {
		bin_lips_ = getBinBoundaries(bin_pose, bin_dimensions);
	};

	LocalPoseProcessor(int bin_index, uint8_t object_type, const std::vector<apc16delft_msgs::GraspCandidate> & candidates, const Eigen::Vector3d & center_of_mass, const Eigen::Isometry3d & bin_pose, const Eigen::Vector3d & bin_dimensions, const CostFunction & cost_function, const LocalPoseParams & params) :
	center_of_mass_{center_of_mass},
	bin_pose_{bin_pose},
	params_{params},
	grasp_orientation_synthesizer_{bin_index, params_, bin_pose, bin_dimensions},
	cost_function_{cost_function},
	deformable_grasp_candidates_{candidates},
	object_type_{object_type},
	bin_index_{bin_index} {
		bin_lips_ = getBinBoundaries(bin_pose, bin_dimensions);
	};

	Candidates operator()(const Cone & cone) {
		Candidates candidates{cone};
		candidates.grasp_candidates = cone.grasp_candidates;
		candidates.vacuum           = cone.vacuum;
		origin_                     = cone.origin;

		return calculateParentPose(candidates);
	}

	Candidates operator()(const Sphere & sphere) {
		Candidates candidates{sphere};
		candidates.grasp_candidates = sphere.grasp_candidates;
		candidates.vacuum           = sphere.vacuum;
		origin_                     = sphere.origin;

		return calculateParentPose(candidates);
	}

	Candidates operator()(const Bar & bar) {
		Candidates candidates{bar};
		candidates.grasp_candidates = bar.grasp_candidates;
		candidates.vacuum           = bar.vacuum;
		origin_                     = bar.origin;

		return calculateParentPose(candidates);
	}

	Candidates operator()(const Cylinder & cylinder) {
		Candidates candidates{cylinder};
		candidates.grasp_candidates = cylinder.grasp_candidates;
		candidates.vacuum           = cylinder.vacuum;
		origin_                     = cylinder.origin;

		return calculateParentPose(candidates);
	}

	Candidates operator()(const Deformable & deformable) {
		Candidates candidates{deformable};
		candidates.grasp_candidates = deformable_grasp_candidates_;
		candidates.vacuum           = deformable.vacuum;
		origin_                     = deformable.origin;

		return calculateDeformablePose(candidates);
	}

	Candidates operator()(const Ring & ring) {
		Candidates candidates{ring};
		candidates.grasp_candidates = ring.grasp_candidates;
		candidates.vacuum           = ring.vacuum;
		origin_                     = ring.origin;

		return calculateParentPose(candidates);
	}

	Candidates operator()(const Circle & circle) {
		Candidates candidates{circle};
		candidates.grasp_candidates = circle.grasp_candidates;
		candidates.vacuum           = circle.vacuum;
		origin_                     = Eigen::Isometry3d::Identity();

		return calculateParentPose(candidates);
	}

	Candidates operator()(const Manual & manual) {
		Candidates candidates{manual};
		candidates.grasp_candidates = manual.grasp_candidates;
		candidates.vacuum           = manual.vacuum;
		origin_                     = manual.origin;

		return calculateParentPose(candidates);
	}
};

}
