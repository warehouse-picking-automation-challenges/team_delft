#include "grasp_synthesizer.hpp"
#include "deformable_grasps.hpp"

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <apc16delft_util/util.hpp>
#include <apc16delft_msgs/objects.hpp>
#include <pcl/common/transforms.h>

#include <dr_eigen/yaml.hpp>
#include <dr_eigen/ros.hpp>

namespace apc16delft {

bool GraspSynthesizer::loadParams() {
	std::stringstream param_string;

	number_of_items_                            = dr::getParam<int>(node_handle                        , "/grasping/number_of_items"                 , number_of_items_                                        );
	visualise_                                  = dr::getParam<bool>(node_handle                       , "/grasping/visualise"                       , visualise_                                              );
	std::map<std::string, GraspItem> item_map   = dr::getParamMap<std::string , GraspItem>(node_handle , "/item"                                     , std::map<std::string , GraspItem>{}                     );
	extremes_.distance_epsilon                  = dr::getParam<double>(node_handle                     , "/grasping/params/distance_epsilon"         , extremes_.distance_epsilon                              );
	extremes_.distance_tau                      = dr::getParam<double>(node_handle                     , "/grasping/params/distance_tau"             , extremes_.distance_tau                                  );
	extremes_.length_tau                        = dr::getParam<double>(node_handle                     , "/grasping/params/length_tau"               , extremes_.length_tau                                    );
	extremes_.length_epsilon                    = dr::getParam<double>(node_handle                     , "/grasping/params/length_epsilon"           , extremes_.length_epsilon                                );
	weights_.distance_weight                    = dr::getParam<double>(node_handle                     , "/grasping/params/distance_weight"          , weights_.distance_weight                                );
	weights_.length_weight                      = dr::getParam<double>(node_handle                     , "/grasping/params/length_weight"            , weights_.length_weight                                  );
	weights_.knn_weight                         = dr::getParam<double>(node_handle                     , "/grasping/params/knn_weight"               , weights_.knn_weight                                     );
	weights_.after_snap_weight                  = dr::getParam<double>(node_handle                     , "/grasping/params/after_snap_weight"        , weights_.after_snap_weight                              );
	weights_.before_snap_weight                 = dr::getParam<double>(node_handle                     , "/grasping/params/before_snap_weight"       , weights_.before_snap_weight                             );
	local_pose_params_.knn                      = dr::getParam<bool>(node_handle                       , "/grasping/params/knn"                      , local_pose_params_.knn                                  );
	local_pose_params_.knn_offset               = dr::getParam<double>(node_handle                     , "/grasping/params/knn_offset"               , local_pose_params_.knn_offset                           );
	local_pose_params_.facing_back_tolerance    = dr::getParam<double>(node_handle                     , "/grasping/params/facing_back_tolerance"    , local_pose_params_.facing_back_tolerance                );
	local_pose_params_.facing_up_tolerance      = dr::getParam<double>(node_handle                     , "/grasping/params/facing_up_tolerance"      , local_pose_params_.facing_up_tolerance                  );
	local_pose_params_.top_bin_edge_clearing    = dr::getParam<double>(node_handle                     , "/grasping/params/top_bin_edge_clearing"    , local_pose_params_.top_bin_edge_clearing                );
	local_pose_params_.bottom_bin_edge_clearing = dr::getParam<double>(node_handle                     , "/grasping/params/bottom_bin_edge_clearing" , local_pose_params_.bottom_bin_edge_clearing             );
	local_pose_params_.side_bin_edge_clearing   = dr::getParam<double>(node_handle                     , "/grasping/params/side_bin_edge_clearing"   , local_pose_params_.side_bin_edge_clearing               );
	local_pose_params_.back_bin_edge_clearing   = dr::getParam<double>(node_handle                     , "/grasping/params/back_bin_edge_clearing"   , local_pose_params_.back_bin_edge_clearing               );
	local_pose_params_.approach_angle           = deg2rad(dr::getParam<double>(node_handle             , "/grasping/params/approach_angle"           , local_pose_params_.approach_angle                      ));
	local_pose_params_.side_approach_angle      = deg2rad(dr::getParam<double>(node_handle             , "/grasping/params/side_approach_angle"      , local_pose_params_.side_approach_angle                 ));
	deformable_normal_radius_                   = dr::getParam<double>(node_handle                     , "/grasping/params/deformable_normal_radius_", deformable_normal_radius_                               );
	deformable_leaf_size_                       = dr::getParam<double>(node_handle                     , "/grasping/params/deformable_leaf_size_"    , deformable_leaf_size_                                   );

	item_list_.resize(number_of_items_);
	for (uint8_t i = 1; i <= number_of_items_; i++) {
		try {
			item_list_.at(i-1) = item_map.at(apc16delft_msgs::objectTypeToString(i));
		} catch (const std::out_of_range & e) {
			ROS_ERROR_STREAM(e.what());
			return false;
		}
	}

	tote_dimensions_ = dr::getParam<Eigen::Vector3d>(node_handle, "/tote/dimensions");

	return loadBinDimensions(bin_dimensions_, node_handle);
}

std::vector<Candidates> GraspSynthesizer::applyVisitors(apc16delft_msgs::SynthesizeGrasp::Request & req) {
	Eigen::Isometry3d object_pose  = dr::toEigen(req.object_pose.pose);
	Eigen::Isometry3d bin_pose     = dr::toEigen(req.bin_pose.pose);
	Eigen::Vector3d bin_dimensions;
	if(req.bin_index == -1) { // It's actually not a bin, but a tote.
		bin_dimensions = tote_dimensions_;
	} else {
		bin_dimensions = bin_dimensions_.at(req.bin_index);
	}

	GraspItem item = item_list_.at(req.object.type - 1);
	std::vector<Candidates> sample_spaces;
	CostFunction cost_function{extremes_, weights_};

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::fromROSMsg(req.point_cloud, *pcl_point_cloud);

	Eigen::Isometry3d parent_frame = dr::toEigen(req.cloud_transform);
	Eigen::Affine3d affine_parent_frame{parent_frame.matrix()};

	pcl::transformPointCloud(*pcl_point_cloud, *pcl_point_cloud, affine_parent_frame);
	local_pose_params_.pre_grasp_offset = item.pre_grasp_offset;
	LocalPoseProcessor local_visitor{pcl_point_cloud, req.bin_index, req.object.type, object_pose, item.center_of_mass, bin_pose, bin_dimensions, cost_function, local_pose_params_};

	for (const Shape & shape : item.sample_spaces) {
		Candidates processed_candidates_a = boost::apply_visitor(local_visitor, shape);
		for (Candidates & processed_candidates_b : sample_spaces) {
			GlobalPoseProcessor global_visitor_a{&processed_candidates_a, object_pose};
			boost::apply_visitor(global_visitor_a, processed_candidates_b.shape);

			GlobalPoseProcessor global_visitor_b{&processed_candidates_b, object_pose};
			boost::apply_visitor(global_visitor_b, processed_candidates_a.shape);
		}

		sample_spaces.push_back(processed_candidates_a);
	}

	return sample_spaces;
}

bool GraspSynthesizer::synthesizeGrasp(apc16delft_msgs::SynthesizeGrasp::Request & req, apc16delft_msgs::SynthesizeGrasp::Response & res) {
	vacuum_pose_array_.poses.clear();
	pinch_pose_array_.poses.clear();
	pinch_pose_array_.header.frame_id    = req.object_pose.header.frame_id;
	vacuum_pose_array_.header.frame_id = req.object_pose.header.frame_id;
	pinch_pose_array_.header.stamp     = ros::Time::now();
	vacuum_pose_array_.header.stamp    = ros::Time::now();

	std::vector<Candidates> sample_spaces = applyVisitors(req);
	for (Candidates & processed_candidates : sample_spaces) {
		for (apc16delft_msgs::GraspCandidate & vacuum_candidate : processed_candidates.grasp_candidates) {
			if (visualise_) {
				Eigen::Isometry3d suction_pose = dr::toEigen(vacuum_candidate.stamped_pose.pose);
				vacuum_pose_array_.poses.push_back(dr::toRosPose(suction_pose));
			}

			vacuum_candidate.stamped_pose.header.frame_id = req.object_pose.header.frame_id;
			vacuum_candidate.stamped_pose.header.stamp    = ros::Time::now();
			res.candidates.push_back(vacuum_candidate);
		}
	}

	if (visualise_) {
		GraspItem item = item_list_.at(req.object.type - 1);
		std::string model_path = resolvePackagePath(item.model_path);

		// read model
		pcl::PointCloud<pcl::PointXYZ>::Ptr model{new pcl::PointCloud<pcl::PointXYZ>};
		if (pcl::io::loadPCDFile(model_path, *model) == -1) {
			ROS_ERROR_STREAM("No model found. Model path: " << model_path);
			return false;
		}

		// check for empty model
		if (model->empty()) {
			ROS_ERROR_STREAM("Model point cloud is empty.");
			return false;
		}

		Eigen::Isometry3d parent_frame = dr::toEigen(req.object_pose.pose);
		Eigen::Affine3d affine_parent_frame{parent_frame.matrix()};
		pcl::transformPointCloud(*model, *model, affine_parent_frame);

		sensor_msgs::PointCloud2 ros_cloud;
		pcl::toROSMsg(*model, ros_cloud);
		ros_cloud.header.frame_id = "world";
		ros_cloud.header.stamp    = ros::Time::now();
		pcl_pub_.publish(ros_cloud);

		vacuum_visualiser_.publish(vacuum_pose_array_);
		pinch_visualiser_.publish(pinch_pose_array_);
	}

	return true;
}

bool GraspSynthesizer::synthesizeDeformable(apc16delft_msgs::SynthesizeDeformable::Request & req, apc16delft_msgs::SynthesizeDeformable::Response & res) {
	Eigen::Vector3d bin_dimensions;
	if(req.bin_index == -1) { // It's actually not a bin, but a tote.
		bin_dimensions = tote_dimensions_;
	} else {
		bin_dimensions = bin_dimensions_.at(req.bin_index);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>};
	pcl::fromROSMsg(req.point_cloud, *cloud);

	Eigen::Isometry3d parent_frame = dr::toEigen(req.cloud_transform);
	Eigen::Affine3d affine_parent_frame{parent_frame.matrix()};

	pcl::transformPointCloud(*cloud, *cloud, affine_parent_frame);

	if (visualise_) {
		pcl_pub_.publish(req.point_cloud);
	}

	DeformableGraspsResult result = synthesizeDeformableGrasps(cloud, deformable_normal_radius_, deformable_leaf_size_);

	PointCloud::Ptr points = std::get<0>(result);
	PointCloudNormal::Ptr normals = std::get<1>(result);
	Point centroid;
	pcl::computeCentroid(*points, centroid);

	if(points->size() != normals->size()) {
		res.error = apc16delft_msgs::SynthesizeDeformable::Response::E_DEFORMABLE_SANITY;
		res.message = "points.size() != normals.size()";
		return true;
	}

	GraspItem item = item_list_.at(req.object.type - 1);
	item.center_of_mass.x() = centroid.x;
	item.center_of_mass.y() = centroid.y;
	item.center_of_mass.z() = centroid.z;

	std::vector<Candidates> sample_spaces;
	// Loop through all shapes (for deformables should be only one anyways.)
	for (Shape shape : item.sample_spaces) {
		Candidates candidates{shape};
		candidates.vacuum = true;
		// Construct the Candidates struct.
		for(size_t i = 0; i < points->size(); ++i) {
			Point p = points->at(i);
			PointNormal normal = normals->at(i);

			Eigen::Vector3d candidate_z{normal.normal_x, normal.normal_y, normal.normal_z};
			candidate_z.normalize();
			if(req.object.type == apc16delft_msgs::Object::WOMENS_KNIT_GLOVES ||
			   req.object.type == apc16delft_msgs::Object::CHEROKEE_EASY_TEE_SHIRT ||
			   req.object.type == apc16delft_msgs::Object::COOL_SHOT_GLUE_STICKS) {
				if(req.bin_index >= 0) {
					// Cheat to only use top grasp
					candidate_z.x() = 0;
					candidate_z.y() = 0;
					candidate_z.z() = -1;
				}
			}
			if(req.bin_index == -1) {
				// Cheat to only use wall grasp
				candidate_z.x() = 0;
				candidate_z.y() = 0;
				candidate_z.z() = -1;
			}
			Eigen::Vector3d candidate_y = candidate_z.cross(dr::axes::x());
			candidate_y.normalize();
			Eigen::Vector3d candidate_x = candidate_y.cross(candidate_z);
			candidate_x.normalize();

//			ROS_INFO_STREAM("Sanity checking candidate orthogonality:");
//			ROS_INFO_STREAM("z.dot(x) = " << candidate_z.dot(candidate_x));
//			ROS_INFO_STREAM("z.dot(y) = " << candidate_z.dot(candidate_y));
//			ROS_INFO_STREAM("x.dot(y) = " << candidate_x.dot(candidate_y));
//
//			ROS_INFO_STREAM("Sanity checking candidate righthandedness:");
//			ROS_INFO_STREAM("x.dot(y.cross(z)) = " << candidate_x.dot(candidate_y.cross(candidate_z)));


			Eigen::Matrix3d R;
			R.col(0) = candidate_x;
			R.col(1) = candidate_y;
			R.col(2) = candidate_z;

//			Eigen::Quaterniond rotation{R};
//			ROS_INFO_STREAM("Quaternion.matrix: " << rotation.toRotationMatrix());
//			Eigen::Isometry3d grasp_pose = dr::toEigen(req.cloud_transform) * dr::translate(p.x, p.y, p.z) * rotation;

			Eigen::Matrix4d grasp_matrix;
			grasp_matrix.block<3,3>(0,0) = R;
			grasp_matrix(0,3) = p.x;
			grasp_matrix(1,3) = p.y;
			grasp_matrix(2,3) = p.z;
			grasp_matrix(3,3) = 1;
//			ROS_INFO_STREAM("grasp_matrix:\n" << grasp_matrix);

			Eigen::Isometry3d grasp_pose{grasp_matrix};
//			ROS_INFO_STREAM("Isometry Matrix:\n" << grasp_pose.matrix());

			apc16delft_msgs::GraspCandidate grasp_candidate_msg;

			grasp_candidate_msg.stamped_pose.pose = dr::toRosPose(grasp_pose);
			candidates.grasp_candidates.push_back(grasp_candidate_msg);
		}

		// Apply LocalPoseProcessor.
		CostFunction cost_function{extremes_, weights_};
		local_pose_params_.pre_grasp_offset = item.pre_grasp_offset;
		LocalPoseProcessor local_visitor{req.bin_index, req.object.type, candidates.grasp_candidates, item.center_of_mass, dr::toEigen(req.bin_pose.pose), bin_dimensions, cost_function, local_pose_params_};
		Candidates processed_candidates = boost::apply_visitor(local_visitor, shape);
	//	Candidates processed_candidates = candidates;
		sample_spaces.push_back(processed_candidates);
	}

	// Visualise and push back to service response.
	for (Candidates & processed_candidates : sample_spaces) {
		for (apc16delft_msgs::GraspCandidate & vacuum_candidate : processed_candidates.grasp_candidates) {
			if (visualise_) {
				Eigen::Isometry3d suction_pose = dr::toEigen(vacuum_candidate.stamped_pose.pose);
				vacuum_pose_array_.poses.push_back(dr::toRosPose(suction_pose));
			}

			vacuum_candidate.stamped_pose.header.frame_id = "world";
			vacuum_candidate.stamped_pose.header.stamp    = ros::Time::now();
			res.candidates.push_back(vacuum_candidate);
		}
	}

	if (visualise_) {
		//vacuum_pose_array_.header.frame_id = req.point_cloud.header.frame_id;
		vacuum_pose_array_.header.frame_id = "world";
		vacuum_pose_array_.header.stamp    = ros::Time::now();
		vacuum_visualiser_.publish(vacuum_pose_array_);
	}

	return true;
}

int GraspSynthesizer::onConfigure() {
	try {
		loadParams();
	} catch (const std::out_of_range & e) {
		ROS_ERROR_STREAM("Item map is out of range " << e.what());
		ROS_ERROR_STREAM("Make sure to launch apc16delft_data param.launch first.");
		return 1;
	}

	return 0;
}

int GraspSynthesizer::onActivate() {
	synthesize_grasp_      = node_handle.advertiseService("synthesize_grasp", &GraspSynthesizer::synthesizeGrasp, this);
	synthesize_deformable_ = node_handle.advertiseService("synthesize_deformable", &GraspSynthesizer::synthesizeDeformable, this);
	vacuum_visualiser_     = node_handle.advertise<geometry_msgs::PoseArray>("visualise_vacuum", 1, true);
	pinch_visualiser_      = node_handle.advertise<geometry_msgs::PoseArray>("visualise_pinch", 1, true);
	pcl_pub_               = node_handle.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);

	return 0;
}

} // namespace

int main(int argc, char ** argv) {
	ROS_INFO("ROS NODE grasp_synthesizer started");
	ros::init(argc, argv, "grasp_synthesizer");
	apc16delft::GraspSynthesizer grasp_synthesizer;
	ros::spin();
	return 0;
}


