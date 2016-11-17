#include <dr_eigen/ros.hpp>
#include <dr_eigen/param.hpp>
#include <dr_eigen/yaml.hpp>
#include <dr_param/param.hpp>

#include <apc16delft_msgs/EstimateBinPose.h>
#include <apc16delft_util/util.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

namespace apc16delft {

class BinPoseEstimationNode {

using Point              = pcl::PointXYZ;
using PointCloud         = pcl::PointCloud<Point>;
using PoseEstimateResult = std::tuple<Eigen::Isometry3d, PointCloud::Ptr, double>;

public:
	BinPoseEstimationNode() :
		node_handle("~")
	{
		// setup ros service servers
		bin_pose_estimation_server   = node_handle.advertiseService("estimate_bin_pose", &BinPoseEstimationNode::estimateBinPose, this);

		// setup ros publishers
		bin_pose_publisher          = node_handle.advertise<geometry_msgs::PoseStamped>("bin_pose", 1, true);
		initial_guess_publisher     = node_handle.advertise<geometry_msgs::PoseStamped>("initial_guess", 1, true);
		transformed_model_publisher = node_handle.advertise<PointCloud>("transformed_model", 1, true);
		scene_publisher             = node_handle.advertise<PointCloud>("scene", 1, true);

		// load parameters
		node_handle.param<double>("leaf_size", leaf_size, 0.005);
		node_handle.param<int>("icp_max_iterations", icp_max_iterations, 20);
		node_handle.param<double>("sanity_translation_threshold", sanity_translation_threshold, 0.1);
		node_handle.param<double>("sanity_orientation_threshold_degree", sanity_orientation_threshold, 10.0);
		sanity_orientation_threshold = sanity_orientation_threshold / 180.0 * M_PI; // convert to [rad]

		// load bin poses and models
		for (int i = int(BinIndex::BIN_A); i <= int(BinIndex::BIN_L); i++) {
			bin_poses.push_back(dr::getParam<Eigen::Isometry3d>(node_handle, "/bin/" + std::to_string(i) + "/pose"));
			ROS_INFO_STREAM(std::string("bin_") + char(int('A') +  i) << " pose: " << dr::toYaml(bin_poses.back()));

			std::string model_path;
			if (!node_handle.getParam("/bin/" + std::to_string(i) + "/model_path", model_path)) {
				ROS_WARN_STREAM("Failed to read model path for bin: " << i);
				bin_models.emplace_back(new PointCloud);
				continue;
			}

			// resolve package://* to file package path
			model_path = apc16delft::resolvePackagePath(model_path);

			// check if file exists
			if (!boost::filesystem::exists(model_path)) {
				ROS_WARN_STREAM("Model file for bin " << i << " does not exist. Path: " << model_path);
				bin_models.emplace_back(new PointCloud);
				continue;
			}

			// read model
			PointCloud::Ptr model(new PointCloud);
			if (pcl::io::loadPCDFile(model_path, *model) == -1) {
				ROS_WARN_STREAM("No model found. Model path: " << model_path);
				bin_models.emplace_back(new PointCloud);
				continue;
			}

			// check for empty model
			if (model->empty()) {
				ROS_ERROR_STREAM("Model point cloud is empty.");
				bin_models.emplace_back(new PointCloud);
				continue;
			}

			// add to cache
			bin_models.push_back(model);
		}
	}

protected:
	/// Handle for ros related stuff.
	ros::NodeHandle node_handle;

	/// Service server for handling request of bin poses.
	ros::ServiceServer bin_pose_estimation_server;

	/// Publishes detected bin poses.
	ros::Publisher bin_pose_publisher;

	/// Publishes the initial guess.
	ros::Publisher initial_guess_publisher;

	/// Publishes the transformed model.
	ros::Publisher transformed_model_publisher;

	/// Publishes the transformed model.
	ros::Publisher scene_publisher;

	/// Leaf size with which to reduce the size of the input clouds.
	double leaf_size;

	/// Maximum number of iterations for ICP pose estimation.
	int icp_max_iterations;

	/// Poses of the bin w.r.t. the shelf.
	std::vector<Eigen::Isometry3d> bin_poses;

	/// Models of the bin.
	std::vector<PointCloud::Ptr> bin_models;

	/// Threshold (in [m]) which the bin pose estimation may divert from the initial guess.
	double sanity_translation_threshold;

	/// Threshold (in [rad]) which the orientation of the bin pose estimation may divert from the initial guess.
	double sanity_orientation_threshold;

	PoseEstimateResult estimateBinPoseImpl(PointCloud::Ptr scene, Eigen::Isometry3d const & initial_guess, PointCloud::ConstPtr model) {
		// create the filtering object
		//pcl::VoxelGrid<Point> voxel_filter;
		//voxel_filter.setInputCloud(scene);
		//voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
		//voxel_filter.filter(*scene);

		//// initialize ICP algorithm
		//pcl::IterativeClosestPoint<Point, Point> icp;
		//icp.setInputSource(model);
		//icp.setInputTarget(scene);
		//icp.setMaximumIterations(icp_max_iterations);

		//pcl::PointCloud<Point>::Ptr aligned_model(new PointCloud);
		//icp.align(*aligned_model, initial_guess.cast<float>().matrix());
		//aligned_model->header = scene->header;

		// store transformation as Eigen::Isometry3d
		//return std::make_tuple(Eigen::Isometry3d{icp.getFinalTransformation().cast<double>().matrix()}, aligned_model, icp.getFitnessScore());
		
		PointCloud::Ptr ret{new PointCloud(*model)};
		return std::make_tuple(initial_guess, ret, 1.00);
	}

	bool estimateBinPose(apc16delft_msgs::EstimateBinPose::Request & req, apc16delft_msgs::EstimateBinPose::Response & res) {
		// convert sensor_msgs/PointCloud2 to pcl::PointCloud
		PointCloud::Ptr scene(new PointCloud);
		pcl::fromROSMsg(req.scene, *scene);

		// check empty scene
		if (scene->empty()) {
			res.error.code = apc16delft_msgs::EstimateBinPoseResponse::E_SCENE_EMPTY;
			res.error.message = "The supplied scene point cloud is empty.";
			return true;
		}

		scene_publisher.publish(scene);

		Eigen::Isometry3d shelf_initial_guess = dr::toEigen(req.shelf_initial_guess);
		Eigen::Isometry3d initial_guess       = shelf_initial_guess * bin_poses[req.bin_index];

		// store transformation as Eigen::Isometry3d
		PoseEstimateResult bin_pose_output = estimateBinPoseImpl(scene, initial_guess, bin_models[req.bin_index]);
		Eigen::Isometry3d bin_pose         = std::get<0>(bin_pose_output);

		// copy to output
		res.pose             = dr::toRosPoseStamped(bin_pose, req.scene.header.frame_id, req.scene.header.stamp);
		res.estimation_error = std::get<2>(bin_pose_output);

		// publish debug data
		bin_pose_publisher.publish(res.pose);
		initial_guess_publisher.publish(dr::toRosPoseStamped(initial_guess, req.scene.header.frame_id, req.scene.header.stamp));
		transformed_model_publisher.publish(std::get<1>(bin_pose_output));

		double translation = (initial_guess.translation() - bin_pose.translation()).norm();
		if (translation > sanity_translation_threshold) {
			res.error.code    = apc16delft_msgs::EstimateBinPoseResponse::E_SANITY_TRANSLATION;
			res.error.message = "Bin pose translation is too far off from the initial guess (translation distance = " + std::to_string(translation) + ")";
			return true;
		}
		double orientation = Eigen::AngleAxisd{initial_guess.rotation().inverse() * bin_pose.rotation()}.angle();
		if (orientation > sanity_orientation_threshold) {
			res.error.code    = apc16delft_msgs::EstimateBinPoseResponse::E_SANITY_ORIENTATION;
			res.error.message = "Bin pose orientation is too far off from the initial guess (orientation distance = " + std::to_string(orientation) + ")";
			return true;
		}

		return true;
	}
};

}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "bin_pose_estimation");
	apc16delft::BinPoseEstimationNode node;
	ros::spin();
}

