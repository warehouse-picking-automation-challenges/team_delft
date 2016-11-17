#include <apc16delft_util/managed_node.hpp>
#include <apc16delft_util/util.hpp>
#include <apc16delft_msgs/EstimateObjectPose.h>
#include <apc16delft_msgs/EstimateBinPose.h>
#include <apc16delft_msgs/objects.hpp>

#include <dr_eigen/ros.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/init.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include <pcl/search/impl/search.hpp>

namespace apc16delft {

class PoseEstimationNode : public ManagedNode {

using Point          = pcl::PointXYZ;
using Particle       = pcl::tracking::ParticleXYZRPY;
using PointCloud     = pcl::PointCloud<Point>;
using ParticleCloud  = pcl::PointCloud<pcl::PointXYZ>;
using ParticleFilter = pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<Point, Particle>;
using SearchMethod   = pcl::search::KdTree<Point>;
using PoseEstReq     = apc16delft_msgs::EstimateObjectPose::Request;
using PoseEstRes     = apc16delft_msgs::EstimateObjectPose::Response;
using BinPoseEstReq  = apc16delft_msgs::EstimateBinPose::Request;
using BinPoseEstRes  = apc16delft_msgs::EstimateBinPose::Response;

protected:

	/// Read necessary parameters from the param server.
	int onConfigure() override {
		// ros param server
		node_handle.param<int>("pf_max_particles", pf_max_particles, 200);
		node_handle.param<int>("pf_min_particles", pf_min_particles, 20);
		node_handle.param<int>("pf_threads", pf_threads, 8);
		node_handle.param<float>("pf_translation_covariance", pf_translation_covariance, 0.005f);
		node_handle.param<float>("pf_rotation_covariance", pf_rotation_covariance, 0.05f);
		node_handle.param<int>("object_pf_max_iterations", object_pf_max_iterations, 20);
		node_handle.param<int>("object_icp_max_iterations", object_icp_max_iterations, 2);

		node_handle.param<float>("subsampling_leaf_size", subsampling_leaf_size, 0.005);

		node_handle.param<bool>("publish_clouds", publish_clouds, true);

		// initialize the particle filter
		particle_filter = ParticleFilter(pf_threads);
		particle_filter.setMaximumParticleNum(pf_max_particles);
		particle_filter.setDelta(0.99);
		particle_filter.setEpsilon(0.2);

		Particle bin_size;
		bin_size.x = bin_size.y = bin_size.z = pf_bin_size;
		bin_size.roll = bin_size.pitch = bin_size.yaw = pf_bin_rotation;

		particle_filter.setBinSize(bin_size);

		// translation / rotation covariance
		std::vector<double> default_step_covariance = std::vector<double>(6, pf_translation_covariance * pf_translation_covariance);
		default_step_covariance[3] = pf_rotation_covariance * pf_rotation_covariance;
		default_step_covariance[4] = pf_rotation_covariance * pf_rotation_covariance;
		default_step_covariance[5] = pf_rotation_covariance * pf_rotation_covariance;

		std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
		std::vector<double> default_initial_mean     = std::vector<double>(6, 0.0);

		particle_filter.setStepNoiseCovariance(default_step_covariance);
		particle_filter.setInitialNoiseCovariance(initial_noise_covariance);
		particle_filter.setInitialNoiseMean(default_initial_mean);

		particle_filter.setParticleNum(pf_min_particles);
		particle_filter.setResampleLikelihoodThr(0.00);
		particle_filter.setUseNormal(false);

		// setup coherence object for tracking
		pcl::tracking::ApproxNearestPairPointCloudCoherence<Point>::Ptr coherence = pcl::tracking::ApproxNearestPairPointCloudCoherence<Point>::Ptr
			(new pcl::tracking::ApproxNearestPairPointCloudCoherence<Point>());

		boost::shared_ptr<pcl::tracking::DistanceCoherence<Point>> distance_coherence
			= boost::shared_ptr<pcl::tracking::DistanceCoherence<Point>>(new pcl::tracking::DistanceCoherence<Point>());
		coherence->addPointCoherence(distance_coherence);

		boost::shared_ptr<pcl::search::Octree<Point>> search(new pcl::search::Octree<Point>(0.01));
		coherence->setSearchMethod(search);
		coherence->setMaximumDistance(0.01);

		particle_filter.setCloudCoherence(coherence);
		return 0;
	}

	int onActivate() override {
		// service servers
		estimate_object_pose_server = node_handle.advertiseService("estimate_object_pose", &PoseEstimationNode::estimateObjectPose, this);

		// publishers
		particle_publisher              = node_handle.advertise<PointCloud>("particles", 1, true);
		transformed_model_pf_publisher  = node_handle.advertise<PointCloud>("transformed_model_pf", 1, true);
		transformed_model_icp_publisher = node_handle.advertise<PointCloud>("transformed_model_icp", 1, true);
		object_cloud_publisher          = node_handle.advertise<PointCloud>("object_cloud", 1, true);
		initial_guess_publisher         = node_handle.advertise<geometry_msgs::PoseStamped>("initial_guess", 1, true);

		return 0;
	}

	int onDeactivate() override {
		// deactivate services
		estimate_object_pose_server.shutdown();

		// deactivate publishers
		particle_publisher.shutdown();
		transformed_model_pf_publisher.shutdown();
		transformed_model_icp_publisher.shutdown();
		object_cloud_publisher.shutdown();
		initial_guess_publisher.shutdown();
		return 0;
	}

	/// Get a point cloud with particle information.
	ParticleCloud::Ptr getParticles(pcl::PCLHeader const & header) {
		// get the particles from the particle_filter object
		ParticleFilter::PointCloudStatePtr particles = particle_filter.getParticles();
		if (!particles) {
			return nullptr;
		}

		// construct a normal pcl::PointCloud object
		ParticleCloud::Ptr particle_cloud(new ParticleCloud);
		particle_cloud->header = header;
		for (Particle const & p: particles->points) {
			particle_cloud->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
		}

		return particle_cloud;
	}

	/// Segment the object from the scene by taking points near the transformed model.
	PointCloud::Ptr getSegmentedObject(PointCloud::Ptr transformed_model, PointCloud::Ptr scene, double dist = 0.01) {
		// initialize a Kd tree for searching
		SearchMethod search;
		search.setInputCloud(scene);

		PointCloud::Ptr segmented_cloud(new PointCloud);
		segmented_cloud->header = scene->header;

		std::vector<int> nn_indices;
		std::vector<float> nn_dists;
		std::set<int> indices;

		// for each point, find the closest point in the other pointcloud
		for (Point p: transformed_model->points) {
			search.radiusSearch(p, dist, nn_indices, nn_dists);
			for (int i: nn_indices)
				indices.insert(i);
		}

		// add nearby indices to the segmented cloud
		for (int i: indices) {
			segmented_cloud->points.push_back(scene->points[i]);
		}

		return segmented_cloud;
	}

	/// Estimate the objects pose using the particle filter algorithm.
	Eigen::Isometry3d estimatePoseParticleFilter(
		PointCloud::Ptr model,          ///< PointCloud of the current model.
		PointCloud::Ptr scene,          ///< PointCloud of the scene (or cutout of the scene).
		int pf_max_iterations,          ///< Maximum number of particle filter iterations to run.
		Eigen::Isometry3d initial_guess ///< Transform (for model to scene) containing the initial guess of the pose.
	) {
		// Set number of iterations
		particle_filter.setIterationNum(pf_max_iterations);

		// set the initial guess
		particle_filter.setTrans(initial_guess.cast<float>());

		// set the model
		particle_filter.setReferenceCloud(model);

		// set input cloud
		particle_filter.setInputCloud(scene);

		// estimate pose using particle filter
		particle_filter.compute();

		// return result
		return Eigen::Isometry3d(particle_filter.toEigenMatrix(particle_filter.getResult()).cast<double>().matrix());
	}

	/// Estimate the objects pose using the iterative closest point algorithm.
	Eigen::Isometry3d estimatePoseICP(
		PointCloud::Ptr model,                   ///< PointCloud of the current model.
		PointCloud::Ptr scene,                   ///< PointCloud of the scene (or cutout of the scene).
		int icp_max_iterations,                  ///< Maximum number of ICP iterations to run.
		Eigen::Isometry3d const & initial_guess, ///< Transform (for model to scene) containing the initial guess of the pose.
		double & fitness_score                   ///< Fitness score for pose (average squared distance from source to target).
	) {
		// initialize an ICP object.
		pcl::IterativeClosestPoint<Point, Point> icp;
		icp.setMaximumIterations(icp_max_iterations);

		// set input clouds
		icp.setInputSource(scene);
		icp.setInputTarget(model);

		// compute alignment
		PointCloud::Ptr refined_cloud(new PointCloud);
		icp.align(*refined_cloud, initial_guess.inverse().cast<float>().matrix());

		// compute the fitness score (average squared distances from source to target).
		fitness_score = icp.getFitnessScore();

		// return result
		return Eigen::Isometry3d(icp.getFinalTransformation().inverse().cast<double>().matrix());
	}

	/// Estimating the pose of an object
	bool estimatePose(
		PointCloud::Ptr model,
		PointCloud::Ptr scene,
		int pf_max_iterations,
		int icp_max_iterations,
		Eigen::Isometry3d const & initial_guess,
		double & estimation_error,
		Eigen::Isometry3d & pose
	) {
		if (scene->empty()) {
			ROS_ERROR_STREAM("Scene point cloud is empty.");
			return false;
		}

		if (model->empty()) {
			ROS_ERROR_STREAM("Model point cloud is empty.");
			return false;
		}

		// set same header as input
		model->header = scene->header;

		// estimate pose using particle filter
		Eigen::Isometry3d pose_particle_filter = estimatePoseParticleFilter(model, scene, pf_max_iterations, initial_guess);

		// transform model using particle filter estimate
		PointCloud::Ptr transformed_model_pf(new PointCloud);
		pcl::transformPointCloud(*model, *transformed_model_pf, Eigen::Affine3f(pose_particle_filter.cast<float>()));

		// segment the scene point cloud
		PointCloud::Ptr segmented_scene = getSegmentedObject(transformed_model_pf, scene);

		// estimate pose using icp
		pose = Eigen::Isometry3d(estimatePoseICP(model, segmented_scene, icp_max_iterations, pose_particle_filter, estimation_error));

		// publish point cloud information
		if (publish_clouds) {
			// transform model using particle filter
			PointCloud::Ptr transformed_model_icp(new PointCloud);
			pcl::transformPointCloud(*model, *transformed_model_icp, pose.cast<float>());

			particle_publisher.publish(getParticles(scene->header));
			transformed_model_pf_publisher.publish(transformed_model_pf);
			transformed_model_icp_publisher.publish(transformed_model_icp);
			object_cloud_publisher.publish(scene);
		}

		return true;
	}

	/// Service call for estimating the pose of an object.
	bool estimateObjectPose(PoseEstReq & req, PoseEstRes & res) {
		// convert sensor_msgs/PointCloud2 to pcl::PointCloud
		PointCloud::Ptr scene(new PointCloud);
		pcl::fromROSMsg(req.object.point_cloud, *scene);

		PointCloud::Ptr scene_subsampled(new PointCloud);
		pcl::VoxelGrid<Point> sor;
		sor.setInputCloud(scene);
		sor.setLeafSize(subsampling_leaf_size, subsampling_leaf_size, subsampling_leaf_size);
		sor.filter(*scene_subsampled);

		// load model from database
		// TODO: Cache ?
		std::string model_path;
		if (!node_handle.getParam("/item/" + apc16delft_msgs::objectTypeToString(req.object.object.type) + "/model_path", model_path)) {
			ROS_ERROR_STREAM("Failed to read model path for object: " << apc16delft_msgs::objectTypeToString(req.object.object.type));
			return false;
		}

		// resolve package://* to full path
		model_path = resolvePackagePath(model_path);

		// read model
		PointCloud::Ptr model(new PointCloud);
		if (pcl::io::loadPCDFile(model_path, *model) == -1) {
			ROS_ERROR_STREAM("No model found. Model path: " << model_path);
			return false;
		}

		// check for empty model
		if (model->empty()) {
			ROS_ERROR_STREAM("Model point cloud is empty.");
			return false;
		}

		// set initial guess
		Eigen::Isometry3d initial_guess =
			Eigen::Translation3d(req.object.centroid.x, req.object.centroid.y, req.object.centroid.z) *
			Eigen::AngleAxisd(105.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
			Eigen::AngleAxisd(M_PI,  Eigen::Vector3d::UnitZ());

		geometry_msgs::PoseStamped initial_guess_msg = dr::toRosPoseStamped(initial_guess, req.object.point_cloud.header.frame_id, req.object.point_cloud.header.stamp);
		initial_guess_publisher.publish(initial_guess_msg);

		ROS_INFO_STREAM("Starting Pose Estimation");
		// perform pose estimation
		double estimation_error;
		Eigen::Isometry3d pose;
		if(!estimatePose(model, scene_subsampled, object_pf_max_iterations, object_icp_max_iterations, initial_guess, estimation_error, pose)) {
			return false;
		}
		ROS_INFO_STREAM("Finalizing Pose Estimation");

		// copy output
		res.estimation_error = estimation_error;
		res.pose.pose        = dr::toRosPose(pose);
		res.pose.header      = req.object.point_cloud.header;
		return true;
	}

	/// Service server for estimating the pose of an object.
	ros::ServiceServer estimate_object_pose_server;

	/// Publisher for viewing the current particles in the particle filter tracker.
	ros::Publisher particle_publisher;

	/// Publisher for viewing the transformed model after particle filter pose estimation.
	ros::Publisher transformed_model_pf_publisher;

	/// Publisher for viewing the transformed model after the ICP pose estimation refinement.
	ros::Publisher transformed_model_icp_publisher;

	/// Publisher for viewing the original input point cloud.
	ros::Publisher object_cloud_publisher;

	/// Publisher for viewing initial guess.
	ros::Publisher initial_guess_publisher;

	/// Maximum number of particles to use in particle filter.
	int pf_max_particles;

	/// Minimum number of particles to use in particle filter.
	int pf_min_particles;

	/// Number of particle filter threads to use (usually 8 for modern CPU's).
	int pf_threads;

	/// The bin size with which the particle filter initializes new particles.
	float pf_bin_size;

	/// The bin rotation with which the particle filter initializes new particles.
	float pf_bin_rotation;

	/// Translation covariance (measured in m) which the particle filter uses to add noise to newly spawned particles.
	float pf_translation_covariance;

	/// Rotation covariance (measured in radian) which the particle filter uses to add noise to newly spawned particles.
	float pf_rotation_covariance;

	/// Maximum number of iterations for objects using particle filter
	int object_pf_max_iterations;

	/// Maximum number of iterations for objects using ICP
	int object_icp_max_iterations;

	/// Subsampling leaf size: Effectively approximate distance between points after subsampling
	float subsampling_leaf_size;

	/// Determines if this node publishes point clouds for debugging.
	bool publish_clouds;

	/// The particle filter object.
	ParticleFilter particle_filter;

};

}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "pose_estimation");
	apc16delft::PoseEstimationNode node;
	ros::spin();
}

