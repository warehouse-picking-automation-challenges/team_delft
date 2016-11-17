#include <ros/ros.h>
#include <apc16delft_util/managed_node.hpp>
#include <apc16delft_msgs/GetCameraData.h>
#include <apc16delft_msgs/GetPose.h>
#include <apc16delft_msgs/InitializeCalibration.h>
#include <apc16delft_msgs/RecordCalibration.h>
#include <apc16delft_msgs/FinalizeCalibration.h>

#include <dr_eigen/ros.hpp>
#include <dr_eigen/param.hpp>
#include <dr_param/param.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

namespace apc16delft {

class CameraNode : ManagedNode {
protected:

	/// Read in parameters from the ROS parameter server and initialize objects.
	int onConfigure() override {
		// load other parameters
		node_handle.param<std::string>("camera_frame", camera_frame, "camera_optical_frame");
		pattern_pose = dr::getParam<Eigen::Isometry3d>(node_handle, "pattern_pose");
		camera_pose = dr::getParam<Eigen::Isometry3d>(node_handle, "camera_pose");

		resetCalibration();

		return 0;
	}

	/// Activate ROS service servers and publishers
	int onActivate() override {
		// activate service servers
		camera_data_server            = node_handle.advertiseService("get_data"              , &CameraNode::getData              , this);
		dump_data_server              = node_handle.advertiseService("dump_data"             , &CameraNode::dumpData             , this);
		get_pattern_pose_server       = node_handle.advertiseService("get_pattern_pose"      , &CameraNode::getPatternPose       , this);
		initialize_calibration_server = node_handle.advertiseService("initialize_calibration", &CameraNode::initializeCalibration, this);
		record_calibration_server     = node_handle.advertiseService("record_calibration"    , &CameraNode::recordCalibration    , this);
		finalize_calibration_server   = node_handle.advertiseService("finalize_calibration"  , &CameraNode::finalizeCalibration  , this);
		clear_camera_pose_server      = node_handle.advertiseService("clear_camera_pose"     , &CameraNode::clearCameraPose      , this);

		return 0;
	}

	int onDeactivate() override {
		// deactivate service servers
		camera_data_server.shutdown();
		dump_data_server.shutdown();
		get_pattern_pose_server.shutdown();
		initialize_calibration_server.shutdown();
		record_calibration_server.shutdown();
		finalize_calibration_server.shutdown();
		clear_camera_pose_server.shutdown();
		return 0;
	}

	/// Resets calibration state from this node.
	void resetCalibration() {
		moving_frame  = "";
		static_frame  = "";
	}

	bool getData(apc16delft_msgs::GetCameraData::Request & req, apc16delft_msgs::GetCameraData::Response & res) {
		// read image file path
		std::string image_file;
		if (!node_handle.getParam("image", image_file)) {
			ROS_ERROR_STREAM("Failed to read image file path from parameter server.");
			return false;
		}

		// read point cloud file path
		std::string point_cloud_file;
		if (!node_handle.getParam("point_cloud", point_cloud_file)) {
			ROS_ERROR_STREAM("Failed to read point cloud file path from parameter server.");
			return false;
		}

		// load image
		image = cv::imread(image_file);
		if (image.empty()) {
			ROS_ERROR_STREAM("Failed to load image from path: " << image_file);
			return false;
		}

		// load point cloud
		if (pcl::io::loadPCDFile(point_cloud_file, point_cloud) == -1) {
			ROS_ERROR_STREAM("Failed to load point cloud from path: " << point_cloud_file);
			return false;
		}

		
		// check for existing data
		if (image.empty() || point_cloud.empty()) {
			ROS_ERROR_STREAM("Fake camera node not properly configured.");
			return false;
		}

		// copy the image
		std_msgs::Header header;
		header.frame_id = camera_frame;
		header.stamp = ros::Time::now();
		cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, image);
		res.color = *cv_image.toImageMsg();

		// copy the point cloud
		pcl::toROSMsg(point_cloud, res.point_cloud);
		res.point_cloud.header = header;

		if (req.dump) {
			ROS_WARN_STREAM("Data dumping is disabled for fake camera.");
		}

		return true;
	}

	bool dumpData(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		ROS_WARN_STREAM("Data dumping is disabled for fake camera.");
		return true;
	}

	bool getPatternPose(apc16delft_msgs::GetPose::Request &, apc16delft_msgs::GetPose::Response & res) {
		// retrieve pattern pose and return as geometry_msgs/PoseStamped
		res.data = dr::toRosPose(pattern_pose);
		return true;
	}

	bool initializeCalibration(apc16delft_msgs::InitializeCalibration::Request & req, apc16delft_msgs::InitializeCalibration::Response &) {
		camera_moving = req.camera_moving;
		moving_frame  = req.moving_frame;
		static_frame  = req.static_frame;
		return true;
	}

	bool recordCalibration(apc16delft_msgs::RecordCalibration::Request &, apc16delft_msgs::RecordCalibration::Response &) {
		ROS_WARN_STREAM("Calibration recording disabled for fake camera.");
		return true;
	}

	bool finalizeCalibration(apc16delft_msgs::FinalizeCalibration::Request &, apc16delft_msgs::FinalizeCalibration::Response & res) {
		// check for proper initialization
		if (moving_frame == "" || static_frame == "") {
			ROS_ERROR_STREAM("No calibration frame provided.");
			return false;
		}

		res.pattern_pose       = dr::toRosPoseStamped(pattern_pose, camera_moving ? static_frame : moving_frame);
		res.camera_pose        = dr::toRosPoseStamped(camera_pose, camera_moving ? moving_frame : static_frame);
		res.iterations         = 1;
		res.reprojection_error = 0;

		resetCalibration();
		return true;
	}

	bool clearCameraPose(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		ROS_WARN_STREAM("Clearing of camera pose disabled for fake camera.");
		return true;
	}

	/// Service server for supplying point clouds and images.
	ros::ServiceServer camera_data_server;

	/// Service server for dumping image and cloud to disk.
	ros::ServiceServer dump_data_server;

	/// Service server for retrieving the pose of the pattern.
	ros::ServiceServer get_pattern_pose_server;

	/// Service server for initializing the calibration sequence.
	ros::ServiceServer initialize_calibration_server;

	/// Service server for recording one calibration sample.
	ros::ServiceServer record_calibration_server;

	/// Service server for finalizing the calibration.
	ros::ServiceServer finalize_calibration_server;

	/// Service server for clearing the camera pose setting of the Ensenso.
	ros::ServiceServer clear_camera_pose_server;

	/// Buffered image.
	cv::Mat image;

	/// Buffered point cloud.
	pcl::PointCloud<pcl::PointXYZ> point_cloud;

	/// Buffered pattern pose.
	Eigen::Isometry3d pattern_pose;

	/// Buffered camera pose.
	Eigen::Isometry3d camera_pose;

	/// Name for the camera frame.
	std::string camera_frame;

	/// Frame to calibrate the camera to when camera_moving is true (gripper frame).
	std::string moving_frame;

	/// Frame to calibrate the camera to when camera_moving is false (robot origin or world frame).
	std::string static_frame;

	/// Used in calibration. Determines if the camera is moving (eye in hand) or static.
	bool camera_moving;
};

}

int main(int argc, char ** argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME "_camera");
	apc16delft::CameraNode node;
	ros::spin();
}
