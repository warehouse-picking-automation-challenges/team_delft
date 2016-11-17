#include <apc16delft_util/managed_node.hpp>
#include <apc16delft_util/timestamp.hpp>
#include <apc16delft_msgs/GetCameraData.h>
#include <apc16delft_msgs/InitializeCalibration.h>
#include <apc16delft_msgs/RecordCalibration.h>
#include <apc16delft_msgs/FinalizeCalibration.h>
#include <apc16delft_msgs/GetPose.h>

#include <dr_ensenso/ensenso.hpp>
#include <dr_eigen/ros.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

namespace apc16delft {

// this is needed because std::make_unique only exists from c++14
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
	return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

class CameraNode : public ManagedNode {
public:
	CameraNode() : ManagedNode(), image_transport(node_handle) {}

private:
	/// Resets calibration state from this node.
	void resetCalibration() {
		moving_frame  = "";
		static_frame  = "";
		camera_guess  = boost::none;
		pattern_guess = boost::none;
		robot_poses.clear();
	}

protected:
	using Point = pcl::PointXYZ;
	using PointCloud = pcl::PointCloud<Point>;

	int onConfigure() override {
		// load ROS parameters
		node_handle.param<std::string>("camera_frame", camera_frame, "camera_frame");
		node_handle.param<std::string>("camera_data_path", camera_data_path, "camera_data");
		node_handle.param<bool>("publish_images", publish_images, true);

		serial = "";
		if (node_handle.getParam("serial", serial)) {
			ROS_INFO_STREAM("Opening Ensenso with serial '" << serial << "'...");
		} else {
			ROS_WARN_STREAM("No Ensenso serial found.");
		}

		try {
			// create the camera
			ensenso_camera = make_unique<dr::Ensenso>(serial);
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed initializing camera. " << e.what());
			return 1;
		} catch (std::runtime_error const & e) {
			ROS_ERROR_STREAM("Failed initializing camera. " << e.what());
			return 2;
		}

		return 0;
	}

	/// Activate ROS service servers and publishers
	int onActivate() override {
		// activate service servers
		servers.camera_data            = node_handle.advertiseService("get_data"              , &CameraNode::getData              , this);
		servers.dump_data              = node_handle.advertiseService("dump_data"             , &CameraNode::dumpData             , this);
		servers.get_pattern_pose       = node_handle.advertiseService("get_pattern_pose"      , &CameraNode::getPatternPose       , this);
		servers.initialize_calibration = node_handle.advertiseService("initialize_calibration", &CameraNode::initializeCalibration, this);
		servers.record_calibration     = node_handle.advertiseService("record_calibration"    , &CameraNode::recordCalibration    , this);
		servers.finalize_calibration   = node_handle.advertiseService("finalize_calibration"  , &CameraNode::finalizeCalibration  , this);
		servers.clear_workspace        = node_handle.advertiseService("clear_workspace"       , &CameraNode::clearWorkspace       , this);

		// activate publishers
		publishers.calibration = node_handle.advertise<geometry_msgs::PoseStamped>("calibration", 1, true);
		publishers.cloud       = node_handle.advertise<PointCloud>("cloud", 1, true);
		publishers.image       = image_transport.advertise("image", 1, true);

		// load ensenso parameters file
		std::string ensenso_param_file;
		if (node_handle.getParam("ensenso_param_file", ensenso_param_file)) {
			try {
				if (!ensenso_camera->loadParameters(ensenso_param_file)) {
					ROS_ERROR_STREAM("Failed to set Ensenso params. File path: " << ensenso_param_file);
					return 1;
				}
			} catch (dr::NxError const & e) {
				ROS_ERROR_STREAM("Failed to set Ensenso params. " << e.what());
				return 1;
			}
		}

		// load overlay parameters file
		std::string overlay_param_file;
		if (node_handle.getParam("overlay_param_file", overlay_param_file)) {
			try {
				if (!ensenso_camera->loadOverlayParameters(overlay_param_file)) {
					ROS_ERROR_STREAM("Failed to set overlay camera params. File path: " << overlay_param_file);
					return 2;
				}
			} catch (dr::NxError const & e) {
				ROS_ERROR_STREAM("Failed to set overlay camera params. " << e.what());
				return 2;
			}
		}

		// load overlay parameter set file
		std::string overlay_param_set_file;
		if (node_handle.getParam("overlay_param_set_file", overlay_param_set_file)) {
			try {
				ensenso_camera->loadOverlayParameterSet(overlay_param_set_file);
			} catch (dr::NxError const & e) {
				ROS_ERROR_STREAM("Failed to set overlay param set file. " << e.what());
				return 3;
			}
		}

		// initialize other member variables
		resetCalibration();

		publish_calibration_timer = node_handle.createTimer(ros::Duration(5), &CameraNode::publishCalibration, this);

		// start image publishing timer
		if (publish_images) {
			publish_images_timer = node_handle.createTimer(ros::Rate(30), &CameraNode::publishImage, this);
		}

		return 0;
	}

	int onDeactivate() override {
		// deactivate service servers
		servers.camera_data.shutdown();
		servers.dump_data.shutdown();
		servers.get_pattern_pose.shutdown();
		servers.initialize_calibration.shutdown();
		servers.record_calibration.shutdown();
		servers.finalize_calibration.shutdown();
		servers.clear_workspace.shutdown();

		// deactivate publishers
		publishers.calibration.shutdown();
		publishers.cloud.shutdown();
		publishers.image.shutdown();

		// deactivate timers
		publish_calibration_timer.stop();
		publish_images_timer.stop();

		// release the camera
		ensenso_camera.release();

		return 0;
	}

	void publishImage(ros::TimerEvent const &) {
		// create a header
		std_msgs::Header header;
		header.frame_id = camera_frame;
		header.stamp    = ros::Time::now();

		// prepare message
		cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, cv::Mat());

		// read the image
		try {
			ensenso_camera->loadIntensity(cv_image.image);
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to retrieve image. " << e.what());
			return;
		}

		// publish the image
		publishers.image.publish(cv_image.toImageMsg());
	}

	PointCloud::Ptr getPointCloud() {
		PointCloud::Ptr cloud(new PointCloud);
		try {
			ensenso_camera->loadRegisteredPointCloud(*cloud);
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to retrieve PointCloud. " << e.what());
			return nullptr;
		}
		cloud->header.frame_id = camera_frame;
		return cloud;
	}

	void dumpData(cv::Mat const & image, PointCloud::Ptr point_cloud) {
		// create path if it does not exist
		boost::filesystem::path path(camera_data_path);
		if (!boost::filesystem::is_directory(path)) {
			boost::filesystem::create_directory(camera_data_path);
		}

		std::string time_string = getTimeString();

		pcl::io::savePCDFile(camera_data_path + "/" + time_string + "_cloud.pcd", *point_cloud);
		cv::imwrite(camera_data_path + "/" + time_string + "_image.png", image);
	}

	bool getData(apc16delft_msgs::GetCameraData::Request & req, apc16delft_msgs::GetCameraData::Response & res) {
		// prepare message
		cv_bridge::CvImage cv_image(res.point_cloud.header, sensor_msgs::image_encodings::BGR8, cv::Mat());

		// read the image
		try {
			ensenso_camera->loadIntensity(cv_image.image);
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to retrieve image. " << e.what());
			return false;
		}
		res.color = *cv_image.toImageMsg();

		// read the point cloud
		PointCloud::Ptr point_cloud = getPointCloud();

		// convert to ROS message
		pcl::toROSMsg(*point_cloud, res.point_cloud);

		// store image and point cloud
		if (req.dump) {
			dumpData(cv_image.image, point_cloud);
		}

		// publish point cloud if requested
		if (req.publish) {
			publishers.cloud.publish(point_cloud);
		}

		return true;
	}

	bool dumpData(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		// get image
		cv::Mat image;
		try {
			ensenso_camera->loadIntensity(image);
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to retrieve image. " << e.what());
			return false;
		}
		cv::imwrite(getTimeString() + "_image.png", image);

		// dump the image and point cloud
		dumpData(image, getPointCloud());

		return true;
	}

	bool getPatternPose(apc16delft_msgs::GetPose::Request &, apc16delft_msgs::GetPose::Response & res) {
		try {
			Eigen::Isometry3d pattern_pose;

			if (!ensenso_camera->calibrate(1, pattern_pose)) {
				ROS_ERROR_STREAM("Failed to calibrate.");
				return false;
			}

			res.data = dr::toRosPose(pattern_pose);
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to find calibration pattern. " << e.what());
			return false;
		}

		return true;
	}

	bool initializeCalibration(apc16delft_msgs::InitializeCalibration::Request & req, apc16delft_msgs::InitializeCalibration::Response &) {
		try {
			ensenso_camera->discardPatterns();
			ensenso_camera->clearWorkspace();
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to discard patterns. " << e.what());
			return false;
		}
		resetCalibration();

		camera_moving = req.camera_moving;
		moving_frame  = req.moving_frame;
		static_frame  = req.static_frame;

		// check for valid camera guess
		if (req.camera_guess.position.x == 0 && req.camera_guess.position.y == 0 && req.camera_guess.position.z == 0 &&
			req.camera_guess.orientation.x == 0 && req.camera_guess.orientation.y == 0 && req.camera_guess.orientation.z == 0 && req.camera_guess.orientation.w == 0) {
			camera_guess = boost::none;
		} else {
			camera_guess = dr::toEigen(req.camera_guess);
		}

		// check for valid pattern guess
		if (req.pattern_guess.position.x == 0 && req.pattern_guess.position.y == 0 && req.pattern_guess.position.z == 0 &&
			req.pattern_guess.orientation.x == 0 && req.pattern_guess.orientation.y == 0 && req.pattern_guess.orientation.z == 0 && req.pattern_guess.orientation.w == 0) {
			pattern_guess = boost::none;
		} else {
			pattern_guess = dr::toEigen(req.pattern_guess);
		}

		// check for proper initialization
		if (moving_frame == "" || static_frame == "") {
			ROS_ERROR_STREAM("No calibration frame provided.");
			return false;
		}

		ROS_INFO_STREAM("Successfully initialized calibration sequence.");

		return true;
	}

	bool recordCalibration(apc16delft_msgs::RecordCalibration::Request & req, apc16delft_msgs::RecordCalibration::Response &) {
		// check for proper initialization
		if (moving_frame == "" || static_frame == "") {
			ROS_ERROR_STREAM("No calibration frame provided.");
			return false;
		}

		try {
			// record a pattern
			ensenso_camera->recordCalibrationPattern();

			// add robot pose to list of poses
			robot_poses.push_back(dr::toEigen(req.pose));
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to record calibration pattern. " << e.what());
			return false;
		}

		ROS_INFO_STREAM("Successfully recorded a calibration sample.");
		return true;
	}

	bool finalizeCalibration(apc16delft_msgs::FinalizeCalibration::Request &, apc16delft_msgs::FinalizeCalibration::Response & res) {
		// check for proper initialization
		if (moving_frame == "" || static_frame == "") {
			ROS_ERROR_STREAM("No calibration frame provided.");
			return false;
		}

		try {
			// perform calibration
			dr::Ensenso::CalibrationResult calibration =
				ensenso_camera->computeCalibration(robot_poses, camera_moving, camera_guess, pattern_guess, camera_moving ? moving_frame : static_frame);

			// copy result
			res.camera_pose        = dr::toRosPoseStamped(std::get<0>(calibration), camera_moving ? moving_frame : static_frame);
			res.pattern_pose       = dr::toRosPoseStamped(std::get<1>(calibration), camera_moving ? static_frame : moving_frame);
			res.iterations         = std::get<2>(calibration);
			res.reprojection_error = std::get<3>(calibration);

			// store result in camera
			ensenso_camera->storeCalibration();

			// clear state
			resetCalibration();
		} catch (dr::NxError const & e) {
			// clear state (?)
			resetCalibration();

			ROS_ERROR_STREAM("Failed to finalize calibration. " << e.what());
			return false;
		}

		ROS_INFO_STREAM("Successfully finished calibration sequence.");

		publishCalibration();
		return true;
	}

	bool clearWorkspace(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
		try {
			ensenso_camera->clearWorkspace();
		} catch (dr::NxError const & e) {
			ROS_ERROR_STREAM("Failed to clear camera pose. " << e.what());
			return false;
		}
		return true;
	}

	void publishCalibration(ros::TimerEvent const & = {}) {
		geometry_msgs::PoseStamped pose;
		boost::optional<std::string> frame = ensenso_camera->getFrame();
		if (frame) {
			pose = dr::toRosPoseStamped(ensenso_camera->getCameraPose()->inverse(), *frame, ros::Time::now());
		} else {
			pose = dr::toRosPoseStamped(Eigen::Isometry3d::Identity(), "", ros::Time::now());
		}

		publishers.calibration.publish(pose);
	}

	/// The wrapper for the Ensenso stereo camera.
	std::unique_ptr<dr::Ensenso> ensenso_camera;

	struct {
		/// Service server for supplying point clouds and images.
		ros::ServiceServer camera_data;

		/// Service server for dumping image and cloud to disk.
		ros::ServiceServer dump_data;

		/// Service server for retrieving the pose of the pattern.
		ros::ServiceServer get_pattern_pose;

		/// Service server for initializing the calibration sequence.
		ros::ServiceServer initialize_calibration;

		/// Service server for recording one calibration sample.
		ros::ServiceServer record_calibration;

		/// Service server for finalizing the calibration.
		ros::ServiceServer finalize_calibration;

		/// Service server for clearing the camera pose setting of the Ensenso.
		ros::ServiceServer clear_workspace;
	} servers;

	/// Object for handling transportation of images.
	image_transport::ImageTransport image_transport;

	/// Timer to trigger calibration publishing.
	ros::Timer publish_calibration_timer;

	/// Timer to trigger image publishing.
	ros::Timer publish_images_timer;

	struct Publishers {
		/// Publisher for the calibration result.
		ros::Publisher calibration;

		/// Publisher for publishing raw point clouds.
		ros::Publisher cloud;

		/// Publisher for publishing images.
		image_transport::Publisher image;
	} publishers;

	/// The calibrated pose of the camera.
	geometry_msgs::PoseStamped camera_pose;

	/// The frame in which the image and point clouds are send.
	std::string camera_frame;

	/// Frame to calibrate the camera to when camera_moving is true (gripper frame).
	std::string moving_frame;

	/// Frame to calibrate the camera to when camera_moving is false (robot origin or world frame).
	std::string static_frame;

	/// Serial id of the Ensenso camera.
	std::string serial;

	/// If true, publishes images with a frequency of 30Hz.
	bool publish_images;

	// Guess of the camera pose relative to gripper (for moving camera) or relative to robot origin (for static camera).
	boost::optional<Eigen::Isometry3d> camera_guess;

	// Guess of the calibration pattern pose relative to gripper (for static camera) or relative to robot origin (for moving camera).
	boost::optional<Eigen::Isometry3d> pattern_guess;

	/// Used in calibration. Determines if the camera is moving (eye in hand) or static.
	bool camera_moving;

	/// List of robot poses corresponding to the list of recorded calibration patterns.
	std::vector<Eigen::Isometry3d> robot_poses;

	/// Location where the images and point clouds are stored.
	std::string camera_data_path;
};

}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "ensenso_camera");
	apc16delft::CameraNode node;
	ros::spin();
}

