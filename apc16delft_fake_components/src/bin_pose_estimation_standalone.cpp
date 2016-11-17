#include <apc16delft_msgs/GetCameraData.h>
#include <apc16delft_msgs/EstimateBinPose.h>
#include <apc16delft_msgs/DetectObjects.h>
#include <apc16delft_msgs/objects.hpp>

#include <apc16delft_util/util.hpp>

#include <dr_eigen/ros.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

int main(int argc, char ** argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME "_bin_pose_estimation_standalone");

	ros::NodeHandle node_handle("~");

	// construct service clients
	ros::ServiceClient camera_data_client     = node_handle.serviceClient<apc16delft_msgs::GetCameraData>("/camera_driver/get_data");
	ros::ServiceClient pose_estimation_client = node_handle.serviceClient<apc16delft_msgs::EstimateBinPose>("/bin_pose_estimation/estimate_bin_pose");

	// request camera data
	apc16delft_msgs::GetCameraData camera_srv;
	camera_srv.request.dump = false;
	if (!camera_data_client.call(camera_srv)) {
		ROS_ERROR_STREAM("Failed to get camera data.");
		return 0;
	}

	// set up pose estimation request
	apc16delft_msgs::EstimateBinPose pose_estimate_srv;
	pose_estimate_srv.request.scene                             = camera_srv.response.point_cloud;
	pose_estimate_srv.request.shelf_initial_guess.position.x    = 0.276428;
	pose_estimate_srv.request.shelf_initial_guess.position.y    = -0.169315;
	pose_estimate_srv.request.shelf_initial_guess.position.z    = 0.554420;
	pose_estimate_srv.request.shelf_initial_guess.orientation.x = 0.638396;
	pose_estimate_srv.request.shelf_initial_guess.orientation.y = 0.611295;
	pose_estimate_srv.request.shelf_initial_guess.orientation.z = 0.337826;
	pose_estimate_srv.request.shelf_initial_guess.orientation.w = 0.323484;
	pose_estimate_srv.request.bin_index                         = int(apc16delft::BinIndex::BIN_C);

	// estimate pose
	if (!pose_estimation_client.call(pose_estimate_srv)) {
		ROS_ERROR_STREAM("Failed to find pose.");
		return 0;
	}

	ROS_INFO_STREAM("object pose: " << pose_estimate_srv.response.pose);

	return 0;
}
