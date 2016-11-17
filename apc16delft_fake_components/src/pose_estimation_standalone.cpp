#include <apc16delft_msgs/GetCameraData.h>
#include <apc16delft_msgs/EstimateObjectPose.h>
#include <apc16delft_msgs/DetectObjects.h>
#include <apc16delft_msgs/objects.hpp>

#include <dr_eigen/ros.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

int main(int argc, char ** argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME "_pose_estimation_standalone");

	ros::NodeHandle node_handle("~");

	// construct service clients
	ros::ServiceClient camera_data_client     = node_handle.serviceClient<apc16delft_msgs::GetCameraData>("/gripper_camera/get_data");
	ros::ServiceClient detect_objects_client  = node_handle.serviceClient<apc16delft_msgs::DetectObjects>("/object_detection_server/detect_objects");
	ros::ServiceClient pose_estimation_client = node_handle.serviceClient<apc16delft_msgs::EstimateObjectPose>("/pose_estimation/estimate_object_pose");

	// request camera data
	apc16delft_msgs::GetCameraData camera_srv;
	camera_srv.request.dump = false;
	if (!camera_data_client.call(camera_srv)) {
		ROS_ERROR_STREAM("Failed to get camera data.");
		return 0;
	}

	apc16delft_msgs::DetectObjects objects_srv;
	objects_srv.request.point_cloud = camera_srv.response.point_cloud;
	objects_srv.request.color = camera_srv.response.color;

	// push all items in expected list
	for (size_t i = 1; i <= 39; i++) {
		objects_srv.request.objects.push_back(i);
	}

	if (!detect_objects_client.call(objects_srv)) {
		ROS_ERROR_STREAM("Failed to detect objects.");
		return 0;
	}

	if (objects_srv.response.objects.size() == 0) {
		ROS_ERROR_STREAM("No objects detected.");
		return 0;
	}

	// set up detect objects request
	apc16delft_msgs::EstimateObjectPose pose_estimate_srv;
	pose_estimate_srv.request.object = objects_srv.response.objects[0];

	// estimate pose
	if (!pose_estimation_client.call(pose_estimate_srv)) {
		ROS_ERROR_STREAM("Failed to find pose.");
		return 0;
	}

	ROS_INFO_STREAM("object pose: " << pose_estimate_srv.response.pose);
	ROS_INFO_STREAM("estimation error: " << pose_estimate_srv.response.estimation_error);
	ROS_INFO_STREAM("model match: " << pose_estimate_srv.response.model_match);

	return 0;
}
