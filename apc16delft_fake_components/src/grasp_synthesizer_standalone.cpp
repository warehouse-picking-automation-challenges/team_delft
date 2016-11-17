#include <ros/ros.h>
#include <apc16delft_msgs/GetCameraData.h>
#include <apc16delft_msgs/DetectObjects.h>
#include <apc16delft_msgs/objects.hpp>
#include <apc16delft_msgs/SynthesizeGrasp.h>

#include <dr_eigen/ros.hpp>

int main(int argc, char ** argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME "_vision_standalone");

	ros::NodeHandle node_handle("~");

	// construct service clients
	ros::ServiceClient camera_data_client    = node_handle.serviceClient<apc16delft_msgs::GetCameraData>("/gripper_camera/get_data");
	ros::ServiceClient detect_objects_client = node_handle.serviceClient<apc16delft_msgs::DetectObjects>("/object_detection_server/detect_objects");
	ros::ServiceClient grasp_synthesizer_client = node_handle.serviceClient<apc16delft_msgs::SynthesizeGrasp>("/grasp_synthesizer/synthesize_grasp");

	ros::Publisher cloud_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("/cropped_cloud", 1, true);

	// request camera data
	apc16delft_msgs::GetCameraData camera_srv;
	camera_srv.request.dump = false;
	camera_srv.request.publish = false;
	if (!camera_data_client.call(camera_srv)) {
		ROS_ERROR_STREAM("Failed to get camera data.");
		return 0;
	}

	Eigen::Isometry3d bin_pose = Eigen::Translation3d(0.01, 0.02, 0.03) * Eigen::AngleAxisd(0.001, dr::axes::x());
	Eigen::Vector3d bin_dimensions{10, 10, 10}; // insanely large

	// set up detect objects request
	apc16delft_msgs::DetectObjects objects_srv;
	objects_srv.request.color          = camera_srv.response.color;
	objects_srv.request.point_cloud    = camera_srv.response.point_cloud;
	objects_srv.request.bin_pose       = dr::toRosPose(bin_pose);
	//objects_srv.request.bin_dimensions = dr::toRosVector3(bin_dimensions);

	// push all items in expected list
	for (size_t i = 1; i <= 39; i++) {
		objects_srv.request.objects.push_back(i);
	}

	// detect objects
	if (!detect_objects_client.call(objects_srv)) {
		ROS_ERROR_STREAM("Failed to find objects.");
		return 0;
	}

	for (auto const & object : objects_srv.response.objects) {
		ROS_INFO_STREAM("Found object (" << apc16delft_msgs::objectTypeToString(object.object.type) << ") with score : " << object.score);
	}

	cloud_publisher.publish(objects_srv.response.objects[0].point_cloud);
	ROS_INFO_STREAM("point cloud header: " << objects_srv.response.objects[0].point_cloud.header);

	apc16delft_msgs::SynthesizeGrasp synth_grasps;
	synth_grasps.request.object = objects_srv.response.objects[0].object;

	grasp_synthesizer_client.call(synth_grasps);
}
