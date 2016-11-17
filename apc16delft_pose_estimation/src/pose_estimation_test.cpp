#include <ros/ros.h>
#include <apc16delft_msgs/EstimateObjectPose.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char ** argv) {
	ros::init(argc, argv, "pose_estimation_test");

	if (argc != 2) {
		std::cerr << "Please provide a scene point cloud file." << std::endl;
		return 0;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile(argv[1], *scene) == -1) {
		PCL_ERROR("No scene found.\n");
		return 0;
	}

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid<pcl::PointXYZRGB>(*scene, centroid);

	ros::NodeHandle node_handle;
	ros::ServiceClient pose_estimation_client = node_handle.serviceClient<apc16delft_msgs::EstimateObjectPose>("/pose_estimation/estimate_pose");

	apc16delft_msgs::EstimateObjectPose srv;
	pcl::toROSMsg(*scene, srv.request.object.point_cloud);
	srv.request.object.centroid.x = centroid.x();
	srv.request.object.centroid.y = centroid.y();
	srv.request.object.centroid.z = centroid.z();

	ROS_INFO_STREAM("Calling pose estimation service call...");
	pose_estimation_client.call(srv.request, srv.response);
	ROS_INFO_STREAM("Done.");

	return 0;
}
