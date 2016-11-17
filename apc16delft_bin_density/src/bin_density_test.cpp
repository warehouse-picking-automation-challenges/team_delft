#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>
#include <ros/init.h>

#include <geometry_msgs/TransformStamped.h>

#include <apc16delft_msgs/EstimateBinPose.h>
#include <apc16delft_msgs/EstimateBinDensity.h>

using Point          = pcl::PointXYZRGB;
using PointCloud     = pcl::PointCloud<Point>;
using BinDensityReq  = apc16delft_msgs::EstimateBinDensity::Request;
using BinDensityRes  = apc16delft_msgs::EstimateBinDensity::Response;
using PoseEstReq     = apc16delft_msgs::EstimateBinPose::Request;
using PoseEstRes     = apc16delft_msgs::EstimateBinPose::Response;
using Viewer         = pcl::visualization::PCLVisualizer;

namespace apc16delft {

bool viewer_ = true;

/// Callback to continue and stop the viewer
void keyboardEventOccurred(pcl::visualization::KeyboardEvent const & event) {
	if (event.getKeySym() == "space" && event.keyDown()) {
		viewer_ = false;
	}
}

ros::NodeHandle * node_handle;

/// Calls the bin density service server with a transformed bin scene point cloud
void callBinDensity(PointCloud::Ptr transformed_scene){

	// Create a ros service client
	ros::ServiceClient bin_density_client = node_handle->serviceClient<apc16delft_msgs::EstimateBinDensity>("/bin_density/estimate_bin_density");
	apc16delft_msgs::EstimateBinDensity srv;

	// Fill request with bin cloud
	pcl::toROSMsg(*transformed_scene, srv.request.bin);

	// Fill request bin arguments (bin size varies)
	srv.request.bin_index = 1;

	// Fill radius request bin arguments
	srv.request.radius = 0.05;

	// Fill step size request bin arguments
	srv.request.step_size = 0.05;

	// Fill request with the pose of the bin
	Eigen::Translation3d trans(0.132253, -0.656527, 0.102018);
	Eigen::Quaterniond rot(-0.314278, 0.620836, 0.615473, 0.370116);
	Eigen::Isometry3d transform(trans * rot);
	tf::poseEigenToMsg(transform, srv.request.bin_pose);

	// Fill the request with the pose of the left camera lens
	Eigen::Isometry3d lens_pose = Eigen::Isometry3d::Identity();
	tf::poseEigenToMsg(lens_pose, srv.request.lens_pose);

	// Call the service
	if (!bin_density_client.call(srv.request, srv.response)) {
		ROS_ERROR_STREAM("Failed to call bin density. Is the bin_density node running?");
	};

}

} // namespace

int main(int argc, char * * argv) {
	if (argc < 2) {
		PCL_WARN("Usage: ./density scene.pcd\n");
		return 0;
	}
	ros::init(argc, argv, "bin_density_test");
	ROS_INFO_STREAM("Starting node: 'bin_density_test'");

	ros::NodeHandle node_handle;
	apc16delft::node_handle = &node_handle;

	// Load scene from file
	PointCloud::Ptr scene(new PointCloud);
	if (pcl::io::loadPCDFile(argv[1], *scene) == -1) {
		PCL_ERROR("No scene found. %s\n", argv[1]);
		return 0;
	}
	if (scene->empty()) {
		PCL_ERROR("Empty scene point cloud");
		return 0;
	}

	// Call service to bin density estimation
	apc16delft::callBinDensity(scene);
}
