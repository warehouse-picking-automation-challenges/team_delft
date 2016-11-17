#include "static_pointcloud_publisher.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <limits>

namespace apc16delft {

StaticPointCloudPublisher::StaticPointCloudPublisher() : nh_{"~"} {
	ros::Rate publish_rate{0.01};
	pub_pointcloud_          = nh_.advertise<sensor_msgs::PointCloud2>("static_pointcloud", 10, true);
	pub_static_pointcloud_   = nh_.advertiseService("publish_point_cloud", &StaticPointCloudPublisher::publishStaticPointCloud, this);
	clear_static_pointcloud_ = nh_.advertiseService("clear_static_point_cloud", &StaticPointCloudPublisher::clearStaticPointCloud, this);
	publish_timer_           = nh_.createTimer(publish_rate, &StaticPointCloudPublisher::onPublishPointCloud, this);
}

void StaticPointCloudPublisher::onPublishPointCloud(ros::TimerEvent const &) {
	ros::Time now = ros::Time::now();
	point_cloud_.header.stamp = now;
	pub_pointcloud_.publish(point_cloud_);
}

bool StaticPointCloudPublisher::publishStaticPointCloud(apc16delft_msgs::PublishStaticPointCloud::Request &req, apc16delft_msgs::PublishStaticPointCloud::Response &res) {
	//Publish
	pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud, transformed_point_cloud;
	pcl::fromROSMsg(req.point_cloud, pcl_point_cloud);
	Eigen::Isometry3d parent_frame = dr::toEigen(req.transform);
	Eigen::Affine3d affine_parent_frame{parent_frame.matrix()};

	pcl::transformPointCloud(pcl_point_cloud, transformed_point_cloud, affine_parent_frame);
	pcl::toROSMsg(transformed_point_cloud, point_cloud_);
	point_cloud_.header.frame_id = "world";
	clear_is_true_ = false;

	ros::TimerEvent rt;
	onPublishPointCloud(rt);
	return true;
}

bool StaticPointCloudPublisher::clearStaticPointCloud(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res) {
	ROS_INFO_STREAM("Before creating pcl_clear_cloud");
	pcl::fromROSMsg(point_cloud_, pcl_clear_cloud_);
	for (size_t i = 0; i < pcl_clear_cloud_.size(); ++i) {
		pcl_clear_cloud_[i] = pcl::PointXYZ(1000, 1000, 1000);
	}
	pcl::toROSMsg(pcl_clear_cloud_, point_cloud_);
	
	ros::TimerEvent rt;
	onPublishPointCloud(rt);
	return true;
}

}


int main(int argc, char * * argv) {
	ros::init(argc, argv, "point_cloud_publisher");
	apc16delft::StaticPointCloudPublisher static_pointcloud_publisher;
	ros::spin();
	return 0;
}
