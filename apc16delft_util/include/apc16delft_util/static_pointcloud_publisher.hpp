#pragma once

#include <apc16delft_msgs/PublishStaticPointCloud.h>
#include <dr_eigen/ros.hpp>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace apc16delft {

class StaticPointCloudPublisher {
private:
	/// Node handle.
	ros::NodeHandle nh_;

	sensor_msgs::PointCloud2 point_cloud_;
	sensor_msgs::PointCloud2 clear_cloud_;

	pcl::PointCloud<pcl::PointXYZ> pcl_clear_cloud_;
	bool clear_is_true_;

	ros::Publisher pub_pointcloud_;
	ros::ServiceServer pub_static_pointcloud_;
	ros::ServiceServer clear_static_pointcloud_;

	ros::Timer publish_timer_;

public:
	StaticPointCloudPublisher();

	void onPublishPointCloud(ros::TimerEvent const &);
	void onClearPointCloud(ros::TimerEvent const &);
	// Publish a cropped static point cloud that is provided by the coordinator.
	bool publishStaticPointCloud(apc16delft_msgs::PublishStaticPointCloud::Request &req, apc16delft_msgs::PublishStaticPointCloud::Response &res);
	bool clearStaticPointCloud(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res);
};
}
