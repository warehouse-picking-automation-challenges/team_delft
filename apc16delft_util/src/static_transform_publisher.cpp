#include <dr_param/param.hpp>
#include <dr_eigen/param.hpp>
#include <dr_eigen/tf.hpp>
#include <dr_eigen/ros.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace apc16delft {

class StaticTransformPublisher {
	/// Node handle.
	ros::NodeHandle node;

	/// Old style tf broadcaster.
	tf::TransformBroadcaster tf_broadcaster;

	/// New style static tf broadcaster.
	tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

	/// Timer for re-publishing transform.
	ros::Timer publish_timer;

	/// The transform for the statis broadcaster.
	geometry_msgs::TransformStamped static_transform;

	/// The transform for the old tf broadcaster.
	tf::StampedTransform tf_transform;

	/// If true, use the new static tf broadcaster.
	bool use_static = true;

public:
	StaticTransformPublisher() : node("~") {
		// Load parameters.
		use_static = dr::getParam<bool>(node, "use_static", true);
		dr::Pose pose = dr::getParam<dr::Pose>(node, "pose");
		int rate = dr::getParam<int>("publish_rate", 10);

		// Convert to types understood by the tf broadcasters.
		static_transform.child_frame_id  = pose.header.child_frame;
		static_transform.header.frame_id = pose.header.parent_frame;
		static_transform.header.stamp    = ros::Time::now();
		static_transform.transform       = dr::toRosTransform(pose.isometry);
		tf_transform = dr::toTfStampedTransform(pose.isometry, pose.header.parent_frame, pose.header.child_frame);

		// Publish the transform and start a timer to keep publishing them.
		onPublishTimeout({});
		publish_timer = node.createTimer(ros::Rate(rate), &StaticTransformPublisher::onPublishTimeout, this);
	}

	/// Called when the publish timer expires.
	void onPublishTimeout(ros::TimerEvent const &) {
		if (use_static) {
			static_tf_broadcaster.sendTransform(static_transform);
		} else {
			tf_broadcaster.sendTransform(tf_transform);
		}
	}
};

}

int main(int argc, char * * argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME "_static_transform_publisher");
	apc16delft::StaticTransformPublisher static_transform_publisher;
	ros::spin();
}
